using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Route understanding: a dense centerline polyline from race start toward
    /// the finish, plus continuous progress / lookahead / loss detection.
    ///
    /// Source priority:
    ///   1) GTA's active GPS route (GET_GPS_BLIP_ROUTE_FOUND /
    ///      GET_POS_ALONG_GPS_TYPE_ROUTE). The race already creates a routed
    ///      finish blip, so this is the real connected road path — not a guess.
    ///   2) Connected fallback walk (street-snapped stepping toward finish).
    ///   3) Straight origin->finish (last resort, still tracked for progress).
    ///
    /// Localization invariant (this pass):
    ///   Normal tracking answers "which segment near where I was last tick
    ///   am I on?", NOT "which nearby segment is spatially closest?".
    ///   Candidates are scored by distance + heading compatibility +
    ///   continuity around the expected station (predicted from vehicle
    ///   motion). Large AlongS jumps while the car barely moves are
    ///   projection jumps, not motion, and are rejected unless recovering.
    internal sealed class RaceRoute
    {
        public readonly List<Vector3> Points = new List<Vector3>();
        public readonly List<float> CumulativeS = new List<float>();
        public float TotalLength;
        public bool Built;

        // Where the centerline came from (telemetry / debug).
        public string Source = "None";
        public int GpsSamples;

        // Tracking state.
        public int NearestIndex;
        public float AlongS;        // progress along route (m from start)
        public float MaxS;          // high-water mark (detects going backwards)
        public float Lateral;       // signed: + = left of route direction
        public float DistToRoute;   // |lateral| (+ off-end distance)
        public float HeadingErrorDeg;
        public bool IsLost;
        public string LossReason = "";
        public int LostSinceMs;
        public int LastProgressMs;
        public float FinishGapEuclid;

        // --- Continuity-aware localization diagnostics / state.
        // Heading compatibility: normal tracking requires approximately
        // forward-aligned tangents. dot(routeDir, egoFwd) > 0.5  <=>  <60 deg.
        public const float SameDirDotMin = 0.5f;
        // Normal racing planning is invalid beyond ~50 deg. Severe beyond 60.
        public const float PlanInvalidHeadErrDeg = 50f;
        public const float SevereHeadErrDeg = 60f;
        public const float LostHeadErrDeg = 65f;

        public float PrevAlongS;
        public float ExpectedS;
        public int LastUpdateMs = -1;
        public float LastUpdateDtS = 0.1f;
        public string LocDetail = "";
        public float LocBestScore = 999f;
        public float LocSecondScore = 999f;
        public bool LocAmbiguous;
        public float LocJumpM;
        public float RouteHeadingDeg;
        public Vector3 RouteTangentDir = new Vector3(0f, 1f, 0f);

        public bool PlanInvalid => Math.Abs(HeadingErrorDeg) > PlanInvalidHeadErrDeg;
        public bool HeadingSevere => Math.Abs(HeadingErrorDeg) > SevereHeadErrDeg;

        private float lastHeadingForCircle;
        private float circleAccumDeg;
        private bool hasLastHeading;
        private Vector3 finish = Vector3.Zero;
        private int headingInvalidSinceMs = -100000;

        public Vector3 Finish => finish;

        public void Reset()
        {
            Points.Clear();
            CumulativeS.Clear();
            TotalLength = 0f;
            Built = false;
            Source = "None";
            GpsSamples = 0;
            NearestIndex = 0;
            AlongS = 0f;
            MaxS = 0f;
            Lateral = 0f;
            DistToRoute = 999f;
            HeadingErrorDeg = 0f;
            IsLost = false;
            LossReason = "";
            LostSinceMs = 0;
            LastProgressMs = 0;
            FinishGapEuclid = 0f;
            PrevAlongS = 0f;
            ExpectedS = 0f;
            LastUpdateMs = -1;
            LastUpdateDtS = 0.1f;
            LocDetail = "";
            LocBestScore = 999f;
            LocSecondScore = 999f;
            LocAmbiguous = false;
            LocJumpM = 0f;
            RouteHeadingDeg = 0f;
            RouteTangentDir = new Vector3(0f, 1f, 0f);
            hasLastHeading = false;
            circleAccumDeg = 0f;
            headingInvalidSinceMs = -100000;
        }

        public void Build(Vector3 origin, Vector3 finishIn)
        {
            Reset();
            finish = finishIn;
            DistToRoute = RaceMath.FlatDistance(origin, finishIn);
            LastProgressMs = Game.GameTime;

            // 1) Real connected GPS route first.
            try
            {
                List<Vector3> gps;
                string how;
                if (TryBuildFromGps(origin, finishIn, out gps, out how) && gps != null && gps.Count >= 2)
                {
                    foreach (var p in gps) Points.Add(p);
                    Source = how;
                    GpsSamples = gps.Count;
                    FinalizeGeometry();
                    return;
                }
            }
            catch { }

            // 2) Connected fallback walk (street-snapped stepping).
            try
            {
                if (BuildFallbackWalk(origin, finishIn))
                {
                    Source = "FallbackWalk";
                    FinalizeGeometry();
                    return;
                }
            }
            catch { }

            // 3) Last resort: straight line so progress tracking still exists.
            try
            {
                Points.Clear();
                Points.Add(origin);
                Points.Add(finishIn);
                Source = "StraightFallback";
                FinalizeGeometry();
            }
            catch
            {
                if (Points.Count == 0) Points.Add(origin);
                Points.Add(finishIn);
                Source = "StraightFallback";
                FinalizeGeometry();
            }
        }

        /// Immutable snapshot export (no GPS natives). The returned snapshot
        /// is a by-value copy of the current geometry for handoff.
        public RouteSnapshot ExportSnapshot()
        {
            var s = new RouteSnapshot
            {
                TotalLength = TotalLength,
                Source = Source,
                GpsSamples = GpsSamples,
                Finish = finish,
            };
            try
            {
                if (Points.Count > 0) s.Origin = Points[0];
            }
            catch { }
            try
            {
                foreach (var p in Points) s.Points.Add(p);
                foreach (var c in CumulativeS) s.CumulativeS.Add(c);
            }
            catch { }
            return s;
        }

        /// Import a previously acquired snapshot (no GPS natives). Recomputes
        /// cumulative arclength deterministically so validation and the brain
        /// share EXACTLY the accepted geometry.
        public void ImportSnapshot(RouteSnapshot snap)
        {
            Reset();
            if (snap == null || snap.Points == null || snap.Points.Count < 2)
            {
                finish = snap != null ? snap.Finish : Vector3.Zero;
                Source = "None";
                return;
            }
            finish = snap.Finish;
            foreach (var p in snap.Points) Points.Add(p);
            Source = snap.Source ?? "None";
            GpsSamples = snap.GpsSamples;
            try { DistToRoute = Points.Count > 0 ? RaceMath.FlatDistance(Points[0], finish) : 0f; } catch { }
            FinalizeGeometry();
        }

        /// GPS native readiness probe (no geometry sampling). Returns the raw
        /// GET_GPS_BLIP_ROUTE_FOUND flag plus route length for telemetry.
        public static bool GpsNativeReady(out int routeLen)
        {
            routeLen = 0;
            bool found = false;
            try { found = Function.Call<bool>(Hash.GET_GPS_BLIP_ROUTE_FOUND); }
            catch { return false; }
            try { routeLen = Function.Call<int>(Hash.GET_GPS_BLIP_ROUTE_LENGTH); }
            catch { routeLen = 0; }
            return found;
        }

        /// GPS-ONLY acquisition (no fallback). Samples the GTA GPS natives
        /// once and returns an immutable snapshot when globally plausible.
        /// Any failure is readiness (RETRY), never validation (REROLL):
        ///   native-not-ready / too-few-samples / implausible-geometry.
        /// Caller must NOT delete the blip or reroll the finish on failure.
        public static bool TryAcquireGpsSnapshot(Vector3 origin, Vector3 finishIn,
            out RouteSnapshot snap, out string detail)
        {
            snap = null;
            detail = "";
            bool found = false;
            int routeLen = 0;
            try { found = Function.Call<bool>(Hash.GET_GPS_BLIP_ROUTE_FOUND); }
            catch (Exception ex) { detail = $"native-exc found;{ex.Message}"; return false; }
            if (!found) { detail = "native-not-ready"; return false; }
            try { routeLen = Function.Call<int>(Hash.GET_GPS_BLIP_ROUTE_LENGTH); }
            catch { routeLen = 0; }

            int[] types = { 1, 0, 2 };
            string attempts = $"nativeFound=1;routeLen={routeLen}";
            foreach (int t in types)
            {
                List<Vector3> byDist = null;
                List<Vector3> byIdx = null;
                try { byDist = SampleGpsByDistance(origin, finishIn, routeLen, t); }
                catch { byDist = null; }
                int nDist = byDist != null ? byDist.Count : -1;
                string whyDist = "";
                bool okDist = false;
                try { okDist = IsPlausibleRouteWithReason(byDist, origin, finishIn, out whyDist); }
                catch { okDist = false; whyDist = "exc"; }
                if (okDist)
                {
                    var pts = ResamplePolyline(byDist, 5f);
                    snap = BuildSnapshotFromPoints(origin, finishIn, pts, "GpsDist(t" + t + ")");
                    detail = $"{attempts};t={t};byDist n={nDist} OK;src={snap.Source};pts={snap.Points.Count};len={snap.TotalLength:F0}";
                    return true;
                }
                try { byIdx = SampleGpsByIndex(origin, finishIn, routeLen, t); }
                catch { byIdx = null; }
                int nIdx = byIdx != null ? byIdx.Count : -1;
                string whyIdx = "";
                bool okIdx = false;
                try { okIdx = IsPlausibleRouteWithReason(byIdx, origin, finishIn, out whyIdx); }
                catch { okIdx = false; whyIdx = "exc"; }
                if (okIdx)
                {
                    var pts = ResamplePolyline(byIdx, 5f);
                    snap = BuildSnapshotFromPoints(origin, finishIn, pts, "GpsIdx(t" + t + ")");
                    detail = $"{attempts};t={t};byDist n={nDist} fail({whyDist});byIdx n={nIdx} OK;src={snap.Source};pts={snap.Points.Count};len={snap.TotalLength:F0}";
                    return true;
                }
                attempts += $";t{t}:dist n={nDist}({whyDist}) idx n={nIdx}({whyIdx})";
            }
            detail = attempts + ";no-plausible-gps";
            return false;
        }

        private static RouteSnapshot BuildSnapshotFromPoints(Vector3 origin, Vector3 finishIn,
            List<Vector3> pts, string how)
        {
            var s = new RouteSnapshot
            {
                Source = how,
                Finish = finishIn,
                Origin = origin,
                GpsSamples = pts != null ? pts.Count : 0,
            };
            if (pts != null)
            {
                foreach (var p in pts) s.Points.Add(p);
            }
            float acc = 0f;
            s.CumulativeS.Add(0f);
            for (int i = 1; i < s.Points.Count; i++)
            {
                acc += RaceMath.FlatDistance(s.Points[i - 1], s.Points[i]);
                s.CumulativeS.Add(acc);
            }
            s.TotalLength = acc;
            return s;
        }

        private static bool IsPlausibleRouteWithReason(List<Vector3> pts, Vector3 origin,
            Vector3 finishIn, out string why)
        {
            why = "null";
            if (pts == null) return false;
            if (pts.Count < 5) { why = $"too-few-samples n={pts.Count}"; return false; }
            float dStart = RaceMath.FlatDistance(pts[0], origin);
            float dEnd = RaceMath.FlatDistance(pts[pts.Count - 1], finishIn);
            float closestStart = float.MaxValue;
            float closestEnd = float.MaxValue;
            for (int i = 0; i < System.Math.Min(pts.Count, 12); i++)
                closestStart = System.Math.Min(closestStart, RaceMath.FlatDistance(pts[i], origin));
            for (int i = System.Math.Max(0, pts.Count - 12); i < pts.Count; i++)
                closestEnd = System.Math.Min(closestEnd, RaceMath.FlatDistance(pts[i], finishIn));
            if (System.Math.Min(dStart, closestStart) > 220f) { why = $"start-far dStart={dStart:F0} closest={closestStart:F0}"; return false; }
            if (System.Math.Min(dEnd, closestEnd) > 260f) { why = $"end-far dEnd={dEnd:F0} closest={closestEnd:F0}"; return false; }
            float straight = RaceMath.FlatDistance(origin, finishIn);
            float len = 0f;
            for (int i = 1; i < pts.Count; i++) len += RaceMath.FlatDistance(pts[i - 1], pts[i]);
            if (len < straight * 0.6f) { why = $"too-short len={len:F0} straight={straight:F0}"; return false; }
            if (len > straight * 4f + 1500f) { why = $"too-long len={len:F0} straight={straight:F0}"; return false; }
            if (len < 60f) { why = $"len-too-small {len:F0}"; return false; }
            why = $"ok len={len:F0}";
            return true;
        }

        /// Called by RaceBrain during the first seconds if Build() fell back
        /// before the GPS route existed (blip route takes a frame or two to
        /// compute). Returns true when an upgrade happened.
        /// Invariant: switching FallbackWalk-&gt;GPS must NOT redefine
        /// progress/heading underneath the planner.
        public bool TryUpgradeToGps(Vector3 egoPos)
        {
            string dummy;
            return TryUpgradeToGps(egoPos, 0f, 0f, Game.GameTime, 7f, out dummy);
        }

        public bool TryUpgradeToGps(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            int nowMs, float corridorHalfWidth, out string upgradeLog)
        {
            upgradeLog = "";
            try
            {
                if (Built && Source.StartsWith("Gps")) return false;
                float oldS = AlongS;
                float oldMax = MaxS;
                float oldLat = Lateral;
                float oldHeadErr = HeadingErrorDeg;
                float oldHead = 0f;
                string oldSrc = Source ?? "?";
                int oldPts = Points.Count;
                float oldLen = TotalLength;
                try { oldHead = HeadingAtS(oldS); } catch { }
                List<Vector3> gps;
                string how;
                if (!TryBuildFromGps(Points.Count > 0 ? Points[0] : egoPos, finish, out gps, out how))
                    return false;
                if (gps == null || gps.Count < 2) return false;
                Points.Clear();
                foreach (var p in gps) Points.Add(p);
                Source = how;
                GpsSamples = gps.Count;
                FinalizeGeometry();
                // Clean reproject onto the NEW geometry: require a
                // forward-compatible segment (dot > 0.5). Never accept a
                // sideways projection as healthy and never silently jump to
                // s=0. If no forward-compatible segment exists, reject the
                // upgrade (caller keeps the old route).
                try
                {
                    int bestSeg = -1;
                    float bestDist = float.MaxValue;
                    RaceMath.Projection bestPr = new RaceMath.Projection();
                    Vector3 bestDir = new Vector3(0f, 1f, 0f);
                    var egoFwd = RaceMath.VectorFromHeading(egoHeadingDeg);
                    for (int pass = 0; pass < 2; pass++)
                    {
                        for (int i = 0; i < Points.Count - 1; i++)
                        {
                            var a = Points[i];
                            var b = Points[i + 1];
                            var segDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                            float dot = RaceMath.FlatDot(segDir, egoFwd);
                            if (pass == 0 && dot <= SameDirDotMin) continue;
                            if (pass == 1 && dot <= 0f) continue;
                            var pr = RaceMath.ProjectOnSegment(egoPos, a, b);
                            if (pr.Dist < bestDist)
                            {
                                bestDist = pr.Dist;
                                bestSeg = i;
                                bestPr = pr;
                                bestDir = segDir;
                            }
                        }
                        if (bestSeg >= 0 && bestDist < 60f) break;
                        if (pass == 0) { bestSeg = -1; bestDist = float.MaxValue; }
                    }
                    if (bestSeg >= 0)
                    {
                        NearestIndex = bestSeg;
                        AlongS = CumulativeS[bestSeg] + bestPr.Along;
                        PrevAlongS = AlongS;
                        ExpectedS = AlongS;
                        DistToRoute = bestPr.Dist;
                        Lateral = RaceMath.FlatCross(bestDir, new Vector3(egoPos.X - bestPr.Closest.X, egoPos.Y - bestPr.Closest.Y, 0f));
                        float newHead = RaceMath.HeadingFromVector(bestDir);
                        HeadingErrorDeg = RaceMath.HeadingDiffDeg(newHead, egoHeadingDeg);
                        RouteHeadingDeg = newHead;
                        RouteTangentDir = bestDir;
                        LocBestScore = bestDist;
                        LocSecondScore = 999f;
                        LocAmbiguous = false;
                        LocJumpM = 0f;
                        LocDetail = $"upgrade seg={bestSeg} s={AlongS:F1} dist={bestDist:F1} headErr={HeadingErrorDeg:F0}";
                    }
                    else
                    {
                        return false;
                    }
                }
                catch
                {
                    return false;
                }
                MaxS = AlongS;
                LastProgressMs = nowMs;
                LastUpdateMs = nowMs;
                FinishGapEuclid = RaceMath.FlatDistance(egoPos, finish);
                hasLastHeading = false;
                circleAccumDeg = 0f;
                headingInvalidSinceMs = -100000;
                IsLost = false;
                LossReason = "";
                float newHeadAt = 0f;
                try { newHeadAt = HeadingAtS(AlongS); } catch { }
                float progJump = AlongS - oldS;
                float headJump = RaceMath.HeadingDiffDeg(newHeadAt, oldHead);
                try
                {
                    upgradeLog = $"old={oldSrc};oldPts={oldPts};oldLen={oldLen:F0};oldS={oldS:F0};oldMax={oldMax:F0};oldLat={oldLat:F1};oldHeadErr={oldHeadErr:F0};oldHead={oldHead:F0};"
                        + $"new={Source};newPts={Points.Count};newLen={TotalLength:F0};newS={AlongS:F0};newLat={Lateral:F1};newHeadErr={HeadingErrorDeg:F0};newHead={newHeadAt:F0};"
                        + $"dS={progJump:F0};dHead={headJump:F0};dist={DistToRoute:F1};egoSpd={egoSpeed:F1}";
                }
                catch { upgradeLog = $"old={oldSrc};new={Source}"; }
                return true;
            }
            catch { return false; }
        }

        /// Re-acquire the active GPS geometry using the rival's CURRENT
        /// pose as the validation origin. This is used only after local
        /// trajectory planning has been unable to produce a route-valid path
        /// and the heading error is diverging.
        ///
        /// The replacement is atomic: a candidate snapshot is fully validated
        /// against the current pose/heading before this route is mutated.
        public bool TryRebuildFromCurrentGps(
            Vector3 egoPos,
            float egoHeadingDeg,
            float egoSpeed,
            int nowMs,
            float corridorHalfWidth,
            out string rebuildLog)
        {
            rebuildLog = "";
            try
            {
                RouteSnapshot snap;
                string acquire;
                if (!TryAcquireGpsSnapshot(
                    egoPos, finish, out snap, out acquire)
                    || snap == null
                    || snap.Points == null
                    || snap.Points.Count < 2)
                {
                    rebuildLog = "acquire-fail;" + acquire;
                    return false;
                }

                Vector3 egoFwd = RaceMath.FlatNormalize(
                    RaceMath.VectorFromHeading(egoHeadingDeg));

                int bestSeg = -1;
                float bestDist = float.MaxValue;
                float bestScore = float.MaxValue;
                RaceMath.Projection bestPr = new RaceMath.Projection();
                Vector3 bestDir = new Vector3(0f, 1f, 0f);

                // First require the normal >0.5 heading compatibility. A second
                // pass allows any forward-facing segment, but we still reject
                // the snapshot below if the resulting heading is too severe.
                for (int pass = 0; pass < 2; pass++)
                {
                    for (int i = 0; i < snap.Points.Count - 1; i++)
                    {
                        Vector3 a = snap.Points[i];
                        Vector3 b = snap.Points[i + 1];
                        Vector3 dir = RaceMath.FlatNormalize(
                            new Vector3(
                                b.X - a.X,
                                b.Y - a.Y,
                                0f));
                        float dot = RaceMath.FlatDot(dir, egoFwd);
                        if (pass == 0 && dot <= SameDirDotMin) continue;
                        if (pass == 1 && dot <= 0f) continue;

                        RaceMath.Projection pr =
                            RaceMath.ProjectOnSegment(egoPos, a, b);
                        float segHeading =
                            RaceMath.HeadingFromVector(dir);
                        float headErr = Math.Abs(
                            RaceMath.HeadingDiffDeg(
                                segHeading, egoHeadingDeg));
                        float score = pr.Dist + headErr * 0.20f;
                        if (score >= bestScore) continue;

                        bestScore = score;
                        bestSeg = i;
                        bestDist = pr.Dist;
                        bestPr = pr;
                        bestDir = dir;
                    }

                    if (bestSeg >= 0 && bestDist <= 45f) break;
                    if (pass == 0)
                    {
                        bestSeg = -1;
                        bestDist = float.MaxValue;
                        bestScore = float.MaxValue;
                    }
                }

                float maxProjectionDist = RaceMath.Clamp(
                    corridorHalfWidth + 20f, 25f, 45f);
                if (bestSeg < 0 || bestDist > maxProjectionDist)
                {
                    rebuildLog = $"snapshot-not-near;dist={bestDist:F1};limit={maxProjectionDist:F1};{acquire}";
                    return false;
                }

                float newHeading =
                    RaceMath.HeadingFromVector(bestDir);
                float newHeadErr = RaceMath.HeadingDiffDeg(
                    newHeading, egoHeadingDeg);
                if (Math.Abs(newHeadErr) > PlanInvalidHeadErrDeg)
                {
                    rebuildLog = $"snapshot-heading-incompatible;dist={bestDist:F1};headErr={newHeadErr:F0};{acquire}";
                    return false;
                }

                string oldSource = Source ?? "?";
                float oldS = AlongS;
                float oldDist = DistToRoute;
                float oldHeadErr = HeadingErrorDeg;

                ImportSnapshot(snap);

                NearestIndex = bestSeg;
                AlongS = CumulativeS[bestSeg] + bestPr.Along;
                PrevAlongS = AlongS;
                ExpectedS = AlongS;
                MaxS = AlongS;
                DistToRoute = bestDist;
                Lateral = RaceMath.FlatCross(
                    bestDir,
                    new Vector3(
                        egoPos.X - bestPr.Closest.X,
                        egoPos.Y - bestPr.Closest.Y,
                        0f));
                HeadingErrorDeg = newHeadErr;
                RouteHeadingDeg = newHeading;
                RouteTangentDir = bestDir;
                LocBestScore = bestDist;
                LocSecondScore = 999f;
                LocAmbiguous = false;
                LocJumpM = 0f;
                LocDetail = $"rebuild seg={bestSeg} s={AlongS:F1} dist={bestDist:F1} headErr={newHeadErr:F0}";
                LastProgressMs = nowMs;
                LastUpdateMs = nowMs;
                FinishGapEuclid = RaceMath.FlatDistance(
                    egoPos, finish);
                IsLost = false;
                LossReason = "";
                LostSinceMs = 0;
                hasLastHeading = false;
                circleAccumDeg = 0f;
                headingInvalidSinceMs = -100000;

                rebuildLog =
                    $"old={oldSource};oldS={oldS:F0};oldDist={oldDist:F1};oldHeadErr={oldHeadErr:F0};"
                    + $"new={Source};newS={AlongS:F0};newDist={DistToRoute:F1};newHeadErr={HeadingErrorDeg:F0};"
                    + $"pts={Points.Count};len={TotalLength:F0};egoSpd={egoSpeed:F1};{acquire}";
                return true;
            }
            catch (Exception ex)
            {
                try { rebuildLog = "exc:" + ex.Message; } catch { }
                return false;
            }
        }

        private void FinalizeGeometry()
        {
            CumulativeS.Clear();
            float s = 0f;
            CumulativeS.Add(0f);
            for (int i = 1; i < Points.Count; i++)
            {
                s += RaceMath.FlatDistance(Points[i - 1], Points[i]);
                CumulativeS.Add(s);
            }
            TotalLength = s;
            Built = Points.Count >= 2;
            LastProgressMs = Game.GameTime;
        }

        // ------------------------------------------------------------------
        // GPS sampling.
        //
        // Natives (from nativedb):
        //   BOOL GET_GPS_BLIP_ROUTE_FOUND()
        //   int  GET_GPS_BLIP_ROUTE_LENGTH()
        //   BOOL GET_POS_ALONG_GPS_TYPE_ROUTE(Vector3* out, BOOL p1, float p2, int p3)
        //        p3 in {0,1,2} (route type). p2 semantics are undocumented —
        //        most likely distance-along-route in meters. We probe distance
        //        first, then index fallback, and validate geometrically so a
        //        wrong interpretation can never silently corrupt the route.
        //
        // Geometry: sample densely (~5 m) so interpolation never cuts city
        // junctions/bends into bad tangents. Expensive downstream systems may
        // resample/coarsen separately. Type 1 (mission/blip driving route) is
        // tried first and wins when plausible; alternates are only probed when
        // type 1 fails validation.
        // ------------------------------------------------------------------
        private static bool TryBuildFromGps(Vector3 origin, Vector3 finishIn,
            out List<Vector3> pts, out string how)
        {
            pts = null;
            how = "None";
            bool found = false;
            try { found = Function.Call<bool>(Hash.GET_GPS_BLIP_ROUTE_FOUND); }
            catch { return false; }
            if (!found) return false;

            int routeLen = 0;
            try { routeLen = Function.Call<int>(Hash.GET_GPS_BLIP_ROUTE_LENGTH); }
            catch { routeLen = 0; }

            int[] types = { 1, 0, 2 };
            foreach (int t in types)
            {
                var byDist = SampleGpsByDistance(origin, finishIn, routeLen, t);
                if (IsPlausibleRoute(byDist, origin, finishIn))
                {
                    pts = ResamplePolyline(byDist, 5f);
                    how = "GpsDist(t" + t + ")";
                    return true;
                }
                var byIdx = SampleGpsByIndex(origin, finishIn, routeLen, t);
                if (IsPlausibleRoute(byIdx, origin, finishIn))
                {
                    pts = ResamplePolyline(byIdx, 5f);
                    how = "GpsIdx(t" + t + ")";
                    return true;
                }
            }
            return false;
        }

        private static List<Vector3> SampleGpsByDistance(Vector3 origin, Vector3 finishIn, int routeLen, int type)
        {
            var outPts = new List<Vector3>();
            float maxD = 9000f;
            if (routeLen > 200 && routeLen < 15000) maxD = routeLen + 400f;
            float step = 5f;
            int failStreak = 0;
            Vector3 last = Vector3.Zero;
            bool haveLast = false;
            for (float d = 0f; d <= maxD; d += step)
            {
                Vector3 p;
                if (!TryGpsPos(true, d, type, out p))
                {
                    if (!TryGpsPos(false, d, type, out p))
                    {
                        failStreak++;
                        if (failStreak >= 12 && outPts.Count >= 4) break;
                        if (d > 1500f && outPts.Count < 3) break;
                        continue;
                    }
                }
                failStreak = 0;
                if (p == Vector3.Zero) continue;
                if (haveLast)
                {
                    float gap = RaceMath.FlatDistance(last, p);
                    if (gap < 1.5f) continue;
                    if (gap > 400f) break;
                }
                outPts.Add(p);
                last = p;
                haveLast = true;
                if (RaceMath.FlatDistance(p, finishIn) < 35f && outPts.Count > 4) break;
                if (outPts.Count > 1200) break;
            }
            return outPts;
        }

        private static List<Vector3> SampleGpsByIndex(Vector3 origin, Vector3 finishIn, int routeLen, int type)
        {
            var outPts = new List<Vector3>();
            if (routeLen < 2 || routeLen > 2000) return outPts;
            for (int i = 0; i < routeLen; i++)
            {
                Vector3 p;
                if (!TryGpsPos(true, (float)i, type, out p)) continue;
                if (p == Vector3.Zero) continue;
                if (outPts.Count > 0 && RaceMath.FlatDistance(outPts[outPts.Count - 1], p) < 2f) continue;
                outPts.Add(p);
                if (outPts.Count > 1200) break;
            }
            return outPts;
        }

        private static bool TryGpsPos(bool p1, float p2, int type, out Vector3 pos)
        {
            pos = Vector3.Zero;
            try
            {
                var outArg = new OutputArgument();
                bool ok = Function.Call<bool>(Hash.GET_POS_ALONG_GPS_TYPE_ROUTE, outArg, p1, p2, type);
                if (!ok) return false;
                pos = outArg.GetResult<Vector3>();
                if (pos == Vector3.Zero) return false;
                if (Math.Abs(pos.X) > 9000f || Math.Abs(pos.Y) > 9000f) return false;
                return true;
            }
            catch { return false; }
        }

        private static bool IsPlausibleRoute(List<Vector3> pts, Vector3 origin, Vector3 finishIn)
        {
            if (pts == null || pts.Count < 5) return false;
            float dStart = RaceMath.FlatDistance(pts[0], origin);
            float dEnd = RaceMath.FlatDistance(pts[pts.Count - 1], finishIn);
            float closestStart = float.MaxValue;
            float closestEnd = float.MaxValue;
            for (int i = 0; i < Math.Min(pts.Count, 12); i++)
                closestStart = Math.Min(closestStart, RaceMath.FlatDistance(pts[i], origin));
            for (int i = Math.Max(0, pts.Count - 12); i < pts.Count; i++)
                closestEnd = Math.Min(closestEnd, RaceMath.FlatDistance(pts[i], finishIn));
            if (Math.Min(dStart, closestStart) > 220f) return false;
            if (Math.Min(dEnd, closestEnd) > 260f) return false;
            float straight = RaceMath.FlatDistance(origin, finishIn);
            float len = 0f;
            for (int i = 1; i < pts.Count; i++) len += RaceMath.FlatDistance(pts[i - 1], pts[i]);
            if (len < straight * 0.6f) return false;
            if (len > straight * 4f + 1500f) return false;
            if (len < 60f) return false;
            return true;
        }

        /// LOCAL START validity: global plausibility (start-near / end-near /
        /// length) is not enough. Before accepting a route for a rival,
        /// verify near the rival there is a close forward-compatible
        /// projection and the first tens of meters form a continuous forward
        /// path with no absurd immediate branch/turn.
        public bool ValidateStart(Vector3 egoPos, float egoHeadingDeg, out string reason)
        {
            reason = "not-built";
            try
            {
                if (!Built || Points.Count < 2) return false;
                var egoFwd = RaceMath.VectorFromHeading(egoHeadingDeg);
                int bestSeg = -1;
                float bestScore = float.MaxValue;
                float bestDist = float.MaxValue;
                Vector3 bestDir = new Vector3(0f, 1f, 0f);
                float bestS = 0f;
                for (int i = 0; i < Points.Count - 1; i++)
                {
                    var a = Points[i];
                    var b = Points[i + 1];
                    var segDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                    float dot = RaceMath.FlatDot(segDir, egoFwd);
                    if (dot <= SameDirDotMin) continue;
                    var pr = RaceMath.ProjectOnSegment(egoPos, a, b);
                    float score = pr.Dist + (1f - dot) * 12f;
                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestSeg = i;
                        bestDist = pr.Dist;
                        bestDir = segDir;
                        bestS = CumulativeS[i] + pr.Along;
                    }
                }
                if (bestSeg < 0) { reason = "no-fwd-segment"; return false; }
                if (bestDist > 20f) { reason = $"too-far dist={bestDist:F1} seg={bestSeg}"; return false; }
                float routeHead = RaceMath.HeadingFromVector(bestDir);
                float headErr = Math.Abs(RaceMath.HeadingDiffDeg(routeHead, egoHeadingDeg));
                if (headErr > 45f) { reason = $"head-err {headErr:F0} seg={bestSeg} dist={bestDist:F1}"; return false; }
                float h0 = HeadingAtS(bestS);
                float h1 = HeadingAtS(bestS + 10f);
                float h2 = HeadingAtS(bestS + 20f);
                float h3 = HeadingAtS(bestS + 30f);
                if (Math.Abs(RaceMath.HeadingDiffDeg(h1, h0)) > 35f ||
                    Math.Abs(RaceMath.HeadingDiffDeg(h2, h0)) > 45f ||
                    Math.Abs(RaceMath.HeadingDiffDeg(h3, h0)) > 60f)
                {
                    reason = $"immediate-branch h0={h0:F0} h1={h1:F0} h2={h2:F0} h3={h3:F0}";
                    return false;
                }
                for (int k = 1; k <= 3; k++)
                {
                    float d = k * 10f;
                    Vector3 pt = PointAtS(bestS + d);
                    var to = new Vector3(pt.X - egoPos.X, pt.Y - egoPos.Y, 0f);
                    if (RaceMath.FlatLength(to) < 2f) continue;
                    float bearing = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(to));
                    float bErr = Math.Abs(RaceMath.HeadingDiffDeg(bearing, egoHeadingDeg));
                    if (bErr > 70f) { reason = $"not-forward d={d:F0} bErr={bErr:F0}"; return false; }
                }
                float curv = CurvatureAtS(bestS + 15f, 30f);
                if (curv > 0.08f) { reason = $"sharp-start curv={curv:F3}"; return false; }
                reason = $"ok seg={bestSeg} s={bestS:F0} dist={bestDist:F1} headErr={headErr:F0} h0={h0:F0}";
                return true;
            }
            catch (Exception ex)
            {
                try { reason = "exc:" + ex.Message; } catch { }
                return false;
            }
        }

        private static List<Vector3> ResamplePolyline(List<Vector3> src, float step)
        {
            var dst = new List<Vector3>();
            if (src == null || src.Count == 0) return dst;
            dst.Add(src[0]);
            float acc = 0f;
            for (int i = 1; i < src.Count; i++)
            {
                float seg = RaceMath.FlatDistance(src[i - 1], src[i]);
                if (seg < 0.01f) continue;
                float t = step - acc;
                var a = src[i - 1];
                var b = src[i];
                while (t < seg)
                {
                    float f = t / seg;
                    dst.Add(new Vector3(
                        a.X + (b.X - a.X) * f,
                        a.Y + (b.Y - a.Y) * f,
                        a.Z + (b.Z - a.Z) * f));
                    t += step;
                }
                acc = seg - (t - step);
                if (acc < 0f) acc = 0f;
                if (acc >= step) acc -= step;
            }
            var last = src[src.Count - 1];
            if (dst.Count == 0 || RaceMath.FlatDistance(dst[dst.Count - 1], last) > 1f)
                dst.Add(last);
            return dst;
        }

        private static bool BuildFallbackWalkInto(Vector3 origin, Vector3 finishIn, List<Vector3> pts)
        {
            pts.Clear();
            var flat = new Vector3(finishIn.X - origin.X, finishIn.Y - origin.Y, 0f);
            if (RaceMath.FlatLength(flat) < 1f) flat = new Vector3(0f, 1f, 0f);
            flat = RaceMath.FlatNormalize(flat);

            Vector3 cursor = origin;
            pts.Add(SnapToStreet(origin));
            for (int i = 0; i < 160; i++)
            {
                float remaining = RaceMath.FlatDistance(cursor, finishIn);
                if (remaining < 30f) break;
                float step = Math.Min(30f, Math.Max(15f, remaining * 0.4f));
                var toF = RaceMath.FlatNormalize(new Vector3(finishIn.X - cursor.X, finishIn.Y - cursor.Y, 0f));
                var probe = new Vector3(cursor.X + toF.X * step, cursor.Y + toF.Y * step, cursor.Z);
                Vector3 snapped = SnapToStreet(probe);
                if (RaceMath.FlatDistance(snapped, cursor) < 4f)
                {
                    var probe2 = new Vector3(probe.X + toF.X * 20f, probe.Y + toF.Y * 20f, probe.Z);
                    snapped = SnapToStreet(probe2);
                    if (RaceMath.FlatDistance(snapped, cursor) < 4f) break;
                }
                pts.Add(snapped);
                cursor = snapped;
            }
            pts.Add(SnapToStreet(finishIn));
            return pts.Count >= 2;
        }

        private bool BuildFallbackWalk(Vector3 origin, Vector3 finishIn)
        {
            var tmp = new List<Vector3>();
            if (!BuildFallbackWalkInto(origin, finishIn, tmp)) return false;
            Points.Clear();
            foreach (var p in tmp) Points.Add(p);
            return Points.Count >= 2;
        }

        public void Update(Vector3 egoPos, float egoHeadingDeg, float speed, int nowMs, float corridorHalfWidth)
        {
            if (!Built || Points.Count < 2) return;
            FinishGapEuclid = RaceMath.FlatDistance(egoPos, finish);

            float prevS = AlongS;
            float dtS = 0.1f;
            if (LastUpdateMs >= 0)
            {
                dtS = (nowMs - LastUpdateMs) / 1000f;
                if (dtS < 0f) dtS = 0f;
                if (dtS > 0.6f) dtS = 0.6f;
            }
            LastUpdateDtS = dtS;

            // Predicted progress from vehicle motion along the previous
            // route tangent. Sideways motion predicts ~0 forward progress.
            float cosF = 1f;
            try
            {
                float e = Math.Abs(HeadingErrorDeg) * (float)Math.PI / 180f;
                if (e > 1.2f) e = 1.2f;
                cosF = (float)Math.Cos(e);
                if (cosF < 0f) cosF = 0f;
            }
            catch { cosF = 1f; }
            float expectedS = prevS + Math.Max(0f, speed) * dtS * cosF;
            ExpectedS = expectedS;
            PrevAlongS = prevS;

            var egoFwd = RaceMath.VectorFromHeading(egoHeadingDeg);

            int lo, hi;
            bool fullSearch = IsLost || LastUpdateMs < 0;
            if (fullSearch) { lo = 0; hi = Points.Count - 2; }
            else
            {
                lo = Math.Max(0, NearestIndex - 2);
                hi = Math.Min(Points.Count - 2, NearestIndex + 6);
            }

            int bestSeg = -1;
            float bestDist = float.MaxValue;
            float bestScore = float.MaxValue;
            float secondScore = float.MaxValue;
            RaceMath.Projection bestProj = new RaceMath.Projection();
            Vector3 bestDir = new Vector3(0f, 1f, 0f);

            for (int pass = 0; pass < 2; pass++)
            {
                for (int i = lo; i <= hi; i++)
                {
                    var a = Points[i];
                    var b = Points[i + 1];
                    var segDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                    float dot = RaceMath.FlatDot(segDir, egoFwd);
                    if (pass == 0 && dot <= SameDirDotMin) continue;
                    if (pass == 1 && dot <= 0f) continue;
                    var pr = RaceMath.ProjectOnSegment(egoPos, a, b);
                    float s = CumulativeS[i] + pr.Along;
                    float cont = Math.Abs(s - expectedS) * 0.8f;
                    float headPen = (1f - dot) * 12f;
                    float score = pr.Dist + headPen + cont;
                    if (score < bestScore)
                    {
                        secondScore = bestScore;
                        bestScore = score;
                        bestDist = pr.Dist;
                        bestSeg = i;
                        bestProj = pr;
                        bestDir = segDir;
                    }
                    else if (score < secondScore)
                    {
                        secondScore = score;
                    }
                }
                if (bestSeg >= 0 && bestDist < 60f) break;
                if (pass == 0)
                {
                    bestSeg = -1;
                    bestDist = float.MaxValue;
                    bestScore = float.MaxValue;
                    secondScore = float.MaxValue;
                }
            }

            if (bestSeg < 0)
            {
                LocDetail = $"no-fwd-seg prev={prevS:F1} exp={expectedS:F1}";
                LocBestScore = 999f;
                LocSecondScore = 999f;
                LocAmbiguous = false;
                LastUpdateMs = nowMs;
                SetLost(nowMs, "NoFwdSegment", speed);
                return;
            }

            float newS = CumulativeS[bestSeg] + bestProj.Along;

            // Jump guard: normal tracking must not jump many meters to another
            // nearby segment while the car barely moves. If the winner is far
            // from expected, prefer the best candidate inside the continuity
            // window when one exists at reasonable distance.
            if (!fullSearch && Math.Abs(newS - expectedS) > 12f)
            {
                int inSeg = -1;
                float inDist = float.MaxValue;
                float inScore = float.MaxValue;
                float inSecond = float.MaxValue;
                RaceMath.Projection inProj = new RaceMath.Projection();
                Vector3 inDir = bestDir;
                for (int i = lo; i <= hi; i++)
                {
                    var a = Points[i];
                    var b = Points[i + 1];
                    var segDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                    float dot = RaceMath.FlatDot(segDir, egoFwd);
                    if (dot <= SameDirDotMin) continue;
                    var pr = RaceMath.ProjectOnSegment(egoPos, a, b);
                    float s = CumulativeS[i] + pr.Along;
                    if (Math.Abs(s - expectedS) > 12f) continue;
                    float score = pr.Dist + (1f - dot) * 12f + Math.Abs(s - expectedS) * 0.8f;
                    if (score < inScore)
                    {
                        inSecond = inScore;
                        inScore = score;
                        inDist = pr.Dist;
                        inSeg = i;
                        inProj = pr;
                        inDir = segDir;
                    }
                    else if (score < inSecond) inSecond = score;
                }
                if (inSeg >= 0 && inDist < 45f)
                {
                    LocDetail = $"jump-reject winS={newS:F1}->inS={CumulativeS[inSeg] + inProj.Along:F1} prev={prevS:F1} exp={expectedS:F1} winDist={bestDist:F1} inDist={inDist:F1}";
                    bestSeg = inSeg;
                    bestDist = inDist;
                    bestProj = inProj;
                    bestDir = inDir;
                    bestScore = inScore;
                    secondScore = inSecond;
                    newS = CumulativeS[bestSeg] + bestProj.Along;
                }
                else
                {
                    LocDetail = $"jump-no-inside winS={newS:F1} prev={prevS:F1} exp={expectedS:F1} dist={bestDist:F1} HOLD+LOST";
                    LocBestScore = bestScore;
                    LocSecondScore = secondScore;
                    LocAmbiguous = (secondScore - bestScore) < 4f;
                    LocJumpM = newS - prevS;
                    LastUpdateMs = nowMs;
                    SetLost(nowMs, "LocJump", speed);
                    return;
                }
            }

            NearestIndex = bestSeg;
            AlongS = newS;
            LocJumpM = AlongS - prevS;
            LocBestScore = bestScore;
            LocSecondScore = secondScore;
            LocAmbiguous = (secondScore - bestScore) < 4f;
            DistToRoute = bestDist;
            Lateral = RaceMath.FlatCross(bestDir, new Vector3(egoPos.X - bestProj.Closest.X, egoPos.Y - bestProj.Closest.Y, 0f));
            float routeHeading = RaceMath.HeadingFromVector(bestDir);
            HeadingErrorDeg = RaceMath.HeadingDiffDeg(routeHeading, egoHeadingDeg);
            RouteHeadingDeg = routeHeading;
            RouteTangentDir = bestDir;
            LastUpdateMs = nowMs;
            try
            {
                float dotBest = RaceMath.FlatDot(bestDir, egoFwd);
                LocDetail = $"seg={bestSeg} s={AlongS:F1} prev={prevS:F1} exp={expectedS:F1} jump={LocJumpM:F1} dist={bestDist:F1} dot={dotBest:F2} headErr={HeadingErrorDeg:F0} scores={bestScore:F1}/{secondScore:F1}{(LocAmbiguous ? ";AMBIG" : "")}";
            }
            catch { LocDetail = $"seg={bestSeg} s={AlongS:F1}"; }

            if (AlongS > MaxS + 2f)
            {
                MaxS = AlongS;
                LastProgressMs = nowMs;
            }

            if (!hasLastHeading) { lastHeadingForCircle = egoHeadingDeg; hasLastHeading = true; }
            else
            {
                float dh = RaceMath.HeadingDiffDeg(egoHeadingDeg, lastHeadingForCircle);
                circleAccumDeg += Math.Abs(dh);
                lastHeadingForCircle = egoHeadingDeg;
                if (AlongS > MaxS - 5f && nowMs - LastProgressMs > 500)
                {
                    circleAccumDeg *= 0.9f;
                }
            }

            // Heading-incompatibility persistence: a single sideways sample
            // must already gate planning (PlanInvalid, checked by the brain),
            // but IsLost requires ~400 ms of severe misalignment to avoid
            // flicker inside intersections.
            float absHead = Math.Abs(HeadingErrorDeg);
            if (absHead > LostHeadErrDeg && speed > 3f)
            {
                if (headingInvalidSinceMs < 0) headingInvalidSinceMs = nowMs;
            }
            else if (absHead <= SevereHeadErrDeg)
            {
                headingInvalidSinceMs = -100000;
            }

            bool farOff = DistToRoute > Math.Max(28f, corridorHalfWidth + 16f);
            bool goingAway = absHead > 70f && speed > 5f;
            bool wentBackwards = (MaxS - AlongS) > 20f;
            bool circling = circleAccumDeg > 300f && (nowMs - LastProgressMs) > 6000;
            bool noProgress = (nowMs - LastProgressMs) > 12000 && speed > 6f && (MaxS - AlongS) > -5f
                && DistToRoute > corridorHalfWidth + 6f;
            bool headingLost = headingInvalidSinceMs >= 0 && (nowMs - headingInvalidSinceMs) > 400 && absHead > LostHeadErrDeg;

            if (farOff)
                SetLost(nowMs, "AwayFromRoute", speed);
            else if (headingLost)
                SetLost(nowMs, "HeadingIncompatible", speed);
            else if (wentBackwards)
                SetLost(nowMs, "WentBackwards", speed);
            else if (circling)
                SetLost(nowMs, "Circling", speed);
            else if (goingAway && nowMs - LastProgressMs > 1500)
                SetLost(nowMs, "WrongDirection", speed);
            else if (noProgress)
                SetLost(nowMs, "NoProgress", speed);
            else if (!IsLost)
            {
                LossReason = "";
            }
            else
            {
                bool aligned = Math.Abs(HeadingErrorDeg) < 45f;
                if (DistToRoute < corridorHalfWidth + 6f && (aligned || speed < 3f))
                {
                    IsLost = false;
                    LossReason = "";
                    LastProgressMs = nowMs;
                    circleAccumDeg = 0f;
                    headingInvalidSinceMs = -100000;
                }
            }
        }

        private void SetLost(int nowMs, string reason, float speed)
        {
            if (!IsLost)
            {
                IsLost = true;
                LossReason = reason;
                LostSinceMs = nowMs;
            }
            else
            {
                if (string.IsNullOrEmpty(LossReason)) LossReason = reason;
            }
        }

        public void NotifyRecovered(int nowMs)
        {
            IsLost = false;
            LossReason = "";
            LastProgressMs = nowMs;
            circleAccumDeg = 0f;
            headingInvalidSinceMs = -100000;
        }

        /// Point on the route `distM` ahead of current AlongS (clamped to finish).
        public Vector3 LookaheadPoint(float distM)
        {
            return PointAtS(AlongS + distM);
        }

        /// Route heading at `distM` ahead (for curvature / aim).
        public float HeadingAhead(float distM)
        {
            return HeadingAtS(AlongS + distM);
        }

        /// Approximate curvature (rad/m) between now and distM ahead.
        /// Uses smoothed headings so dense GPS points don't inject noise.
        public float CurvatureAhead(float distM)
        {
            if (!Built) return 0f;
            if (distM < 1f) return 0f;
            float h0 = HeadingAtS(AlongS + 2f);
            float h1 = HeadingAtS(AlongS + distM);
            float dh = RaceMath.HeadingDiffDeg(h1, h0) * (float)Math.PI / 180f;
            return Math.Abs(dh) / distM;
        }

        /// Local curvature at absolute arclength s (rad/m), from a centered
        /// heading window. Used by the speed profiler's braking pass.
        public float CurvatureAtS(float s, float windowM = 20f)
        {
            if (!Built) return 0f;
            if (windowM < 4f) windowM = 4f;
            float h0 = HeadingAtS(s - windowM * 0.5f);
            float h1 = HeadingAtS(s + windowM * 0.5f);
            float dh = RaceMath.HeadingDiffDeg(h1, h0) * (float)Math.PI / 180f;
            return Math.Abs(dh) / windowM;
        }

        public Vector3 PointAtS(float s)
        {
            if (!Built || Points.Count == 0) return finish;
            if (s <= 0f) return Points[0];
            if (s >= TotalLength) return Points[Points.Count - 1];
            int lo = Math.Max(0, NearestIndex - 4);
            for (int i = lo; i < CumulativeS.Count - 1; i++)
            {
                if (CumulativeS[i + 1] >= s && s >= CumulativeS[i] - 0.01f)
                {
                    float segLen = CumulativeS[i + 1] - CumulativeS[i];
                    float t = segLen > 1e-4f ? (s - CumulativeS[i]) / segLen : 0f;
                    var a = Points[i];
                    var b = Points[i + 1];
                    return new Vector3(
                        a.X + (b.X - a.X) * t,
                        a.Y + (b.Y - a.Y) * t,
                        a.Z + (b.Z - a.Z) * t);
                }
            }
            for (int i = 0; i < CumulativeS.Count - 1; i++)
            {
                if (CumulativeS[i + 1] >= s)
                {
                    float segLen = CumulativeS[i + 1] - CumulativeS[i];
                    float t = segLen > 1e-4f ? (s - CumulativeS[i]) / segLen : 0f;
                    var a = Points[i];
                    var b = Points[i + 1];
                    return new Vector3(
                        a.X + (b.X - a.X) * t,
                        a.Y + (b.Y - a.Y) * t,
                        a.Z + (b.Z - a.Z) * t);
                }
            }
            return Points[Points.Count - 1];
        }

        public float HeadingAtS(float s)
        {
            if (!Built || Points.Count < 2) return 0f;
            if (s < 0f) s = 0f;
            if (s >= TotalLength) s = Math.Max(0f, TotalLength - 1f);
            float w = 12f;
            Vector3 a = PointAtS(Math.Max(0f, s - w * 0.5f));
            Vector3 b = PointAtS(Math.Min(TotalLength, s + w * 0.5f));
            var d = new Vector3(b.X - a.X, b.Y - a.Y, 0f);
            if (RaceMath.FlatLength(d) < 0.5f)
            {
                for (int i = 0; i < CumulativeS.Count - 1; i++)
                {
                    if (CumulativeS[i + 1] >= s)
                    {
                        var p0 = Points[i];
                        var p1 = Points[i + 1];
                        return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(
                            new Vector3(p1.X - p0.X, p1.Y - p0.Y, 0f)));
                    }
                }
                var l1 = Points[Points.Count - 2];
                var l2 = Points[Points.Count - 1];
                return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(
                    new Vector3(l2.X - l1.X, l2.Y - l1.Y, 0f)));
            }
            return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
        }

        public struct RouteProjection
        {
            public float S;
            public float Lateral;
            public float Dist;
            public int SegIndex;
            public Vector3 Closest;
            public Vector3 Dir;
        }

        /// Project an arbitrary world point onto the route (full search).
        /// Used to express actors in route coordinates.
        public RouteProjection ProjectOntoRoute(Vector3 p)
        {
            var r = new RouteProjection { S = AlongS, Lateral = 0f, Dist = 999f, SegIndex = NearestIndex, Closest = p, Dir = new Vector3(0f, 1f, 0f) };
            if (!Built || Points.Count < 2) return r;
            float best = float.MaxValue;
            int bestI = 0;
            RaceMath.Projection bestPr = new RaceMath.Projection();
            Vector3 bestDir = r.Dir;
            for (int i = 0; i < Points.Count - 1; i++)
            {
                var a = Points[i];
                var b = Points[i + 1];
                var pr = RaceMath.ProjectOnSegment(p, a, b);
                if (pr.Dist < best)
                {
                    best = pr.Dist;
                    bestI = i;
                    bestPr = pr;
                    bestDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                }
            }
            r.SegIndex = bestI;
            r.S = CumulativeS[bestI] + bestPr.Along;
            r.Dist = best;
            r.Closest = bestPr.Closest;
            r.Dir = bestDir;
            r.Lateral = RaceMath.FlatCross(bestDir, new Vector3(p.X - bestPr.Closest.X, p.Y - bestPr.Closest.Y, 0f));
            return r;
        }

        public float Progress01 => TotalLength > 1f ? RaceMath.Clamp(AlongS / TotalLength, 0f, 1f) : 0f;

        /// Recovery aim: prefer a heading-compatible future merge station.
        /// Falls back to a near point ahead (never the distant finish).
        public Vector3 RecoveryTarget()
        {
            return LookaheadPoint(40f);
        }

        /// Find a future route station that is spatially reachable AND
        /// heading-compatible for a low-speed merge. Rejects U-turn-like
        /// merges. Returns false when no sane merge exists (caller must
        /// stop/crawl instead of commanding a point on the invalid route).
        public bool TryGetRecoveryMerge(Vector3 egoPos, float egoHeadingDeg,
            out Vector3 target, out float mergeS, out string detail)
        {
            target = LookaheadPoint(40f);
            mergeS = AlongS + 40f;
            detail = "none";
            try
            {
                if (!Built || Points.Count < 2) { detail = "not-built"; return false; }
                float bestScore = float.MaxValue;
                float bestS = -1f;
                Vector3 bestPt = target;
                float bestHeadErr = 999f;
                float bestBErr = 999f;
                float bestDist = 999f;
                float sMax = Math.Min(TotalLength - 2f, AlongS + 150f);
                for (float s = AlongS + 10f; s <= sMax; s += 5f)
                {
                    Vector3 pt = PointAtS(s);
                    float rh = HeadingAtS(s);
                    float headErr = Math.Abs(RaceMath.HeadingDiffDeg(rh, egoHeadingDeg));
                    if (headErr > 45f) continue;
                    var to = new Vector3(pt.X - egoPos.X, pt.Y - egoPos.Y, 0f);
                    float dist = RaceMath.FlatLength(to);
                    if (dist < 1f) continue;
                    if (dist > 100f) continue;
                    float bearing = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(to));
                    float bErr = Math.Abs(RaceMath.HeadingDiffDeg(bearing, egoHeadingDeg));
                    if (bErr > 65f) continue;
                    float alongGap = s - AlongS;
                    if (dist > alongGap + 40f) continue;
                    float score = headErr * 1.0f + bErr * 0.6f + dist * 0.15f + Math.Max(0f, alongGap - 60f) * 0.2f;
                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestS = s;
                        bestPt = pt;
                        bestHeadErr = headErr;
                        bestBErr = bErr;
                        bestDist = dist;
                    }
                }
                if (bestS < 0f) { detail = "no-compatible-merge"; return false; }
                target = bestPt;
                mergeS = bestS;
                detail = $"s={bestS:F0} headErr={bestHeadErr:F0} bErr={bestBErr:F0} dist={bestDist:F0} score={bestScore:F1}";
                return true;
            }
            catch (Exception ex)
            {
                try { detail = "exc:" + ex.Message; } catch { }
                return false;
            }
        }

        private static Vector3 SnapToStreet(Vector3 p)
        {
            try
            {
                var s = World.GetNextPositionOnStreet(p, true);
                if (s != Vector3.Zero && RaceMath.FlatDistance(s, Vector3.Zero) > 1f) return s;
            }
            catch { }
            return p;
        }

        /// True road-network distance remaining when the native cooperates;
        /// falls back to route arclength.
        public static float TravelDistance(Vector3 a, Vector3 b)
        {
            try
            {
                float d = Function.Call<float>(Hash.CALCULATE_TRAVEL_DISTANCE_BETWEEN_POINTS,
                    a.X, a.Y, a.Z, b.X, b.Y, b.Z);
                if (d > 0f && d < 20000f) return d;
            }
            catch { }
            return RaceMath.FlatDistance(a, b);
        }
    }
}
