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
    /// Why not DriveTo(finish) directly: a 2 km target lets GTA pick either
    /// carriageway at splits and silently drop the route when the target is
    /// unreachable from the current lane. We track a local corridor instead
    /// and always command a short-horizon aim point on the correct side.
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

        private float lastHeadingForCircle;
        private float circleAccumDeg;
        private bool hasLastHeading;
        private Vector3 finish = Vector3.Zero;

        public Vector3 Finish => finish;

        public void Build(Vector3 origin, Vector3 finishIn)
        {
            Points.Clear();
            CumulativeS.Clear();
            TotalLength = 0f;
            Built = false;
            finish = finishIn;
            NearestIndex = 0;
            AlongS = 0f;
            MaxS = 0f;
            Lateral = 0f;
            DistToRoute = RaceMath.FlatDistance(origin, finishIn);
            HeadingErrorDeg = 0f;
            IsLost = false;
            LossReason = "";
            hasLastHeading = false;
            circleAccumDeg = 0f;
            LastProgressMs = Game.GameTime;
            Source = "None";
            GpsSamples = 0;

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

        /// Called by RaceBrain during the first seconds if Build() fell back
        /// before the GPS route existed (blip route takes a frame or two to
        /// compute). Returns true when an upgrade happened.
        /// Invariant: switching FallbackWalk-&gt;GPS must NOT redefine
        /// progress/heading underneath the planner. The old code reset
        /// AlongS=0/NearestIndex=0 without reprojecting, so the next plan
        /// built from s=0 (route behind ego) with a flipped heading
        /// (&gt;90 deg error) while the actuator still held the old maneuver.
        /// This version reprojects ego cleanly onto the NEW polyline (full
        /// search), resets the high-water mark to the new projection, clears
        /// circling/lost state, and returns an old/new diagnostic string the
        /// caller must log + use to invalidate the old maneuver (force replan
        /// + actuator reissue same tick). Prefer GPS before Start (see
        /// StreetRacing arming); this is the safe fallback when it arrives late.
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
                // Snapshot old tracking for the upgrade log (setup-failure evidence).
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
                // Use current ego pos as origin hint for validation, but keep
                // original start anchored: prepend existing start if GPS starts
                // ahead of us.
                if (!TryBuildFromGps(Points.Count > 0 ? Points[0] : egoPos, finish, out gps, out how))
                    return false;
                if (gps == null || gps.Count < 2) return false;
                Points.Clear();
                foreach (var p in gps) Points.Add(p);
                Source = how;
                GpsSamples = gps.Count;
                FinalizeGeometry();
                // Clean reproject onto the NEW geometry (full search, not the
                // windowed Update): ego never jumps to s=0.
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
                            if (pass == 0 && RaceMath.FlatDot(segDir, egoFwd) < -0.1f) continue;
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
                        DistToRoute = bestPr.Dist;
                        Lateral = RaceMath.FlatCross(bestDir, new Vector3(egoPos.X - bestPr.Closest.X, egoPos.Y - bestPr.Closest.Y, 0f));
                        float newHead = RaceMath.HeadingFromVector(bestDir);
                        HeadingErrorDeg = RaceMath.HeadingDiffDeg(newHead, egoHeadingDeg);
                    }
                    else
                    {
                        NearestIndex = 0;
                        AlongS = 0f;
                        DistToRoute = 999f;
                        Lateral = 0f;
                        HeadingErrorDeg = 0f;
                    }
                }
                catch
                {
                    NearestIndex = 0;
                    AlongS = 0f;
                }
                // New geometry has incomparable arclength: reset high-water to
                // the fresh projection so WentBackwards/NoProgress cannot trip
                // on old-route mileage. Clear circling + lost (re-evaluated).
                MaxS = AlongS;
                LastProgressMs = nowMs;
                FinishGapEuclid = RaceMath.FlatDistance(egoPos, finish);
                hasLastHeading = false;
                circleAccumDeg = 0f;
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

            // Candidate route types to try in order. 1 first: in practice the
            // driving route renders as type 1; 0/2 are alternates.
            int[] types = { 1, 0, 2 };
            foreach (int t in types)
            {
                // Interpretation A: p2 = distance along route (meters).
                var byDist = SampleGpsByDistance(origin, finishIn, routeLen, t);
                if (IsPlausibleRoute(byDist, origin, finishIn))
                {
                    pts = ResamplePolyline(byDist, 18f);
                    how = "GpsDist(t" + t + ")";
                    return true;
                }
                // Interpretation B: p2 = node index 0..len-1.
                var byIdx = SampleGpsByIndex(origin, finishIn, routeLen, t);
                if (IsPlausibleRoute(byIdx, origin, finishIn))
                {
                    pts = ResamplePolyline(byIdx, 18f);
                    how = "GpsIdx(t" + t + ")";
                    return true;
                }
            }
            return false;
        }

        private static List<Vector3> SampleGpsByDistance(Vector3 origin, Vector3 finishIn, int routeLen, int type)
        {
            var outPts = new List<Vector3>();
            // Upper bound: route length if it looks like meters, else 9 km cap.
            float maxD = 9000f;
            if (routeLen > 200 && routeLen < 15000) maxD = routeLen + 400f;
            float step = 25f;
            int failStreak = 0;
            Vector3 last = Vector3.Zero;
            bool haveLast = false;
            for (float d = 0f; d <= maxD; d += step)
            {
                Vector3 p;
                if (!TryGpsPos(true, d, type, out p))
                {
                    // Also try p1=false once before giving up on this station.
                    if (!TryGpsPos(false, d, type, out p))
                    {
                        failStreak++;
                        if (failStreak >= 6 && outPts.Count >= 4) break;
                        if (d > 1500f && outPts.Count < 3) break;
                        continue;
                    }
                }
                failStreak = 0;
                if (p == Vector3.Zero) continue;
                if (haveLast)
                {
                    float gap = RaceMath.FlatDistance(last, p);
                    if (gap < 4f) continue;          // duplicate sample
                    if (gap > 400f) break;           // jumped (wrong type?) — stop
                }
                outPts.Add(p);
                last = p;
                haveLast = true;
                // Stop once we reach the finish neighbourhood.
                if (RaceMath.FlatDistance(p, finishIn) < 35f && outPts.Count > 4) break;
                if (outPts.Count > 420) break;
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
                if (outPts.Count > 600) break;
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
            // Must start near origin and end near finish (GPS is player-centric;
            // allow generous tolerance since rival starts near player).
            float dStart = RaceMath.FlatDistance(pts[0], origin);
            float dEnd = RaceMath.FlatDistance(pts[pts.Count - 1], finishIn);
            // Also accept routes whose closest approach is near (route may start
            // slightly ahead of us on the road network).
            float closestStart = float.MaxValue;
            float closestEnd = float.MaxValue;
            for (int i = 0; i < Math.Min(pts.Count, 12); i++)
                closestStart = Math.Min(closestStart, RaceMath.FlatDistance(pts[i], origin));
            for (int i = Math.Max(0, pts.Count - 12); i < pts.Count; i++)
                closestEnd = Math.Min(closestEnd, RaceMath.FlatDistance(pts[i], finishIn));
            if (Math.Min(dStart, closestStart) > 220f) return false;
            if (Math.Min(dEnd, closestEnd) > 260f) return false;
            // Total length sanity: must be >= straight-line * 0.7 (not a stub)
            // and <= straight-line * 4 + 1500 (not a spiral).
            float straight = RaceMath.FlatDistance(origin, finishIn);
            float len = 0f;
            for (int i = 1; i < pts.Count; i++) len += RaceMath.FlatDistance(pts[i - 1], pts[i]);
            if (len < straight * 0.6f) return false;
            if (len > straight * 4f + 1500f) return false;
            if (len < 60f) return false;
            return true;
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

            // Nearest-segment search in a window around the last index (fast
            // path). Full search when lost or when the window finds nothing
            // close — keeps splits cheap but recoverable.
            int bestSeg = -1;
            float bestDist = float.MaxValue;
            RaceMath.Projection bestProj = new RaceMath.Projection();
            Vector3 bestDir = new Vector3(0f, 1f, 0f);

            int lo = Math.Max(0, NearestIndex - 3);
            int hi = Math.Min(Points.Count - 2, NearestIndex + 12);
            // When lost, search everything.
            bool fullSearch = IsLost;
            if (fullSearch) { lo = 0; hi = Points.Count - 2; }

            // Two-pass: prefer same-direction segments near carriageway splits
            // so we don't snap across to the opposite carriageway.
            var egoFwd = RaceMath.VectorFromHeading(egoHeadingDeg);
            for (int pass = 0; pass < 2; pass++)
            {
                for (int i = lo; i <= hi; i++)
                {
                    var a = Points[i];
                    var b = Points[i + 1];
                    var segDir = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                    if (pass == 0)
                    {
                        // Pass 0: same-direction segments only.
                        if (RaceMath.FlatDot(segDir, egoFwd) < -0.1f) continue;
                    }
                    var pr = RaceMath.ProjectOnSegment(egoPos, a, b);
                    if (pr.Dist < bestDist)
                    {
                        bestDist = pr.Dist;
                        bestSeg = i;
                        bestProj = pr;
                        bestDir = segDir;
                    }
                }
                if (bestSeg >= 0 && bestDist < 60f) break; // good enough, keep direction bias
                if (pass == 0)
                {
                    bestSeg = -1;
                    bestDist = float.MaxValue;
                }
            }

            if (bestSeg < 0)
            {
                SetLost(nowMs, "NoSegment", speed);
                return;
            }

            NearestIndex = bestSeg;
            AlongS = CumulativeS[bestSeg] + bestProj.Along;
            DistToRoute = bestProj.Dist;
            Lateral = RaceMath.FlatCross(bestDir, new Vector3(egoPos.X - bestProj.Closest.X, egoPos.Y - bestProj.Closest.Y, 0f));
            float routeHeading = RaceMath.HeadingFromVector(bestDir);
            HeadingErrorDeg = RaceMath.HeadingDiffDeg(routeHeading, egoHeadingDeg);

            if (AlongS > MaxS + 2f)
            {
                MaxS = AlongS;
                LastProgressMs = nowMs;
            }

            // Circling detector: large cumulative heading change without progress.
            if (!hasLastHeading) { lastHeadingForCircle = egoHeadingDeg; hasLastHeading = true; }
            else
            {
                float dh = RaceMath.HeadingDiffDeg(egoHeadingDeg, lastHeadingForCircle);
                circleAccumDeg += Math.Abs(dh);
                lastHeadingForCircle = egoHeadingDeg;
                if (AlongS > MaxS - 5f && nowMs - LastProgressMs > 500)
                {
                    // Making progress: decay the accumulator.
                    circleAccumDeg *= 0.9f;
                }
            }

            bool farOff = DistToRoute > Math.Max(28f, corridorHalfWidth + 16f);
            bool goingAway = Math.Abs(HeadingErrorDeg) > 100f && speed > 8f;
            bool wentBackwards = (MaxS - AlongS) > 25f;
            bool circling = circleAccumDeg > 300f && (nowMs - LastProgressMs) > 6000;
            bool noProgress = (nowMs - LastProgressMs) > 12000 && speed > 6f && (MaxS - AlongS) > -5f
                && DistToRoute > corridorHalfWidth + 6f;

            if (farOff)
                SetLost(nowMs, "AwayFromRoute", speed);
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
                // Recovery: re-acquired when close to the line and roughly aligned,
                // or simply close and moving toward it.
                bool aligned = Math.Abs(HeadingErrorDeg) < 60f;
                if (DistToRoute < corridorHalfWidth + 8f && (aligned || speed < 4f))
                {
                    IsLost = false;
                    LossReason = "";
                    LastProgressMs = nowMs;
                    circleAccumDeg = 0f;
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
                // Keep the first reason while lost (more diagnostic than flapping).
                if (string.IsNullOrEmpty(LossReason)) LossReason = reason;
            }
        }

        public void NotifyRecovered(int nowMs)
        {
            IsLost = false;
            LossReason = "";
            LastProgressMs = nowMs;
            circleAccumDeg = 0f;
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
            // Average direction over a small window for stability on dense GPS.
            float w = 12f;
            Vector3 a = PointAtS(Math.Max(0f, s - w * 0.5f));
            Vector3 b = PointAtS(Math.Min(TotalLength, s + w * 0.5f));
            var d = new Vector3(b.X - a.X, b.Y - a.Y, 0f);
            if (RaceMath.FlatLength(d) < 0.5f)
            {
                // Fall back to raw segment.
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
            public float Lateral; // + = left of route direction
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

        /// Recovery aim: a near route point just ahead of the projection, never
        /// the distant finish — this is what fixes carriageway-split losses.
        public Vector3 RecoveryTarget()
        {
            // Aim ~40 m ahead of the nearest point so the actuator rejoins the
            // route in the correct direction instead of U-turning to the finish.
            return LookaheadPoint(40f);
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
        /// falls back to route arclength. Used to spot wrong-carriageway
        /// situations where Euclid barely moves but travel distance spikes.
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
