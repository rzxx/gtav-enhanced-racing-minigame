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

            // Walk from origin toward finish in ~40 m steps, snapping each probe
            // to the street network. This follows roads coarsely; the corridor
            // estimator + local planner handle exact lane geometry at runtime.
            // If snapping fails we keep the raw probe so the route still exists.
            try
            {
                var flat = new Vector3(finishIn.X - origin.X, finishIn.Y - origin.Y, 0f);
                if (RaceMath.FlatLength(flat) < 1f) flat = new Vector3(0f, 1f, 0f);
                flat = RaceMath.FlatNormalize(flat);

                Vector3 cursor = origin;
                Points.Add(SnapToStreet(origin));
                for (int i = 0; i < 110; i++)
                {
                    float remaining = RaceMath.FlatDistance(cursor, finishIn);
                    if (remaining < 45f) break;
                    float step = Math.Min(45f, remaining * 0.5f);
                    if (step < 20f) step = Math.Min(20f, remaining);
                    // Aim each step slightly toward the finish from the snapped
                    // cursor so the polyline bends with the road network instead
                    // of cutting straight across blocks.
                    var toF = RaceMath.FlatNormalize(new Vector3(finishIn.X - cursor.X, finishIn.Y - cursor.Y, 0f));
                    var probe = new Vector3(cursor.X + toF.X * step, cursor.Y + toF.Y * step, cursor.Z);
                    Vector3 snapped = SnapToStreet(probe);
                    // Guard against the snap collapsing back onto the same node
                    // (junctions / dual carriageways): nudge forward if stuck.
                    if (RaceMath.FlatDistance(snapped, cursor) < 5f)
                    {
                        var probe2 = new Vector3(probe.X + toF.X * 25f, probe.Y + toF.Y * 25f, probe.Z);
                        snapped = SnapToStreet(probe2);
                        if (RaceMath.FlatDistance(snapped, cursor) < 5f) break;
                    }
                    Points.Add(snapped);
                    cursor = snapped;
                }
                Points.Add(SnapToStreet(finishIn));
            }
            catch
            {
                if (Points.Count == 0) Points.Add(origin);
                Points.Add(finishIn);
            }

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
            if (!Built || Points.Count == 0) return finish;
            float target = AlongS + distM;
            if (target >= TotalLength) return Points[Points.Count - 1];
            for (int i = NearestIndex; i < CumulativeS.Count - 1; i++)
            {
                if (CumulativeS[i + 1] >= target)
                {
                    float segLen = CumulativeS[i + 1] - CumulativeS[i];
                    float t = segLen > 1e-4f ? (target - CumulativeS[i]) / segLen : 0f;
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

        /// Route heading at `distM` ahead (for curvature / aim).
        public float HeadingAhead(float distM)
        {
            if (!Built || Points.Count < 2) return 0f;
            float target = AlongS + distM;
            if (target >= TotalLength) target = TotalLength - 1f;
            if (target < 0f) target = 0f;
            for (int i = 0; i < CumulativeS.Count - 1; i++)
            {
                if (CumulativeS[i + 1] >= target)
                {
                    var a = Points[i];
                    var b = Points[i + 1];
                    return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(
                        new Vector3(b.X - a.X, b.Y - a.Y, 0f)));
                }
            }
            var l1 = Points[Points.Count - 2];
            var l2 = Points[Points.Count - 1];
            return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(
                new Vector3(l2.X - l1.X, l2.Y - l1.Y, 0f)));
        }

        /// Approximate curvature (rad/m) between now and distM ahead.
        public float CurvatureAhead(float distM)
        {
            if (!Built) return 0f;
            float h0 = HeadingAhead(0f);
            float h1 = HeadingAhead(distM);
            float dh = RaceMath.HeadingDiffDeg(h1, h0) * (float)Math.PI / 180f;
            if (distM < 1f) return 0f;
            return Math.Abs(dh) / distM;
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
