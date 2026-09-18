using System;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// A cross-section of drivable road at one route station.
    internal struct CorridorSlice
    {
        public bool Valid;
        public Vector3 Center;
        public float HeadingDeg;
        public float HalfWidthM;   // usable half width (road edge, not lane center)
        public float Curvature;    // rad/m estimated from route headings
        public Vector3 LeftEdge;
        public Vector3 RightEdge;
    }

    /// Real drivable-road representation: usable width + boundaries, not a
    /// lane-center line. Replaces the old `offroad_m` metric, which measured
    /// distance to a *future* street node and was meaningless as a lateral
    /// departure signal.
    ///
    /// Method: prefer GET_ROAD_BOUNDARY_USING_HEADING; fall back to a lateral
    /// IS_POINT_ON_ROAD sweep; fall back to a conservative default width.
    /// Off-corridor distance = max(0, |lateral| - halfWidth).
    internal sealed class RoadCorridor
    {
        public CorridorSlice Current;
        public CorridorSlice Ahead40;
        public CorridorSlice Ahead80;
        public CorridorSlice Ahead120;
        public float LastProbeMs;

        private const float DefaultHalfWidth = 7f;

        public void Update(RaceRoute route, Vector3 egoPos, int nowMs)
        {
            try
            {
                Current = EstimateAt(route.LookaheadPoint(5f), route.HeadingAhead(5f), route);
                Ahead40 = EstimateAt(route.LookaheadPoint(40f), route.HeadingAhead(40f), route);
                Ahead80 = EstimateAt(route.LookaheadPoint(80f), route.HeadingAhead(80f), route);
                Ahead120 = EstimateAt(route.LookaheadPoint(120f), route.HeadingAhead(120f), route);
                // Curvature from route headings (independent of width probing).
                Current.Curvature = route.CurvatureAhead(40f);
                Ahead40.Curvature = route.CurvatureAhead(80f);
                Ahead80.Curvature = route.CurvatureAhead(80f);
                Ahead120.Curvature = route.CurvatureAhead(120f);
                LastProbeMs = nowMs;
            }
            catch
            {
                if (!Current.Valid)
                    Current = Fallback(route.LookaheadPoint(5f), route.HeadingAhead(5f));
            }
        }

        public float HalfWidth => Current.Valid ? Current.HalfWidthM : DefaultHalfWidth;

        /// Signed lateral of a world point relative to the current slice.
        public float LateralOf(Vector3 p)
        {
            if (!Current.Valid) return 0f;
            var dir = RaceMath.VectorFromHeading(Current.HeadingDeg);
            return RaceMath.FlatCross(dir, new Vector3(p.X - Current.Center.X, p.Y - Current.Center.Y, 0f));
        }

        /// Meters outside the drivable surface (0 = on road). This is the
        /// replacement for the retired offroad_m metric.
        public float OffCorridor(float lateral)
        {
            float o = Math.Abs(lateral) - HalfWidth;
            return o > 0f ? o : 0f;
        }

        public Vector3 PointAtLateral(float lateralM, float distAheadM, RaceRoute route)
        {
            Vector3 c = route.LookaheadPoint(distAheadM);
            float h = route.HeadingAhead(distAheadM);
            var dir = RaceMath.VectorFromHeading(h);
            var left = new Vector3(-dir.Y, dir.X, 0f);
            return new Vector3(c.X + left.X * lateralM, c.Y + left.Y * lateralM, c.Z);
        }

        private CorridorSlice EstimateAt(Vector3 center, float headingDeg, RaceRoute route)
        {
            // 1) Native road boundary.
            var viaNative = TryNativeBoundary(center, headingDeg);
            if (viaNative.Valid) return viaNative;

            // 2) Lateral IS_POINT_ON_ROAD sweep.
            var viaSweep = TrySweep(center, headingDeg);
            if (viaSweep.Valid) return viaSweep;

            // 3) Conservative fallback.
            return Fallback(center, headingDeg);
        }

        private static CorridorSlice TryNativeBoundary(Vector3 center, float headingDeg)
        {
            var s = new CorridorSlice { Valid = false };
            try
            {
                var out1 = new OutputArgument();
                var out2 = new OutputArgument();
                bool ok = Function.Call<bool>(Hash.GET_ROAD_BOUNDARY_USING_HEADING,
                    center.X, center.Y, center.Z, headingDeg, out1, out2);
                if (!ok) return s;
                Vector3 p1 = out1.GetResult<Vector3>();
                Vector3 p2 = out2.GetResult<Vector3>();
                if (p1 == Vector3.Zero || p2 == Vector3.Zero) return s;
                float w = RaceMath.FlatDistance(p1, p2);
                if (w < 2f || w > 60f) return s; // sanity: reject garbage
                var mid = new Vector3((p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f, (p1.Z + p2.Z) * 0.5f);
                s.Valid = true;
                s.Center = mid;
                s.HeadingDeg = headingDeg;
                s.HalfWidthM = RaceMath.Clamp(w * 0.5f, 2.5f, 18f);
                var dir = RaceMath.VectorFromHeading(headingDeg);
                var left = new Vector3(-dir.Y, dir.X, 0f);
                s.LeftEdge = new Vector3(mid.X + left.X * s.HalfWidthM, mid.Y + left.Y * s.HalfWidthM, mid.Z);
                s.RightEdge = new Vector3(mid.X - left.X * s.HalfWidthM, mid.Y - left.Y * s.HalfWidthM, mid.Z);
                return s;
            }
            catch { return s; }
        }

        private static CorridorSlice TrySweep(Vector3 center, float headingDeg)
        {
            var s = new CorridorSlice { Valid = false };
            try
            {
                var dir = RaceMath.VectorFromHeading(headingDeg);
                var left = new Vector3(-dir.Y, dir.X, 0f);
                float leftEdge = ProbeSide(center, left, 1);
                float rightEdge = ProbeSide(center, left, -1);
                if (leftEdge < 1f && rightEdge < 1f) return s;
                float half = (leftEdge + rightEdge) * 0.5f;
                half = RaceMath.Clamp(half, 2.5f, 18f);
                // Recenter if the route point was off-center (e.g. junction).
                float off = (leftEdge - rightEdge) * 0.5f;
                var mid = new Vector3(center.X + left.X * off, center.Y + left.Y * off, center.Z);
                s.Valid = true;
                s.Center = mid;
                s.HeadingDeg = headingDeg;
                s.HalfWidthM = half;
                s.LeftEdge = new Vector3(mid.X + left.X * half, mid.Y + left.Y * half, mid.Z);
                s.RightEdge = new Vector3(mid.X - left.X * half, mid.Y - left.Y * half, mid.Z);
                return s;
            }
            catch { return s; }
        }

        private static float ProbeSide(Vector3 center, Vector3 left, int sign)
        {
            float lastOn = 0f;
            for (float d = 1.5f; d <= 21f; d += 1.5f)
            {
                var p = new Vector3(center.X + left.X * d * sign, center.Y + left.Y * d * sign, center.Z);
                bool on = false;
                try
                {
                    on = Function.Call<bool>(Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z, 0);
                }
                catch
                {
                    // Native unavailable: assume default lane width and stop.
                    return DefaultHalfWidth;
                }
                if (on) lastOn = d;
                else if (lastOn > 0f) break; // left the surface
            }
            return lastOn > 0f ? lastOn : DefaultHalfWidth * 0.7f;
        }

        private static CorridorSlice Fallback(Vector3 center, float headingDeg)
        {
            var dir = RaceMath.VectorFromHeading(headingDeg);
            var left = new Vector3(-dir.Y, dir.X, 0f);
            return new CorridorSlice
            {
                Valid = true,
                Center = center,
                HeadingDeg = headingDeg,
                HalfWidthM = DefaultHalfWidth,
                Curvature = 0f,
                LeftEdge = new Vector3(center.X + left.X * DefaultHalfWidth, center.Y + left.Y * DefaultHalfWidth, center.Z),
                RightEdge = new Vector3(center.X - left.X * DefaultHalfWidth, center.Y - left.Y * DefaultHalfWidth, center.Z),
            };
        }
    }
}
