using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// A cross-section of drivable road at one route station.
    internal struct CorridorSlice
    {
        public bool Valid;
        public float SAheadM;    // distance ahead of ego AlongS
        public float SRouteM;    // absolute route arclength
        public Vector3 Center;
        public float HeadingDeg;
        public float HalfWidthM;   // usable half width (road edge, not lane center)
        public float Curvature;    // rad/m estimated from route headings
        public Vector3 LeftEdge;
        public Vector3 RightEdge;
        public string Probe;       // "NativePair" | "Sweep" | "Fallback"
    }

    /// Real drivable-road representation: usable width + boundaries, not a
    /// lane-center line.
    ///
    /// NATIVE CORRECTION: GET_ROAD_BOUNDARY_USING_HEADING takes
    ///   (x, y, z, heading, Vector3* outPosition) -> BOOL
    /// i.e. ONE output position, not two. The old code passed two
    /// OutputArguments and interpreted them as both edges — that reads
    /// uninitialized stack memory as geometry. The fix probes twice, once
    /// toward each side (roadHeading +/- 90 deg), and validates the pair
    /// geometrically (width, midpoint) plus cross-checks against an
    /// IS_POINT_ON_ROAD sweep before trusting it.
    ///
    /// The corridor is sampled along the whole planning horizon (slices every
    /// ~10 m out to lookahead+60 m), not a single current HalfWidth. All
    /// planned paths are validated against the slice at their station.
    internal sealed class RoadCorridor
    {
        public CorridorSlice Current;
        public CorridorSlice Ahead40;
        public CorridorSlice Ahead80;
        public CorridorSlice Ahead120;
        public float LastProbeMs;

        // Sampled profile along the horizon. Index i => SAheadM = i * StepM.
        public readonly List<CorridorSlice> Slices = new List<CorridorSlice>();
        public float StepM = 10f;
        public float ProfileLengthM;

        private const float DefaultHalfWidth = 7f;

        public void Reset()
        {
            Slices.Clear();
            ProfileLengthM = 0f;
            Current = new CorridorSlice();
            Ahead40 = new CorridorSlice();
            Ahead80 = new CorridorSlice();
            Ahead120 = new CorridorSlice();
            LastProbeMs = 0;
        }

        public void Update(RaceRoute route, Vector3 egoPos, int nowMs)
        {
            Update(route, egoPos, 150f, nowMs);
        }

        public void Update(RaceRoute route, Vector3 egoPos, float lookaheadM, int nowMs)
        {
            try
            {
                float horizon = Math.Min(lookaheadM + 60f, 230f);
                if (horizon < 60f) horizon = 60f;
                int n = Math.Max(7, (int)(horizon / StepM) + 1);
                if (n > 24) n = 24;
                Slices.Clear();
                for (int i = 0; i < n; i++)
                {
                    float sAhead = i * StepM;
                    // Slight offset for slice 0 so Current matches old behavior.
                    float probeAhead = sAhead == 0f ? 5f : sAhead;
                    Vector3 c = route.LookaheadPoint(probeAhead);
                    float h = route.HeadingAhead(probeAhead);
                    var sl = EstimateAt(c, h, route);
                    sl.SAheadM = sAhead;
                    sl.SRouteM = route.AlongS + probeAhead;
                    sl.Curvature = route.CurvatureAtS(route.AlongS + probeAhead, 20f);
                    Slices.Add(sl);
                }
                ProfileLengthM = (n - 1) * StepM;

                Current = SliceAt(5f, route);
                Ahead40 = SliceAt(40f, route);
                Ahead80 = SliceAt(80f, route);
                Ahead120 = SliceAt(120f, route);
                LastProbeMs = nowMs;
            }
            catch
            {
                if (!Current.Valid)
                    Current = Fallback(route.LookaheadPoint(5f), route.HeadingAhead(5f));
            }
        }

        public float HalfWidth => Current.Valid ? Current.HalfWidthM : DefaultHalfWidth;

        public float HalfWidthAt(float sAheadM)
        {
            if (Slices.Count == 0) return HalfWidth;
            if (sAheadM <= 0f) return Slices[0].HalfWidthM;
            if (sAheadM >= ProfileLengthM) return Slices[Slices.Count - 1].HalfWidthM;
            float f = sAheadM / StepM;
            int i = (int)f;
            if (i < 0) i = 0;
            if (i >= Slices.Count - 1) return Slices[Slices.Count - 1].HalfWidthM;
            float t = f - i;
            return Slices[i].HalfWidthM * (1f - t) + Slices[i + 1].HalfWidthM * t;
        }

        public CorridorSlice SliceAt(float sAheadM, RaceRoute route)
        {
            if (Slices.Count == 0)
                return EstimateAt(route.LookaheadPoint(sAheadM), route.HeadingAhead(sAheadM), route);
            if (sAheadM <= 0f) return Slices[0];
            if (sAheadM >= ProfileLengthM) return Slices[Slices.Count - 1];
            float f = sAheadM / StepM;
            int i = (int)Math.Floor(f);
            if (i < 0) i = 0;
            if (i >= Slices.Count - 1) return Slices[Slices.Count - 1];
            float t = f - i;
            var a = Slices[i];
            var b = Slices[i + 1];
            // Interpolate center/width; heading from route for stability.
            var c = new Vector3(
                a.Center.X + (b.Center.X - a.Center.X) * t,
                a.Center.Y + (b.Center.Y - a.Center.Y) * t,
                a.Center.Z + (b.Center.Z - a.Center.Z) * t);
            var s = new CorridorSlice
            {
                Valid = a.Valid && b.Valid,
                SAheadM = sAheadM,
                SRouteM = a.SRouteM + (b.SRouteM - a.SRouteM) * t,
                Center = c,
                HeadingDeg = route != null ? route.HeadingAtS(a.SRouteM + (b.SRouteM - a.SRouteM) * t) : a.HeadingDeg,
                HalfWidthM = a.HalfWidthM + (b.HalfWidthM - a.HalfWidthM) * t,
                Curvature = a.Curvature + (b.Curvature - a.Curvature) * t,
                Probe = a.Probe,
            };
            var dir = RaceMath.VectorFromHeading(s.HeadingDeg);
            var left = new Vector3(-dir.Y, dir.X, 0f);
            s.LeftEdge = new Vector3(c.X + left.X * s.HalfWidthM, c.Y + left.Y * s.HalfWidthM, c.Z);
            s.RightEdge = new Vector3(c.X - left.X * s.HalfWidthM, c.Y - left.Y * s.HalfWidthM, c.Z);
            return s;
        }

        public float MinHalfWidthAhead(float lookaheadM)
        {
            if (Slices.Count == 0) return HalfWidth;
            float m = float.MaxValue;
            foreach (var s in Slices)
            {
                if (s.SAheadM > lookaheadM + 1f) break;
                if (s.Valid && s.HalfWidthM < m) m = s.HalfWidthM;
            }
            return m == float.MaxValue ? HalfWidth : m;
        }

        /// Signed lateral of a world point relative to the current slice.
        public float LateralOf(Vector3 p)
        {
            if (!Current.Valid) return 0f;
            var dir = RaceMath.VectorFromHeading(Current.HeadingDeg);
            return RaceMath.FlatCross(dir, new Vector3(p.X - Current.Center.X, p.Y - Current.Center.Y, 0f));
        }

        /// Lateral of a world point relative to the slice at sAheadM.
        public float LateralAt(Vector3 p, float sAheadM, RaceRoute route)
        {
            var sl = SliceAt(sAheadM, route);
            if (!sl.Valid) return LateralOf(p);
            var dir = RaceMath.VectorFromHeading(sl.HeadingDeg);
            return RaceMath.FlatCross(dir, new Vector3(p.X - sl.Center.X, p.Y - sl.Center.Y, 0f));
        }

        /// Meters outside the drivable surface at the current slice.
        public float OffCorridor(float lateral)
        {
            float o = Math.Abs(lateral) - HalfWidth;
            return o > 0f ? o : 0f;
        }

        /// Meters outside the surface at a given horizon station.
        public float OffCorridorAt(float lateral, float sAheadM)
        {
            float o = Math.Abs(lateral) - HalfWidthAt(sAheadM);
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
            // 1) Native pair (two single-output probes).
            string why;
            var viaNative = TryNativePair(center, headingDeg, out why);
            if (viaNative.Valid)
            {
                // Cross-check against a cheap center test: the midpoint must be
                // on/near road, else the pair straddled a junction gap.
                if (IsPlausiblyOnRoad(viaNative.Center))
                    return viaNative;
                // Mid off-road: fall through to sweep (junction / shoulder).
            }

            // 2) Lateral IS_POINT_ON_ROAD sweep.
            var viaSweep = TrySweep(center, headingDeg);
            if (viaSweep.Valid) return viaSweep;

            // 3) If native pair was geometrically sane but mid check failed
            // (e.g. bridge where IS_POINT_ON_ROAD lies), still prefer it over
            // the blind default.
            if (viaNative.Valid) return viaNative;

            // 4) Conservative fallback.
            return Fallback(center, headingDeg);
        }

        private static bool IsPlausiblyOnRoad(Vector3 p)
        {
            try
            {
                if (Function.Call<bool>(Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z, 0)) return true;
            }
            catch { return true; } // native unavailable: don't veto native pair
            // One retry slightly below (bridges / overpasses report off-road).
            try
            {
                if (Function.Call<bool>(Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z - 1f, 0)) return true;
            }
            catch { return true; }
            return false;
        }

        /// Two single-output GET_ROAD_BOUNDARY_USING_HEADING probes, one per
        /// side. Heading convention: 0 = +Y, clockwise. Left of roadHeading is
        /// roadHeading - 90.
        private static CorridorSlice TryNativePair(Vector3 center, float roadHeadingDeg, out string why)
        {
            var s = new CorridorSlice { Valid = false };
            why = "fail";
            try
            {
                float leftProbeHeading = roadHeadingDeg - 90f;
                float rightProbeHeading = roadHeadingDeg + 90f;
                Vector3 leftPt, rightPt;
                bool okL = TrySingleBoundary(center, leftProbeHeading, out leftPt);
                bool okR = TrySingleBoundary(center, rightProbeHeading, out rightPt);
                if (!okL || !okR) { why = !okL && !okR ? "both-fail" : (!okL ? "left-fail" : "right-fail"); return s; }
                float w = RaceMath.FlatDistance(leftPt, rightPt);
                if (w < 3f || w > 55f) { why = "width-" + w.ToString("F0"); return s; }
                var mid = new Vector3((leftPt.X + rightPt.X) * 0.5f, (leftPt.Y + rightPt.Y) * 0.5f, (leftPt.Z + rightPt.Z) * 0.5f);
                float midOff = RaceMath.FlatDistance(mid, center);
                // Route point can be a lane off-center at junctions; allow it,
                // but reject pairs whose midpoint is nowhere near the query
                // (wrong-road probe or overpass contamination).
                if (midOff > 16f) { why = "mid-off-" + midOff.ToString("F0"); return s; }
                float dL = RaceMath.FlatDistance(leftPt, center);
                float dR = RaceMath.FlatDistance(rightPt, center);
                if (dL > 32f || dR > 32f) { why = "edge-far"; return s; }

                s.Valid = true;
                s.Center = mid;
                s.HeadingDeg = roadHeadingDeg;
                s.HalfWidthM = RaceMath.Clamp(w * 0.5f, 2.5f, 18f);
                s.LeftEdge = leftPt;
                s.RightEdge = rightPt;
                // Re-derive edges at clamped width so Left/Right match HalfWidth.
                var dir = RaceMath.VectorFromHeading(roadHeadingDeg);
                var left = new Vector3(-dir.Y, dir.X, 0f);
                s.LeftEdge = new Vector3(mid.X + left.X * s.HalfWidthM, mid.Y + left.Y * s.HalfWidthM, mid.Z);
                s.RightEdge = new Vector3(mid.X - left.X * s.HalfWidthM, mid.Y - left.Y * s.HalfWidthM, mid.Z);
                s.Probe = "NativePair";
                why = "ok";
                return s;
            }
            catch { why = "exc"; return s; }
        }

        private static bool TrySingleBoundary(Vector3 from, float probeHeadingDeg, out Vector3 boundaryPt)
        {
            boundaryPt = Vector3.Zero;
            try
            {
                var outPos = new OutputArgument();
                bool ok = Function.Call<bool>(Hash.GET_ROAD_BOUNDARY_USING_HEADING,
                    from.X, from.Y, from.Z, probeHeadingDeg, outPos);
                if (!ok) return false;
                Vector3 p = outPos.GetResult<Vector3>();
                if (p == Vector3.Zero) return false;
                if (Math.Abs(p.X) > 9000f || Math.Abs(p.Y) > 9000f) return false;
                float d = RaceMath.FlatDistance(from, p);
                if (d < 0.8f || d > 40f) return false;
                boundaryPt = p;
                return true;
            }
            catch { return false; }
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
                s.Probe = "Sweep";
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
                Probe = "Fallback",
            };
        }
    }
}
