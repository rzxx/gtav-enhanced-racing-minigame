using System;
using System.Collections.Generic;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Builds the EXECUTABLE local road reference from the raw GTA GPS route.
    ///
    /// GPS is topology, not steering geometry. GET_POS_ALONG_GPS_TYPE_ROUTE
    /// is good at saying which streets to take, but its dense polyline contains
    /// short zig-zags and near-discontinuous junction corners. Feeding those
    /// 5 m segments directly to a steering controller produced impossible
    /// 0.15-0.33 1/m curvature spikes and 100-190 deg/s yaw targets.
    ///
    /// This layer keeps RaceRoute untouched for global progress/localization,
    /// but low-pass filters positions over arclength to create a physically
    /// continuous local center reference. Smoothing is constrained back toward
    /// the raw GPS point whenever the candidate leaves GTA's drivable road.
    internal sealed class DrivingReference
    {
        internal sealed class Result
        {
            public readonly List<Vector3> Path = new List<Vector3>();
            public readonly List<float> StationS = new List<float>();
            // Drivable road envelope measured in the SAME frame as Path.
            // This removes the old mismatch where candidates were generated
            // around the smoothed reference but validated around raw GPS.
            public readonly List<float> LeftRoadM = new List<float>();
            public readonly List<float> RightRoadM = new List<float>();
            public bool Valid;
            public float RawMaxKappa;
            public float MaxKappa;
            public float MaxHeadingStepDeg;
            public int RoadConstrainedPoints;
            public float WindowM;
            public string Detail = "";
        }

        private const float StepM = 4f;
        private const float MinWindowM = 8f;
        private const float MaxWindowM = 16f;

        public Result Build(RaceRoute route, float startS, float horizonM, float speedMps)
        {
            var best = new Result();
            if (route == null || !route.Built || route.Points.Count < 2)
            {
                best.Detail = "route-invalid";
                return best;
            }

            horizonM = RaceMath.Clamp(horizonM, 24f, 140f);
            float baseWindow = RaceMath.Clamp(8f + Math.Max(0f, speedMps) * 0.35f, MinWindowM, 13f);

            // Try progressively smoother references. Prefer the first one whose
            // local geometry has no pathological 4 m kink; otherwise keep the
            // smoothest candidate so the caller can still slow for a tight turn.
            float[] windows = { baseWindow, Math.Min(MaxWindowM, baseWindow + 3f), MaxWindowM };
            for (int attempt = 0; attempt < windows.Length; attempt++)
            {
                var r = BuildOnce(route, startS, horizonM, windows[attempt]);
                if (!r.Valid) continue;
                best = r;
                if (r.MaxHeadingStepDeg <= 28f && r.MaxKappa <= 0.16f)
                    break;
            }
            return best;
        }

        private Result BuildOnce(RaceRoute route, float startS, float horizonM, float windowM)
        {
            var r = new Result { WindowM = windowM };
            try
            {
                var raw = new List<Vector3>();
                int n = Math.Max(7, Math.Min(48, (int)Math.Ceiling(horizonM / StepM) + 1));
                for (int i = 0; i < n; i++)
                {
                    float sAhead = i == n - 1 ? horizonM : i * StepM;
                    if (sAhead > horizonM) sAhead = horizonM;
                    raw.Add(route.PointAtS(startS + sAhead));
                    if (sAhead >= horizonM - 0.01f) break;
                }
                if (raw.Count < 3)
                {
                    r.Detail = "raw-short";
                    return r;
                }

                r.RawMaxKappa = MaxCurvature(raw);

                for (int i = 0; i < raw.Count; i++)
                {
                    float sAhead = i == raw.Count - 1 ? horizonM : i * StepM;
                    if (sAhead > horizonM) sAhead = horizonM;
                    float sAbs = startS + sAhead;
                    Vector3 pRaw = raw[i];
                    Vector3 pSmooth = SmoothAt(route, sAbs, windowM);
                    Vector3 pSafe = KeepOnRoad(pRaw, pSmooth, out bool constrained);
                    if (constrained) r.RoadConstrainedPoints++;
                    r.Path.Add(pSafe);
                }

                // Remove collapsed points before assigning executable station S.
                for (int i = r.Path.Count - 2; i >= 0; i--)
                {
                    if (RaceMath.FlatDistance(r.Path[i], r.Path[i + 1]) < 0.35f)
                        r.Path.RemoveAt(i + 1);
                }
                if (r.Path.Count < 3)
                {
                    r.Detail = "smooth-short";
                    return r;
                }

                float acc = 0f;
                r.StationS.Add(0f);
                for (int i = 1; i < r.Path.Count; i++)
                {
                    acc += RaceMath.FlatDistance(r.Path[i - 1], r.Path[i]);
                    r.StationS.Add(acc);
                }

                MeasureRoadEnvelope(r);

                r.MaxKappa = MaxCurvature(r.Path);
                r.MaxHeadingStepDeg = MaxHeadingStep(r.Path);
                r.Valid = acc >= Math.Min(18f, horizonM * 0.65f)
                    && r.LeftRoadM.Count == r.Path.Count
                    && r.RightRoadM.Count == r.Path.Count;
                float minL = Min(r.LeftRoadM);
                float minR = Min(r.RightRoadM);
                r.Detail = $"window={windowM:F1};rawK={r.RawMaxKappa:F3};refK={r.MaxKappa:F3};"
                    + $"headStep={r.MaxHeadingStepDeg:F0};roadClamp={r.RoadConstrainedPoints};"
                    + $"roadLR={minL:F1}/{minR:F1};pts={r.Path.Count}";
                return r;
            }
            catch (Exception ex)
            {
                r.Detail = "exc:" + ex.Message;
                return r;
            }
        }

        private static void MeasureRoadEnvelope(Result r)
        {
            r.LeftRoadM.Clear();
            r.RightRoadM.Clear();
            for (int i = 0; i < r.Path.Count; i++)
            {
                Vector3 dir;
                if (i <= 0)
                    dir = new Vector3(r.Path[1].X - r.Path[0].X, r.Path[1].Y - r.Path[0].Y, 0f);
                else if (i >= r.Path.Count - 1)
                    dir = new Vector3(r.Path[i].X - r.Path[i - 1].X, r.Path[i].Y - r.Path[i - 1].Y, 0f);
                else
                    dir = new Vector3(r.Path[i + 1].X - r.Path[i - 1].X, r.Path[i + 1].Y - r.Path[i - 1].Y, 0f);
                dir = RaceMath.FlatNormalize(dir);
                var left = new Vector3(-dir.Y, dir.X, 0f);
                r.LeftRoadM.Add(ProbeRoadSide(r.Path[i], left));
                r.RightRoadM.Add(ProbeRoadSide(r.Path[i], new Vector3(-left.X, -left.Y, 0f)));
            }
        }

        private static float ProbeRoadSide(Vector3 center, Vector3 side)
        {
            // The reference point itself has already been constrained onto the
            // drivable road. Sweep outward in the reference frame. A short
            // single off-road hole does not terminate the sweep immediately
            // (junction markings/bridge quirks can flicker IS_POINT_ON_ROAD).
            float lastGood = 0.75f;
            int misses = 0;
            for (float d = 0.5f; d <= 10f; d += 0.5f)
            {
                var p = new Vector3(center.X + side.X * d, center.Y + side.Y * d, center.Z);
                if (IsOnRoad(p))
                {
                    lastGood = d;
                    misses = 0;
                }
                else
                {
                    misses++;
                    if (misses >= 2) break;
                }
            }
            return RaceMath.Clamp(lastGood, 0.75f, 10f);
        }

        private static float Min(List<float> xs)
        {
            if (xs == null || xs.Count == 0) return 0f;
            float m = float.MaxValue;
            for (int i = 0; i < xs.Count; i++) if (xs[i] < m) m = xs[i];
            return m == float.MaxValue ? 0f : m;
        }

        private static Vector3 SmoothAt(RaceRoute route, float s, float windowM)
        {
            // Symmetric triangular kernel along route arclength. It removes
            // 5 m GPS zig-zags and naturally rounds junction corners over
            // roughly 2*windowM instead of handing the controller a vertex.
            float[] f = { -1f, -0.66f, -0.33f, 0f, 0.33f, 0.66f, 1f };
            float[] w = { 1f, 2f, 3f, 4f, 3f, 2f, 1f };
            float x = 0f, y = 0f, z = 0f, sw = 0f;
            for (int i = 0; i < f.Length; i++)
            {
                float si = RaceMath.Clamp(s + f[i] * windowM, 0f, route.TotalLength);
                Vector3 p = route.PointAtS(si);
                x += p.X * w[i];
                y += p.Y * w[i];
                z += p.Z * w[i];
                sw += w[i];
            }
            if (sw < 0.1f) return route.PointAtS(s);
            return new Vector3(x / sw, y / sw, z / sw);
        }

        private static Vector3 KeepOnRoad(Vector3 raw, Vector3 smooth, out bool constrained)
        {
            constrained = false;
            if (IsOnRoad(smooth)) return smooth;

            // The averaged corner may cut the inside sidewalk/building. Walk
            // it back toward GTA's GPS point until it is on drivable road.
            constrained = true;
            float[] blends = { 0.75f, 0.50f, 0.25f, 0f };
            foreach (float t in blends)
            {
                var p = new Vector3(
                    raw.X + (smooth.X - raw.X) * t,
                    raw.Y + (smooth.Y - raw.Y) * t,
                    raw.Z + (smooth.Z - raw.Z) * t);
                if (IsOnRoad(p)) return p;
            }
            return raw;
        }

        private static bool IsOnRoad(Vector3 p)
        {
            try
            {
                if (Function.Call<bool>(Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z, 0))
                    return true;
            }
            catch
            {
                // If the native is unavailable, do not reject the smoothing
                // layer entirely; raw GTA GPS remains the fallback below.
                return true;
            }
            try
            {
                return Function.Call<bool>(Hash.IS_POINT_ON_ROAD, p.X, p.Y, p.Z - 1f, 0);
            }
            catch { return true; }
        }

        private static float MaxCurvature(List<Vector3> path)
        {
            float max = 0f;
            if (path == null || path.Count < 3) return max;
            for (int i = 1; i < path.Count - 1; i++)
            {
                var d0 = new Vector3(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y, 0f);
                var d1 = new Vector3(path[i + 1].X - path[i].X, path[i + 1].Y - path[i].Y, 0f);
                float l0 = RaceMath.FlatLength(d0);
                float l1 = RaceMath.FlatLength(d1);
                if (l0 < 0.35f || l1 < 0.35f) continue;
                float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
                float k = dh / Math.Max((l0 + l1) * 0.5f, 0.5f);
                if (k > max) max = k;
            }
            return max;
        }

        private static float MaxHeadingStep(List<Vector3> path)
        {
            float max = 0f;
            if (path == null || path.Count < 3) return max;
            Vector3 prev = RaceMath.FlatNormalize(new Vector3(
                path[1].X - path[0].X, path[1].Y - path[0].Y, 0f));
            for (int i = 1; i < path.Count - 1; i++)
            {
                Vector3 next = RaceMath.FlatNormalize(new Vector3(
                    path[i + 1].X - path[i].X, path[i + 1].Y - path[i].Y, 0f));
                float d = Math.Abs(RaceMath.SignedAngleDeg(prev, next));
                if (d > max) max = d;
                prev = next;
            }
            return max;
        }
    }
}
