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
    /// continuous local center reference. Smoothing is geometrically bounded
    /// around GTA's routed spine; road structure is supplied separately by
    /// LocalRoadModel, never by a binary IS_POINT_ON_ROAD width scan.
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
            public readonly List<float> RoadConfidence = new List<float>();
            public readonly List<string> RoadSource = new List<string>();
            public readonly List<int> RoadLaneCount = new List<int>();
            public readonly List<Vector3> RoadCenter = new List<Vector3>();
            public readonly List<float> RoadHeadingDeg = new List<float>();
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

        private Result previousRoad;

        public void Reset()
        {
            previousRoad = null;
        }

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
                    Vector3 pSafe = KeepNearGpsSpine(pRaw, pSmooth, out bool constrained);
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

                LocalRoadModel.Populate(r);
                StabilizeRoadTemporally(r, previousRoad);

                r.MaxKappa = MaxCurvature(r.Path);
                r.MaxHeadingStepDeg = MaxHeadingStep(r.Path);
                r.Valid = acc >= Math.Min(18f, horizonM * 0.65f)
                    && r.LeftRoadM.Count == r.Path.Count
                    && r.RightRoadM.Count == r.Path.Count
                    && r.RoadConfidence.Count == r.Path.Count;
                float minL = Min(r.LeftRoadM);
                float minR = Min(r.RightRoadM);
                float minConf = Min(r.RoadConfidence);
                r.Detail = $"window={windowM:F1};rawK={r.RawMaxKappa:F3};refK={r.MaxKappa:F3};"
                    + $"headStep={r.MaxHeadingStepDeg:F0};roadClamp={r.RoadConstrainedPoints};"
                    + $"roadLR={minL:F1}/{minR:F1};roadConf={minConf:F2};pts={r.Path.Count}";
                previousRoad = r;
                return r;
            }
            catch (Exception ex)
            {
                r.Detail = "exc:" + ex.Message;
                return r;
            }
        }

        private static void StabilizeRoadTemporally(Result current, Result previous)
        {
            if (current == null || previous == null
                || current.Path == null || previous.Path == null
                || previous.Path.Count == 0) return;
            try
            {
                for (int i = 0; i < current.Path.Count; i++)
                {
                    int best = -1;
                    float bestD = 7.0f;
                    Vector3 cd = DirectionAt(current.Path, i);
                    float ch = RaceMath.HeadingFromVector(cd);
                    for (int j = 0; j < previous.Path.Count; j++)
                    {
                        float d = RaceMath.FlatDistance(current.Path[i], previous.Path[j]);
                        if (d >= bestD) continue;
                        Vector3 pd = DirectionAt(previous.Path, j);
                        float ph = RaceMath.HeadingFromVector(pd);
                        float axis = Math.Abs(RaceMath.HeadingDiffDeg(ch, ph));
                        axis = Math.Min(axis, Math.Abs(180f - axis));
                        if (axis > 30f) continue;
                        best = j;
                        bestD = d;
                    }
                    if (best < 0
                        || i >= current.RoadConfidence.Count
                        || best >= previous.RoadConfidence.Count
                        || i >= current.LeftRoadM.Count
                        || best >= previous.LeftRoadM.Count) continue;

                    float nc = current.RoadConfidence[i];
                    float pc = previous.RoadConfidence[best];
                    float alpha = nc >= 0.72f ? 0.72f : nc >= 0.45f ? 0.48f : 0.24f;
                    if (pc < 0.35f && nc > pc) alpha = 0.80f;

                    current.LeftRoadM[i] =
                        previous.LeftRoadM[best] * (1f - alpha) + current.LeftRoadM[i] * alpha;
                    current.RightRoadM[i] =
                        previous.RightRoadM[best] * (1f - alpha) + current.RightRoadM[i] * alpha;

                    // Confidence itself should not spike from a weak sample.
                    if (nc < pc)
                        current.RoadConfidence[i] = pc * (1f - alpha) + nc * alpha;
                }
            }
            catch { }
        }

        private static Vector3 DirectionAt(IList<Vector3> path, int i)
        {
            Vector3 d;
            if (i <= 0)
                d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, 0f);
            else if (i >= path.Count - 1)
                d = new Vector3(path[i].X - path[i - 1].X, path[i].Y - path[i - 1].Y, 0f);
            else
                d = new Vector3(path[i + 1].X - path[i - 1].X, path[i + 1].Y - path[i - 1].Y, 0f);
            if (RaceMath.FlatLength(d) < 0.2f) return new Vector3(0f, 1f, 0f);
            return RaceMath.FlatNormalize(d);
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

        private static Vector3 KeepNearGpsSpine(Vector3 raw, Vector3 smooth, out bool constrained)
        {
            // GPS is topology, but it is still a far more reliable statement
            // that "a road exists here" than a one-frame IS_POINT_ON_ROAD
            // boolean. Allow smoothing to round/kink-filter the route while
            // bounding how far it may cut away from GTA's own routed spine.
            constrained = false;
            float d = RaceMath.FlatDistance(raw, smooth);
            const float MaxOffsetM = 3.5f;
            if (d <= MaxOffsetM) return smooth;

            constrained = true;
            float t = MaxOffsetM / Math.Max(d, 0.01f);
            return new Vector3(
                raw.X + (smooth.X - raw.X) * t,
                raw.Y + (smooth.Y - raw.Y) * t,
                raw.Z + (smooth.Z - raw.Z) * t);
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
