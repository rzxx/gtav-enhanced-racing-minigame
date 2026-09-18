using System;
using System.Collections.Generic;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Structural local-road model for the Simple planner.
    ///
    /// IMPORTANT: IS_POINT_ON_ROAD is NOT a width sensor. A single boolean
    /// miss must never collapse the planner's drivable envelope.
    ///
    /// This model anchors each DrivingReference station to GTA's vehicle-node
    /// graph and closest road edge, derives road direction + lane count, then
    /// estimates a stable carriageway envelope in the SAME frame as the
    /// executable reference. Missing observations are filled from neighboring
    /// structural samples with explicitly low confidence.
    internal static class LocalRoadModel
    {
        private const float NominalLaneWidthM = 3.25f;
        private const float EdgeShoulderM = 0.35f;
        private const float MinSideM = 1.55f;
        private const float MaxSideM = 16f;

        private struct Obs
        {
            public bool Valid;
            public Vector3 Center;
            public float Heading;
            public int Lanes;
            public int ForwardLanes;
            public int BackwardLanes;
            public float MedianWidth;
            public float LeftM;
            public float RightM;
            public float Confidence;
            public string Source;
        }

        public static void Populate(DrivingReference.Result r)
        {
            if (r == null) return;
            r.LeftRoadM.Clear();
            r.RightRoadM.Clear();
            r.RoadConfidence.Clear();
            r.RoadSource.Clear();
            r.RoadLaneCount.Clear();
            r.RoadCenter.Clear();
            r.RoadHeadingDeg.Clear();

            int n = r.Path != null ? r.Path.Count : 0;
            if (n < 2) return;

            var obs = new Obs[n];
            for (int i = 0; i < n; i++)
            {
                Vector3 dir = DirectionAt(r.Path, i);
                float refHeading = RaceMath.HeadingFromVector(dir);
                obs[i] = Observe(r.Path[i], refHeading);
            }

            // Fill holes from structural neighbors. Unknown means uncertain,
            // never "road width = almost zero".
            for (int i = 0; i < n; i++)
            {
                if (obs[i].Valid) continue;
                int a = FindPrev(obs, i);
                int b = FindNext(obs, i);
                if (a >= 0 && b >= 0)
                {
                    float span = Math.Max(1f, b - a);
                    float t = (i - a) / span;
                    obs[i] = Interpolate(obs[a], obs[b], t);
                    obs[i].Confidence = Math.Min(0.38f, Math.Min(obs[a].Confidence, obs[b].Confidence) * 0.55f);
                    obs[i].Source = "Continuity";
                }
                else if (a >= 0)
                {
                    obs[i] = obs[a];
                    obs[i].Confidence = Math.Min(0.30f, obs[a].Confidence * 0.45f);
                    obs[i].Source = "CarryPrev";
                }
                else if (b >= 0)
                {
                    obs[i] = obs[b];
                    obs[i].Confidence = Math.Min(0.30f, obs[b].Confidence * 0.45f);
                    obs[i].Source = "CarryNext";
                }
                else
                {
                    obs[i] = Fallback(r.Path[i], DirectionAt(r.Path, i));
                }
            }

            // Structural continuity filter. Road width can change at junctions,
            // but a single 4 m sample must not turn a wide street into an alley.
            for (int i = 0; i < n; i++)
            {
                float medL = NeighborhoodMedian(obs, i, true);
                float medR = NeighborhoodMedian(obs, i, false);
                if (obs[i].Confidence < 0.72f)
                {
                    obs[i].LeftM = ClampToward(obs[i].LeftM, medL, 1.75f);
                    obs[i].RightM = ClampToward(obs[i].RightM, medR, 1.75f);
                }
                obs[i].LeftM = RaceMath.Clamp(obs[i].LeftM, MinSideM, MaxSideM);
                obs[i].RightM = RaceMath.Clamp(obs[i].RightM, MinSideM, MaxSideM);
            }

            for (int i = 0; i < n; i++)
            {
                r.LeftRoadM.Add(obs[i].LeftM);
                r.RightRoadM.Add(obs[i].RightM);
                r.RoadConfidence.Add(obs[i].Confidence);
                r.RoadSource.Add(obs[i].Source ?? "?");
                r.RoadLaneCount.Add(obs[i].Lanes);
                r.RoadCenter.Add(obs[i].Center);
                r.RoadHeadingDeg.Add(obs[i].Heading);
            }
        }

        private static Obs Observe(Vector3 p, float refHeading)
        {
            var node = TryNode(p, refHeading);
            var edge = TryClosestRoad(p, refHeading);

            if (!node.Valid && !edge.Valid)
                return new Obs();

            Vector3 center = node.Valid ? node.Center : edge.Center;
            float heading = node.Valid ? node.Heading : edge.Heading;
            int lanes = 0;
            if (node.Valid) lanes = node.Lanes;
            if (edge.Valid)
            {
                int edgeLanes = edge.ForwardLanes + edge.BackwardLanes;
                if (edgeLanes > lanes) lanes = edgeLanes;
            }
            lanes = Math.Max(1, Math.Min(8, lanes));

            float median = edge.Valid ? RaceMath.Clamp(edge.MedianWidth, 0f, 8f) : 0f;
            float half = (lanes * NominalLaneWidthM + median) * 0.5f + EdgeShoulderM;
            half = RaceMath.Clamp(half, 1.8f, 15f);

            Vector3 dir = RaceMath.VectorFromHeading(heading);
            Vector3 left = new Vector3(-dir.Y, dir.X, 0f);
            Vector3 rel = new Vector3(p.X - center.X, p.Y - center.Y, 0f);
            float lateral = RaceMath.FlatDot(rel, left);

            float leftM = half - lateral;
            float rightM = half + lateral;

            // If the selected node center is implausibly offset, keep the width
            // but do not let that uncertain center make one side disappear.
            float maxReasonableOffset = Math.Max(half - 0.75f, 1.0f);
            if (Math.Abs(lateral) > maxReasonableOffset)
            {
                lateral = RaceMath.Clamp(lateral, -half * 0.45f, half * 0.45f);
                leftM = half - lateral;
                rightM = half + lateral;
            }

            float confidence;
            string source;
            if (node.Valid && edge.Valid)
            {
                float axis = AxisHeadingError(node.Heading, edge.Heading);
                confidence = axis < 18f ? 0.92f : 0.78f;
                source = "Node+Edge";
            }
            else if (node.Valid)
            {
                confidence = 0.72f;
                source = "Node";
            }
            else
            {
                confidence = 0.58f;
                source = "Edge";
            }

            return new Obs
            {
                Valid = true,
                Center = center,
                Heading = heading,
                Lanes = lanes,
                ForwardLanes = edge.ForwardLanes,
                BackwardLanes = edge.BackwardLanes,
                MedianWidth = median,
                LeftM = RaceMath.Clamp(leftM, MinSideM, MaxSideM),
                RightM = RaceMath.Clamp(rightM, MinSideM, MaxSideM),
                Confidence = confidence,
                Source = source,
            };
        }

        private static Obs TryNode(Vector3 p, float refHeading)
        {
            var best = new Obs();
            float bestScore = float.MaxValue;
            for (int nth = 1; nth <= 5; nth++)
            {
                try
                {
                    var posOut = new OutputArgument();
                    var headOut = new OutputArgument();
                    var lanesOut = new OutputArgument();
                    bool ok = Function.Call<bool>(Hash.GET_NTH_CLOSEST_VEHICLE_NODE_WITH_HEADING,
                        p.X, p.Y, p.Z, nth,
                        posOut, headOut, lanesOut,
                        0, 3.0f, 6.0f);
                    if (!ok) continue;

                    Vector3 np = posOut.GetResult<Vector3>();
                    float nh = headOut.GetResult<float>();
                    int lanes = lanesOut.GetResult<int>();
                    float dist = RaceMath.FlatDistance(p, np);
                    float z = Math.Abs(p.Z - np.Z);
                    float axis = AxisHeadingError(refHeading, nh);
                    if (dist > 26f || z > 7f || axis > 55f) continue;

                    float score = dist + z * 2.5f + axis * 0.18f;
                    if (score >= bestScore) continue;
                    bestScore = score;
                    best = new Obs
                    {
                        Valid = true,
                        Center = np,
                        Heading = AlignHeadingAxis(nh, refHeading),
                        Lanes = Math.Max(1, Math.Min(8, lanes)),
                    };
                }
                catch { }
            }
            return best;
        }

        private static Obs TryClosestRoad(Vector3 p, float refHeading)
        {
            try
            {
                var srcOut = new OutputArgument();
                var dstOut = new OutputArgument();
                var fOut = new OutputArgument();
                var bOut = new OutputArgument();
                var widthOut = new OutputArgument();
                bool ok = Function.Call<bool>(Hash.GET_CLOSEST_ROAD,
                    p.X, p.Y, p.Z,
                    2.0f, 1,
                    srcOut, dstOut, fOut, bOut, widthOut,
                    false);
                if (!ok) return new Obs();

                Vector3 a = srcOut.GetResult<Vector3>();
                Vector3 b = dstOut.GetResult<Vector3>();
                int f = Math.Max(0, fOut.GetResult<int>());
                int back = Math.Max(0, bOut.GetResult<int>());
                float median = Math.Abs(widthOut.GetResult<float>());

                Vector3 edge = new Vector3(b.X - a.X, b.Y - a.Y, 0f);
                if (RaceMath.FlatLength(edge) < 2f) return new Obs();
                Vector3 dir = RaceMath.FlatNormalize(edge);
                float h = RaceMath.HeadingFromVector(dir);
                float axis = AxisHeadingError(refHeading, h);
                if (axis > 55f) return new Obs();

                Vector3 center = ClosestPointOnSegment2D(p, a, b);
                if (RaceMath.FlatDistance(p, center) > 22f) return new Obs();
                if (Math.Abs(p.Z - center.Z) > 7f) return new Obs();

                return new Obs
                {
                    Valid = true,
                    Center = center,
                    Heading = AlignHeadingAxis(h, refHeading),
                    Lanes = Math.Max(1, Math.Min(8, f + back)),
                    ForwardLanes = f,
                    BackwardLanes = back,
                    MedianWidth = median,
                };
            }
            catch { return new Obs(); }
        }

        private static Obs Fallback(Vector3 p, Vector3 dir)
        {
            return new Obs
            {
                Valid = true,
                Center = p,
                Heading = RaceMath.HeadingFromVector(dir),
                Lanes = 2,
                LeftM = 3.6f,
                RightM = 3.6f,
                Confidence = 0.18f,
                Source = "UnknownFallback",
            };
        }

        private static int FindPrev(Obs[] xs, int i)
        {
            for (int k = i - 1; k >= 0; k--) if (xs[k].Valid) return k;
            return -1;
        }

        private static int FindNext(Obs[] xs, int i)
        {
            for (int k = i + 1; k < xs.Length; k++) if (xs[k].Valid) return k;
            return -1;
        }

        private static Obs Interpolate(Obs a, Obs b, float t)
        {
            return new Obs
            {
                Valid = true,
                Center = new Vector3(
                    a.Center.X + (b.Center.X - a.Center.X) * t,
                    a.Center.Y + (b.Center.Y - a.Center.Y) * t,
                    a.Center.Z + (b.Center.Z - a.Center.Z) * t),
                Heading = a.Heading,
                Lanes = t < 0.5f ? a.Lanes : b.Lanes,
                LeftM = a.LeftM + (b.LeftM - a.LeftM) * t,
                RightM = a.RightM + (b.RightM - a.RightM) * t,
                Confidence = Math.Min(a.Confidence, b.Confidence),
                Source = "Continuity",
            };
        }

        private static float NeighborhoodMedian(Obs[] xs, int i, bool left)
        {
            var vals = new List<float>();
            for (int k = Math.Max(0, i - 2); k <= Math.Min(xs.Length - 1, i + 2); k++)
            {
                if (!xs[k].Valid) continue;
                vals.Add(left ? xs[k].LeftM : xs[k].RightM);
            }
            if (vals.Count == 0) return 3.6f;
            vals.Sort();
            return vals[vals.Count / 2];
        }

        private static float ClampToward(float value, float target, float maxDelta)
        {
            if (value > target + maxDelta) return target + maxDelta;
            if (value < target - maxDelta) return target - maxDelta;
            return value;
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

        private static float AxisHeadingError(float a, float b)
        {
            float d = Math.Abs(RaceMath.HeadingDiffDeg(a, b));
            return Math.Min(d, Math.Abs(180f - d));
        }

        private static float AlignHeadingAxis(float h, float reference)
        {
            if (Math.Abs(RaceMath.HeadingDiffDeg(h, reference)) <= 90f) return h;
            float x = h + 180f;
            while (x >= 360f) x -= 360f;
            while (x < 0f) x += 360f;
            return x;
        }

        private static Vector3 ClosestPointOnSegment2D(Vector3 p, Vector3 a, Vector3 b)
        {
            float dx = b.X - a.X;
            float dy = b.Y - a.Y;
            float den = dx * dx + dy * dy;
            if (den < 1e-4f) return a;
            float t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / den;
            t = RaceMath.Clamp(t, 0f, 1f);
            return new Vector3(
                a.X + dx * t,
                a.Y + dy * t,
                a.Z + (b.Z - a.Z) * t);
        }
    }
}
