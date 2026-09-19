using System;
using System.Collections.Generic;
using System.Diagnostics;
using GTA.Math;

namespace StreetRacing
{
    /// World-space / surface-constrained spatial planner.
    ///
    /// Search nodes are lightweight parent-pointer states. Full trajectories
    /// are reconstructed only for the best leaves, avoiding the V1 behavior of
    /// copying Path/Heading lists for thousands of child nodes every plan.
    internal sealed class SpatialPlannerV1
    {
        internal sealed class Result
        {
            public bool Valid;
            public TrajectoryCandidate Chosen;
            public readonly List<TrajectoryCandidate> Candidates = new List<TrajectoryCandidate>();
            public string Intent = "Spatial";
            public string Detail = "";
            public float RoadDesired;
            public float Desired;
        }

        private struct SearchNode
        {
            public Vector3 Pos;
            public float HeadingDeg;
            public float TimeS;
            public float Cost;
            public float GoalDist;
            public float FirstCurvature;
            public float LastCurvature;
            public float Curvature;
            public int ParentIndex;
        }

        private static readonly float[] CurvatureSet =
            { -0.075f, -0.045f, -0.022f, 0f, 0.022f, 0.045f, 0.075f };

        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public TrajectoryCandidate LastChosen;
        public bool HasChosen;
        public string LastDecision { get; private set; } = "";
        public float LastPlanMs { get; private set; }
        public int LastPoolCount { get; private set; }

        private readonly List<SearchNode> pool = new List<SearchNode>(1600);
        private readonly List<int> beam = new List<int>(32);
        private readonly List<int> expanded = new List<int>(256);
        private readonly List<int> nextBeam = new List<int>(32);
        private readonly HashSet<long> diversity = new HashSet<long>();
        private readonly List<int> chain = new List<int>(16);

        private float lastFirstCurvature;
        private bool hasLastCurvature;
        private int planId;

        private const float PrimitiveM = 8f;
        private const float SampleM = 2f;
        private const int Layers = 7;
        private const int BeamWidth = 24;

        public void Reset()
        {
            LastCandidates.Clear();
            LastChosen = new TrajectoryCandidate();
            HasChosen = false;
            LastDecision = "";
            LastPlanMs = 0f;
            LastPoolCount = 0;
            lastFirstCurvature = 0f;
            hasLastCurvature = false;
            planId = 0;
            pool.Clear();
            beam.Clear();
            expanded.Clear();
            nextBeam.Clear();
            diversity.Clear();
            chain.Clear();
        }

        public Result Plan(
            LocalWorldModel world,
            RaceRoute route,
            VehicleCapability capability,
            DriverProfile profile,
            Vector3 egoPos,
            float egoHeading,
            float egoSpeed,
            float cruise,
            int nowMs)
        {
            long perfStart = Stopwatch.GetTimestamp();
            var result = new Result();
            LastCandidates.Clear();
            HasChosen = false;
            planId++;

            if (world == null || world.Road.Count == 0 || route == null || !route.Built)
            {
                result.Detail = "world-or-route-invalid";
                return result;
            }

            float aLat = capability != null ? capability.UsableLat(profile.GripFactor) : 7f;
            float aBrake = capability != null ? capability.UsableBrake(profile.GripFactor) : 6f;
            aLat = RaceMath.Clamp(aLat, 3f, 11.5f);
            aBrake = RaceMath.Clamp(aBrake, 3.5f, 11.5f);

            float searchSpeed = RaceMath.Clamp(Math.Max(egoSpeed, 8f), 8f, Math.Min(cruise, 20f));
            float goalS = Math.Min(route.TotalLength, route.AlongS + 75f);
            Vector3 spatialGoal = route.PointAtS(goalS);

            pool.Clear();
            beam.Clear();
            var root = new SearchNode
            {
                Pos = egoPos,
                HeadingDeg = egoHeading,
                TimeS = 0f,
                Cost = 0f,
                GoalDist = RaceMath.FlatDistance(egoPos, spatialGoal),
                FirstCurvature = 0f,
                LastCurvature = 0f,
                Curvature = 0f,
                ParentIndex = -1,
            };
            pool.Add(root);
            beam.Add(0);

            for (int layer = 0; layer < Layers; layer++)
            {
                expanded.Clear();

                for (int bi = 0; bi < beam.Count; bi++)
                {
                    int parentIndex = beam[bi];
                    SearchNode parent = pool[parentIndex];

                    for (int ki = 0; ki < CurvatureSet.Length; ki++)
                    {
                        float curvature = CurvatureSet[ki];
                        if (layer == 0 && Math.Abs(curvature - parent.LastCurvature) > 0.10f)
                            continue;

                        SearchNode child = Expand(
                            parent, parentIndex, curvature, searchSpeed,
                            world, spatialGoal, aLat, layer);
                        int childIndex = pool.Count;
                        pool.Add(child);
                        expanded.Add(childIndex);
                    }
                }

                if (expanded.Count == 0) break;
                expanded.Sort(ComparePoolCost);
                BuildDiverseBeam(expanded, BeamWidth);

                beam.Clear();
                for (int i = 0; i < nextBeam.Count; i++)
                    beam.Add(nextBeam[i]);
            }

            if (beam.Count == 0)
            {
                result.Detail = "search-empty";
                return result;
            }

            beam.Sort(ComparePoolCost);
            int emit = Math.Min(10, beam.Count);
            for (int i = 0; i < emit; i++)
            {
                int leafIndex = beam[i];
                var c = FinalizeCandidate(
                    leafIndex, world, route, aLat, aBrake,
                    cruise, egoSpeed, searchSpeed, i);
                LastCandidates.Add(c);
            }

            int best = -1;
            float bestScore = float.MinValue;
            for (int i = 0; i < LastCandidates.Count; i++)
            {
                var c = LastCandidates[i];
                if (!string.IsNullOrEmpty(c.RejectReason)) continue;
                if (c.Score > bestScore)
                {
                    bestScore = c.Score;
                    best = i;
                }
            }
            if (best < 0)
            {
                result.Detail = "no-final-candidate";
                return result;
            }

            var chosen = LastCandidates[best];
            lastFirstCurvature = ParseFirstCurvature(chosen.Shape);
            hasLastCurvature = true;

            Vector3 ef = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(egoHeading));
            Vector3 el = new Vector3(-ef.Y, ef.X, 0f);
            Vector3 delta = new Vector3(
                chosen.AimPoint.X - egoPos.X,
                chosen.AimPoint.Y - egoPos.Y, 0f);
            float lateralEnd = RaceMath.FlatDot(delta, el);

            string intent;
            if (chosen.ConstrainHandle != -1 && chosen.TargetSpeed < cruise - 1.0f)
                intent = "Follow";
            else if (lateralEnd > 2.0f)
                intent = "SpatialLeft";
            else if (lateralEnd < -2.0f)
                intent = "SpatialRight";
            else
                intent = "SpatialTrack";

            result.Valid = true;
            result.Chosen = chosen;
            result.Candidates.AddRange(LastCandidates);
            result.Intent = intent;
            result.RoadDesired = chosen.RoadTargetSpeed;
            result.Desired = chosen.TargetSpeed;
            LastPoolCount = pool.Count;
            LastPlanMs = (float)((Stopwatch.GetTimestamp() - perfStart) * 1000.0 / Stopwatch.Frequency);
            result.Detail = $"intent={intent};score={chosen.Score:F1};meanV={chosen.MeanSpeed:F1};"
                + $"minV={chosen.MinSpeed:F1};clear={chosen.MinPredClearance:F1};"
                + $"constr={(chosen.ConstrainHandle != -1 ? chosen.ConstrainKind + "#" + chosen.ConstrainHandle : "none")};"
                + $"opp={chosen.OpposingFraction:F2};unknown={chosen.UnknownFraction:F2};"
                + $"flow={chosen.MeanFlowCost:F2};dz={chosen.ElevationDeltaM:F1};"
                + $"{world.Detail};planMs={LastPlanMs:F1};pool={pool.Count};beam={beam.Count};cand={LastCandidates.Count};"
                + $"top={Summarize(LastCandidates)}";

            LastChosen = chosen;
            HasChosen = true;
            LastDecision = result.Detail;
            return result;
        }

        private SearchNode Expand(
            SearchNode parent,
            int parentIndex,
            float curvature,
            float searchSpeed,
            LocalWorldModel world,
            Vector3 spatialGoal,
            float aLat,
            int layer)
        {
            var n = new SearchNode
            {
                Pos = parent.Pos,
                HeadingDeg = parent.HeadingDeg,
                TimeS = parent.TimeS,
                Cost = parent.Cost,
                GoalDist = parent.GoalDist,
                FirstCurvature = layer == 0 ? curvature : parent.FirstCurvature,
                LastCurvature = curvature,
                Curvature = curvature,
                ParentIndex = parentIndex,
            };

            if (layer == 0 && hasLastCurvature)
                n.Cost += Math.Abs(curvature - lastFirstCurvature) * 55f;
            n.Cost += Math.Abs(curvature - parent.LastCurvature) * 18f;
            n.Cost += Math.Abs(curvature) * 5f;

            float feasibleV = Math.Abs(curvature) < 0.002f
                ? searchSpeed
                : (float)Math.Sqrt(aLat / Math.Abs(curvature));
            float primitiveSpeed = RaceMath.Clamp(Math.Min(searchSpeed, feasibleV), 4f, searchSpeed);
            int count = Math.Max(1, (int)Math.Round(PrimitiveM / SampleM));

            for (int s = 0; s < count; s++)
            {
                float ds = PrimitiveM / count;
                float dHead = curvature * ds * 180f / (float)Math.PI;
                float midHeading = WrapHeading(n.HeadingDeg + dHead * 0.5f);
                Vector3 fwd = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(midHeading));

                Vector3 candidate = new Vector3(
                    n.Pos.X + fwd.X * ds,
                    n.Pos.Y + fwd.Y * ds,
                    n.Pos.Z);

                float newHeading = WrapHeading(n.HeadingDeg + dHead);
                Vector3 projected;
                float projectedFlow;
                float projectedConfidence;
                bool projectedOpposing;
                if (world.TryProjectToSurface(
                    candidate, newHeading, n.Pos.Z,
                    out projected, out projectedFlow,
                    out projectedConfidence, out projectedOpposing))
                {
                    n.Pos = projected;
                }
                else
                {
                    // Unknown space retains the previous Z until a connected
                    // support is found again. Its surface cost is intentionally
                    // high, so this is not a free building shortcut.
                    n.Pos = candidate;
                }

                n.HeadingDeg = newHeading;
                n.TimeS += ds / Math.Max(primitiveSpeed, 1f);

                var pc = world.EvaluatePose(n.Pos, n.HeadingDeg, n.TimeS);
                n.Cost += pc.SurfaceCost * ds * 0.42f;
                n.Cost += pc.FlowCost * ds * 0.95f;
                n.Cost += pc.ActorCost * 0.12f;
                if (pc.HardCollision)
                    n.Cost += 90f;
            }

            float newGoalDist = RaceMath.FlatDistance(n.Pos, spatialGoal);
            float goalProgress = parent.GoalDist - newGoalDist;
            n.GoalDist = newGoalDist;
            n.Cost -= goalProgress * 3.4f;
            if (goalProgress < -1f)
                n.Cost += Math.Abs(goalProgress) * 9f;

            Vector3 toGoal = new Vector3(
                spatialGoal.X - n.Pos.X,
                spatialGoal.Y - n.Pos.Y, 0f);
            if (RaceMath.FlatLength(toGoal) > 2f)
            {
                float goalHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(toGoal));
                float headErr = Math.Abs(RaceMath.HeadingDiffDeg(goalHeading, n.HeadingDeg));
                n.Cost += Math.Min(headErr, 100f) * 0.025f;
            }
            return n;
        }

        private TrajectoryCandidate FinalizeCandidate(
            int leafIndex,
            LocalWorldModel world,
            RaceRoute route,
            float aLat,
            float aBrake,
            float cruise,
            float egoSpeed,
            float searchSpeed,
            int index)
        {
            List<Vector3> path;
            List<float> headings;
            ReconstructPath(leafIndex, world, searchSpeed, aLat, out path, out headings);

            SearchNode leaf = pool[leafIndex];
            var c = new TrajectoryCandidate
            {
                CandidateIndex = index,
                Shape = $"Spatial:{leaf.FirstCurvature:+0.000;-0.000;0.000}",
                Path = path,
                StationS = BuildStationS(path),
                AimPoint = path[path.Count - 1],
                RejectReason = "",
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
                MinRoadConfidence = 1f,
                Score = -leaf.Cost,
                FirstTangentErrDeg = 0f,
                RouteHeadErrDeg = route.HeadingErrorDeg,
            };

            int n = c.Path.Count;
            var roadAllow = new float[n];
            float maxK = 0f;
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureAt(c.Path, i);
                if (k > maxK) maxK = k;
                float v = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                roadAllow[i] = RaceMath.Clamp(v, 0f, cruise);
            }
            c.MaxKappa = maxK;

            var roadProfile = BackwardPass(roadAllow, c.StationS, aBrake);
            c.RoadTargetSpeed = roadProfile[0];

            var arrival = BuildArrivalTimes(c.StationS, roadProfile);
            var allow = new float[n];
            Array.Copy(roadAllow, allow, n);

            float earliestStopS = float.MaxValue;
            int blocker = -1;
            string blockerKind = "";
            float minClear = 999f;
            int opposingSamples = 0;
            int unknownSamples = 0;
            float flowCostSum = 0f;

            for (int i = 0; i < n; i++)
            {
                float h = i < headings.Count ? headings[i] : HeadingAt(c.Path, i);
                var pc = world.EvaluatePose(c.Path[i], h, arrival[i]);
                if (pc.RoadConfidence < c.MinRoadConfidence)
                    c.MinRoadConfidence = pc.RoadConfidence;
                if (pc.ClearanceM < minClear)
                    minClear = pc.ClearanceM;
                if (pc.OpposingSide) opposingSamples++;
                if (!pc.OnRoad) unknownSamples++;
                flowCostSum += pc.FlowCost;

                // Every trajectory shares station zero. Proximity at the
                // immutable current pose must NOT poison every alternative.
                // The first future sample (normally ~2 m) is where candidate
                // geometry has actually begun to diverge.
                if (c.StationS[i] < 1.5f)
                    continue;

                if (pc.BlockingHandle == -1 || pc.ClearanceM >= 1.35f)
                    continue;

                TrackedActor a;
                if (!world.TryGetActor(pc.BlockingHandle, out a))
                    continue;

                // An actor already beside the ego is not a lead
                // blocker merely because the first 2 m sample still overlaps
                // the inflated footprint. Let spatial search create separation.
                bool initiallyLateral = a.Longitudinal < 2.5f
                    && Math.Abs(a.Lateral) > 1.35f
                    && a.ClosingSpeed < 3.0f;
                if (initiallyLateral && c.StationS[i] < 6.0f)
                    continue;

                string kind = a.Kind.ToString();
                float actorHeading = a.HeadingDeg;
                if (RaceMath.FlatLength(a.Velocity) > 1.2f)
                    actorHeading = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(a.Velocity));
                float headErr = Math.Abs(RaceMath.HeadingDiffDeg(h, actorHeading));
                Vector3 pf = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(h));
                float actorAlong = RaceMath.FlatDot(a.Velocity, pf);
                bool sameFlow = (a.Kind == ActorKind.TrafficVehicle || a.Kind == ActorKind.Rival)
                    && headErr < 55f
                    && actorAlong > 0.4f;

                if (pc.ClearanceM <= 0.25f)
                {
                    if (sameFlow)
                    {
                        float followV = Math.Max(0f, actorAlong);
                        allow[i] = Math.Min(allow[i], followV);
                        if (blocker == -1 || c.StationS[i] < c.ConstrainS || c.ConstrainS < 0f)
                        {
                            blocker = a.Handle;
                            blockerKind = kind;
                            c.ConstrainS = c.StationS[i];
                        }
                    }
                    else
                    {
                        float stopS = Math.Max(0f, c.StationS[i] - 3.0f);
                        if (stopS < earliestStopS)
                        {
                            earliestStopS = stopS;
                            blocker = a.Handle;
                            blockerKind = kind;
                        }
                    }
                }
                else if (sameFlow && pc.ClearanceM < 1.0f)
                {
                    // Near a lead car but not predicted to overlap: create a
                    // gentle following envelope, never an s=0 hard stop.
                    float followV = Math.Max(2.0f, actorAlong + 1.0f);
                    allow[i] = Math.Min(allow[i], followV);
                    if (blocker == -1)
                    {
                        blocker = a.Handle;
                        blockerKind = kind;
                        c.ConstrainS = c.StationS[i];
                    }
                }
            }

            if (earliestStopS < float.MaxValue)
            {
                for (int i = 0; i < n; i++)
                    if (c.StationS[i] >= earliestStopS)
                        allow[i] = 0f;
                c.ConstrainS = earliestStopS;
            }

            var desired = BackwardPass(allow, c.StationS, aBrake);
            arrival = BuildArrivalTimes(c.StationS, desired);

            c.SpeedProfile = new List<float>(desired);
            c.ArrivalT = arrival;
            c.TargetSpeed = desired[0];
            c.ConstrainHandle = blocker;
            c.ConstrainKind = blockerKind;
            c.MinPredClearance = minClear;
            c.OpposingFraction = n > 0 ? opposingSamples / (float)n : 0f;
            c.UnknownFraction = n > 0 ? unknownSamples / (float)n : 0f;
            c.MeanFlowCost = n > 0 ? flowCostSum / n : 0f;
            c.ElevationDeltaM = c.Path.Count > 1 ? c.Path[c.Path.Count - 1].Z - c.Path[0].Z : 0f;
            c.MeanSpeed = Mean(desired);
            c.MinSpeed = Min(desired, cruise);
            c.RequiredDecel = Math.Max(0f, egoSpeed - c.TargetSpeed);
            c.LookaheadM = c.StationS[c.StationS.Count - 1];
            c.LateralM = route.ProjectOntoRoute(c.AimPoint).Lateral;
            c.SpeedLimiting = blocker != -1 && c.TargetSpeed < c.RoadTargetSpeed - 0.25f
                ? $"Traffic:{blockerKind}#{blocker}"
                : c.RoadTargetSpeed < cruise - 0.5f ? "Curvature" : "Cruise";

            c.Score += c.MeanSpeed * 3.2f + c.MinSpeed * 0.8f
                + RaceMath.Clamp(minClear, -2f, 6f) * 1.2f;
            return c;
        }

        private void ReconstructPath(
            int leafIndex,
            LocalWorldModel world,
            float searchSpeed,
            float aLat,
            out List<Vector3> path,
            out List<float> headings)
        {
            chain.Clear();
            int idx = leafIndex;
            while (idx >= 0)
            {
                chain.Add(idx);
                idx = pool[idx].ParentIndex;
            }
            chain.Reverse();

            path = new List<Vector3>(1 + Math.Max(0, chain.Count - 1) * 4);
            headings = new List<float>(path.Capacity);

            SearchNode root = pool[chain[0]];
            Vector3 pos = root.Pos;
            float heading = root.HeadingDeg;
            path.Add(pos);
            headings.Add(heading);

            for (int ci = 1; ci < chain.Count; ci++)
            {
                SearchNode edge = pool[chain[ci]];
                float curvature = edge.Curvature;
                float feasibleV = Math.Abs(curvature) < 0.002f
                    ? searchSpeed
                    : (float)Math.Sqrt(aLat / Math.Abs(curvature));
                float primitiveSpeed = RaceMath.Clamp(Math.Min(searchSpeed, feasibleV), 4f, searchSpeed);
                int count = Math.Max(1, (int)Math.Round(PrimitiveM / SampleM));

                for (int s = 0; s < count; s++)
                {
                    float ds = PrimitiveM / count;
                    float dHead = curvature * ds * 180f / (float)Math.PI;
                    float midHeading = WrapHeading(heading + dHead * 0.5f);
                    Vector3 fwd = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(midHeading));
                    Vector3 candidate = new Vector3(
                        pos.X + fwd.X * ds,
                        pos.Y + fwd.Y * ds,
                        pos.Z);
                    heading = WrapHeading(heading + dHead);

                    Vector3 projected;
                    float flow;
                    float conf;
                    bool opposing;
                    if (world.TryProjectToSurface(
                        candidate, heading, pos.Z,
                        out projected, out flow, out conf, out opposing))
                        pos = projected;
                    else
                        pos = candidate;

                    path.Add(pos);
                    headings.Add(heading);
                }
            }
        }

        private int ComparePoolCost(int a, int b)
        {
            return pool[a].Cost.CompareTo(pool[b].Cost);
        }

        private void BuildDiverseBeam(List<int> sorted, int max)
        {
            nextBeam.Clear();
            diversity.Clear();

            for (int i = 0; i < sorted.Count && nextBeam.Count < max; i++)
            {
                SearchNode n = pool[sorted[i]];
                int qx = (int)Math.Round(n.Pos.X / 3.5f);
                int qy = (int)Math.Round(n.Pos.Y / 3.5f);
                int qh = (int)Math.Round(WrapHeading(n.HeadingDeg) / 12f);
                long key = ((long)(qx & 0x1FFFFF) << 33)
                    ^ ((long)(qy & 0x1FFFFF) << 12)
                    ^ (long)(qh & 0xFFF);
                if (!diversity.Add(key)) continue;
                nextBeam.Add(sorted[i]);
            }

            for (int i = 0; i < sorted.Count && nextBeam.Count < max; i++)
            {
                int idx = sorted[i];
                bool exists = false;
                for (int j = 0; j < nextBeam.Count; j++)
                    if (nextBeam[j] == idx) { exists = true; break; }
                if (!exists) nextBeam.Add(idx);
            }
        }

        private static string Summarize(IList<TrajectoryCandidate> candidates)
        {
            try
            {
                var parts = new List<string>();
                int n = Math.Min(5, candidates.Count);
                for (int i = 0; i < n; i++)
                {
                    var c = candidates[i];
                    string b = c.ConstrainHandle != -1 ? "T" + c.ConstrainHandle : "-";
                    parts.Add($"{i}:{c.Shape}/S{c.Score:F0}/V{c.MeanSpeed:F1}/M{c.MinSpeed:F1}/"
                        + $"L{c.LateralM:F1}/{b}@{c.ConstrainS:F0}/C{c.MinPredClearance:F1}/"
                        + $"O{c.OpposingFraction * 100f:F0}/U{c.UnknownFraction * 100f:F0}/"
                        + $"Z{c.ElevationDeltaM:F1}");
                }
                return string.Join("|", parts);
            }
            catch { return "?"; }
        }

        private static List<float> BuildStationS(IList<Vector3> path)
        {
            var ss = new List<float>(path.Count);
            ss.Add(0f);
            float acc = 0f;
            for (int i = 1; i < path.Count; i++)
            {
                acc += RaceMath.FlatDistance(path[i - 1], path[i]);
                ss.Add(acc);
            }
            return ss;
        }

        private static float[] BackwardPass(float[] allow, IList<float> ss, float aBrake)
        {
            var r = new float[allow.Length];
            int n = allow.Length;
            if (n == 0) return r;
            r[n - 1] = allow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(0.5f, ss[i + 1] - ss[i]);
                float reach = (float)Math.Sqrt(
                    Math.Max(0f, r[i + 1] * r[i + 1] + 2f * aBrake * ds));
                r[i] = Math.Min(allow[i], reach);
            }
            return r;
        }

        private static List<float> BuildArrivalTimes(IList<float> ss, IList<float> speed)
        {
            var r = new List<float>(ss.Count);
            if (ss.Count == 0) return r;
            r.Add(0f);
            for (int i = 1; i < ss.Count; i++)
            {
                float ds = Math.Max(0.2f, ss[i] - ss[i - 1]);
                float v = Math.Max(1.2f, (speed[i - 1] + speed[i]) * 0.5f);
                r.Add(r[i - 1] + ds / v);
            }
            return r;
        }

        private static float CurvatureAt(IList<Vector3> path, int i)
        {
            if (path == null || path.Count < 3) return 0f;
            int a = Math.Max(0, i - 1);
            int b = i;
            int d = Math.Min(path.Count - 1, i + 1);
            if (a == b) d = Math.Min(path.Count - 1, b + 2);
            if (b == d) a = Math.Max(0, b - 2);

            Vector3 v0 = new Vector3(
                path[b].X - path[a].X,
                path[b].Y - path[a].Y, 0f);
            Vector3 v1 = new Vector3(
                path[d].X - path[b].X,
                path[d].Y - path[b].Y, 0f);
            float l0 = RaceMath.FlatLength(v0);
            float l1 = RaceMath.FlatLength(v1);
            if (l0 < 0.4f || l1 < 0.4f) return 0f;
            float dh = Math.Abs(RaceMath.SignedAngleDeg(v0, v1))
                * (float)Math.PI / 180f;
            return dh / Math.Max(0.5f, (l0 + l1) * 0.5f);
        }

        private static float HeadingAt(IList<Vector3> path, int i)
        {
            if (path.Count < 2) return 0f;
            int a = Math.Max(0, i - 1);
            int b = Math.Min(path.Count - 1, i + 1);
            Vector3 d = new Vector3(
                path[b].X - path[a].X,
                path[b].Y - path[a].Y, 0f);
            return RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
        }

        private static float Mean(IList<float> xs)
        {
            if (xs == null || xs.Count == 0) return 0f;
            float s = 0f;
            for (int i = 0; i < xs.Count; i++) s += xs[i];
            return s / xs.Count;
        }

        private static float Min(IList<float> xs, float fallback)
        {
            if (xs == null || xs.Count == 0) return fallback;
            float m = float.MaxValue;
            for (int i = 0; i < xs.Count; i++)
                if (xs[i] < m) m = xs[i];
            return m == float.MaxValue ? fallback : m;
        }

        private static float ParseFirstCurvature(string shape)
        {
            try
            {
                if (string.IsNullOrEmpty(shape)) return 0f;
                int colon = shape.IndexOf(':');
                if (colon < 0 || colon + 1 >= shape.Length) return 0f;
                float v;
                if (float.TryParse(
                    shape.Substring(colon + 1),
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture,
                    out v))
                    return v;
            }
            catch { }
            return 0f;
        }

        private static float WrapHeading(float h)
        {
            while (h >= 360f) h -= 360f;
            while (h < 0f) h += 360f;
            return h;
        }
    }
}
