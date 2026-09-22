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

        private struct RouteGate
        {
            public Vector3 Center;
            public Vector3 Dir;
            public float S;
            public float HalfWidthM;
            public float ZToleranceM;
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
            public int GateIndex;
            public int SurfaceComponentId;
            public float GateMissCost;
            public bool GateMissed;
        }

        private static readonly float[] CurvatureSet =
            { -0.075f, -0.045f, -0.022f, 0f, 0.022f, 0.045f, 0.075f };

        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public readonly List<Vector3> DebugGateCenters = new List<Vector3>(8);
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
        private readonly List<RouteGate> routeGates = new List<RouteGate>(8);
        private readonly List<Vector3> committedPath = new List<Vector3>(40);
        private readonly List<float> committedStationS = new List<float>(40);
        private bool continuityActive;
        private float continuityBaseS;
        private float motionRootCurvature;
        private float motionCurvatureStep;
        private float continuityWeight;

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
            DebugGateCenters.Clear();
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
            routeGates.Clear();
            committedPath.Clear();
            committedStationS.Clear();
            continuityActive = false;
            continuityBaseS = 0f;
            motionRootCurvature = 0f;
            motionCurvatureStep = 0.075f;
            continuityWeight = 0.75f;
        }

        public Result Plan(
            LocalWorldModel world,
            PhysicalSurfaceMap physicalSurface,
            RaceRoute route,
            VehicleCapability capability,
            DriverProfile profile,
            Vector3 egoPos,
            float egoHeading,
            float egoSpeed,
            float egoYawRateRadS,
            float egoHalfWidth,
            float cruise,
            int nowMs,
            Vector3 finishTarget)
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

            // Start from the motion the car ACTUALLY has, not an imaginary
            // zero-curvature state. yawRate / forward speed is the instantaneous
            // path curvature implied by the current body motion. At low speed
            // yaw is noisy, so fade toward zero.
            float motionSpeed = Math.Max(egoSpeed, 2.5f);
            float motionKLimit = Math.Min(
                0.09f, (aLat * 1.20f) / Math.Max(motionSpeed * motionSpeed, 9f));
            motionRootCurvature = RaceMath.Clamp(
                egoYawRateRadS / motionSpeed, -motionKLimit, motionKLimit);
            if (egoSpeed < 4f)
                motionRootCurvature *= RaceMath.Clamp((egoSpeed - 1.5f) / 2.5f, 0f, 1f);

            // Curvature slew is the planner equivalent of preserving momentum.
            // A fast car may still choose a very different eventual line, but
            // it must bend into that line over distance instead of teleporting
            // between opposite arcs every reaction tick.
            motionCurvatureStep = egoSpeed >= 22f ? 0.012f
                : egoSpeed >= 16f ? 0.017f
                : egoSpeed >= 10f ? 0.026f
                : egoSpeed >= 6f ? 0.040f
                : 0.070f;
            // Commitment is only a small anti-jitter preference. It must not
            // overpower route topology or uncertain-but-necessary replanning.
            continuityWeight = egoSpeed >= 16f ? 1.00f
                : egoSpeed >= 8f ? 0.80f
                : 0.60f;

            BuildRouteGates(route, finishTarget);
            float goalS = Math.Min(route.TotalLength, route.AlongS + 75f);
            Vector3 spatialGoal = routeGates.Count > 0
                ? routeGates[routeGates.Count - 1].Center
                : route.PointAtS(goalS);
            int rootSurface = world.LocateSurfaceComponent(egoPos, egoHeading);

            continuityActive = false;
            continuityBaseS = 0f;
            if (!route.IsLost && committedPath.Count >= 3
                && committedStationS.Count == committedPath.Count)
            {
                float continuityDist;
                continuityBaseS = ClosestStationOnPath(
                    egoPos, committedPath, committedStationS, out continuityDist);
                continuityActive = continuityDist <= 7.0f
                    && Math.Abs(route.HeadingErrorDeg) <= 20f;

                // A previously chosen path is not an incumbent merely because
                // the car is physically close to it. It must still lead through
                // the next route aperture. Otherwise commitment is exactly the
                // wrong thing: it makes a missed turn self-reinforcing.
                if (continuityActive && routeGates.Count > 0)
                {
                    RouteGate g = routeGates[0];
                    Vector3 probe = PointOnPathAtS(
                        committedPath, committedStationS,
                        continuityBaseS + 18f);
                    Vector3 rel = new Vector3(
                        probe.X - g.Center.X,
                        probe.Y - g.Center.Y, 0f);
                    float gateLat = Math.Abs(RaceMath.FlatCross(g.Dir, rel));
                    float gateAlong = RaceMath.FlatDot(rel, g.Dir);
                    if (gateLat > g.HalfWidthM + 3.5f || gateAlong < -7f)
                        continuityActive = false;
                }
            }

            pool.Clear();
            beam.Clear();
            var root = new SearchNode
            {
                Pos = egoPos,
                HeadingDeg = egoHeading,
                TimeS = 0f,
                Cost = 0f,
                GoalDist = RaceMath.FlatDistance(egoPos, spatialGoal),
                FirstCurvature = motionRootCurvature,
                LastCurvature = motionRootCurvature,
                Curvature = motionRootCurvature,
                ParentIndex = -1,
                GateIndex = 0,
                SurfaceComponentId = rootSurface,
                GateMissCost = 0f,
                GateMissed = false,
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
                        // CurvatureSet expresses where we would eventually like
                        // to go. Clamp each primitive to a speed-dependent delta
                        // from the motion inherited by its parent. This creates
                        // continuous curvature ramps without exploding the beam
                        // with another state dimension.
                        float desiredCurvature = CurvatureSet[ki];
                        float curvature = RaceMath.Clamp(
                            desiredCurvature,
                            parent.LastCurvature - motionCurvatureStep,
                            parent.LastCurvature + motionCurvatureStep);

                        SearchNode child = Expand(
                            parent, parentIndex, curvature, searchSpeed,
                            world, physicalSurface, spatialGoal, aLat, layer);
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
                    leafIndex, world, physicalSurface, route, aLat, aBrake,
                    cruise, egoSpeed, searchSpeed, i);
                LastCandidates.Add(c);
            }

            // Score first, then physically validate only the few choices
            // that could actually win. This keeps collision natives bounded:
            // the broad map handles free-space understanding, while a small
            // exact sweep vetoes walls, trees and guardrails on the final line.
            var order = new List<int>(LastCandidates.Count);
            for (int i = 0; i < LastCandidates.Count; i++)
                if (string.IsNullOrEmpty(LastCandidates[i].RejectReason))
                    order.Add(i);
            order.Sort((a, b) =>
                LastCandidates[b].Score.CompareTo(LastCandidates[a].Score));

            int best = -1;
            int physicalChecks = 0;
            int physicalBlocks = 0;
            string physicalDetail = "none";
            const int maxPhysicalChecks = 2;

            for (int oi = 0; oi < order.Count; oi++)
            {
                int ci = order[oi];
                var c = LastCandidates[ci];

                if (physicalSurface != null)
                {
                    if (physicalChecks >= maxPhysicalChecks)
                        break;

                    var pc = physicalSurface.CheckTrajectory(
                        c.Path, c.StationS, egoHalfWidth);
                    physicalChecks++;

                    if (pc.Available)
                    {
                        physicalDetail = pc.Detail ?? "";
                        if (pc.Blocked)
                        {
                            physicalBlocks++;
                            c.RejectReason =
                                $"physical-static@{pc.BlockedS:F0}";
                            LastCandidates[ci] = c;
                            continue;
                        }
                    }
                }

                best = ci;
                break;
            }

            if (best < 0)
            {
                LastPlanMs = (float)((Stopwatch.GetTimestamp() - perfStart)
                    * 1000.0 / Stopwatch.Frequency);
                result.Detail =
                    $"no-final-candidate;physicalChecks={physicalChecks};"
                    + $"physicalBlocks={physicalBlocks};physical={physicalDetail};"
                    + $"planMs={LastPlanMs:F1}";
                LastDecision = result.Detail;
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
                + $"motionK={motionRootCurvature:F3};kStep={motionCurvatureStep:F3};"
                + $"commitW={continuityWeight:F2};commitActive={(continuityActive ? 1 : 0)};"
                + $"minV={chosen.MinSpeed:F1};clear={chosen.MinPredClearance:F1};"
                + $"constr={(chosen.ConstrainHandle != -1 ? chosen.ConstrainKind + "#" + chosen.ConstrainHandle : "none")};"
                + $"opp={chosen.OpposingFraction:F2};unknown={chosen.UnknownFraction:F2};"
                + $"flow={chosen.MeanFlowCost:F2};dz={chosen.ElevationDeltaM:F1};"
                + $"gates={chosen.RouteGatesPassed}/{routeGates.Count};surf={chosen.SurfaceComponentId};"
                + $"gateMiss={chosen.GateMissCost:F1};"
                + $"physicalChecks={physicalChecks};physicalBlocks={physicalBlocks};physical={physicalDetail};"
                + $"{world.Detail};planMs={LastPlanMs:F1};pool={pool.Count};beam={beam.Count};cand={LastCandidates.Count};"
                + $"top={Summarize(LastCandidates)}";

            committedPath.Clear();
            committedStationS.Clear();
            if (chosen.Path != null)
            {
                for (int i = 0; i < chosen.Path.Count; i++)
                    committedPath.Add(chosen.Path[i]);
                if (chosen.StationS != null && chosen.StationS.Count == chosen.Path.Count)
                {
                    for (int i = 0; i < chosen.StationS.Count; i++)
                        committedStationS.Add(chosen.StationS[i]);
                }
                else
                {
                    var ss = BuildStationS(chosen.Path);
                    for (int i = 0; i < ss.Count; i++)
                        committedStationS.Add(ss[i]);
                }
            }

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
            PhysicalSurfaceMap physicalSurface,
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
                GateIndex = parent.GateIndex,
                SurfaceComponentId = parent.SurfaceComponentId,
                GateMissCost = parent.GateMissCost,
                GateMissed = parent.GateMissed,
            };

            if (layer == 0 && hasLastCurvature)
                n.Cost += Math.Abs(curvature - lastFirstCurvature) * 80f;

            // Penalize curvature acceleration, not steering itself. The clamp
            // above is the hard physical envelope; this term simply prefers the
            // smallest necessary change inside that envelope.
            n.Cost += Math.Abs(curvature - parent.LastCurvature) * 65f;
            n.Cost += Math.Abs(curvature) * 4f;

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
                Vector3 prevPos = n.Pos;
                Vector3 projected;
                float projectedFlow;
                float projectedConfidence;
                bool projectedOpposing;
                int projectedSurface;
                if (world.TryProjectToSurface(
                    candidate, newHeading, n.Pos.Z, n.SurfaceComponentId,
                    out projected, out projectedFlow,
                    out projectedConfidence, out projectedOpposing,
                    out projectedSurface))
                {
                    n.Pos = projected;
                    n.SurfaceComponentId = projectedSurface;
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

                ApplyGateProgress(ref n, prevPos, n.Pos);
                var pc = world.EvaluatePose(
                    n.Pos, n.HeadingDeg, n.TimeS, n.SurfaceComponentId);

                float surfaceCost = pc.SurfaceCost;
                if (physicalSurface != null)
                {
                    PhysicalSurfaceMap.PointQuery pq;
                    if (physicalSurface.TryQueryPoint(n.Pos, out pq))
                    {
                        if (pq.StaticObstacle)
                        {
                            // Sampled static obstacle is positive evidence too:
                            // make this branch extremely unattractive before the
                            // exact final sweep gets a chance to veto it.
                            surfaceCost += 45f;
                            n.Cost += 120f;
                        }
                        else if (pq.Reachable)
                        {
                            // Connected physical ground overrides the legacy
                            // model's "unknown/narrow road support" penalty, but
                            // does NOT erase flow direction or actor costs.
                            surfaceCost = Math.Min(surfaceCost, 0.35f);
                        }
                    }
                }

                n.Cost += surfaceCost * ds * 0.42f;
                n.Cost += pc.FlowCost * ds * 0.95f;
                n.Cost += pc.ActorCost * 0.12f;
                if (pc.HardCollision)
                    n.Cost += 90f;
            }

            float newGoalDist = RaceMath.FlatDistance(n.Pos, spatialGoal);
            float goalProgress = parent.GoalDist - newGoalDist;
            n.GoalDist = newGoalDist;

            // Gates are TOPOLOGICAL APERTURES, not waypoints. Staying anywhere
            // inside the road-width aperture is free; only being outside the
            // required cross-section is penalised. The previous implementation
            // charged Euclidean distance to each gate CENTER, which pulled the
            // car left/right across an otherwise open road every replan.
            int gatesAdvanced = n.GateIndex - parent.GateIndex;
            if (gatesAdvanced > 0)
                n.Cost -= gatesAdvanced * 28f;
            if (n.GateIndex < routeGates.Count)
            {
                RouteGate g = routeGates[n.GateIndex];
                Vector3 rel = new Vector3(
                    n.Pos.X - g.Center.X, n.Pos.Y - g.Center.Y, 0f);
                float lat = Math.Abs(RaceMath.FlatCross(g.Dir, rel));
                float dz = Math.Abs(n.Pos.Z - g.Center.Z);
                float latOutside = Math.Max(0f, lat - g.HalfWidthM);
                float zOutside = Math.Max(0f, dz - g.ZToleranceM);
                n.Cost += latOutside * 2.0f + zOutside * 7.0f;
            }

            // Receding-horizon commitment: in the absence of a real obstacle,
            // prefer extending the path we already chose instead of selecting
            // a geometrically different left/right solution every 120 ms.
            // Hard actor/surface costs are much larger, so this yields
            // immediately when the old plan becomes unsafe.
            if (continuityActive)
            {
                float expectedS = continuityBaseS + (layer + 1) * PrimitiveM;
                Vector3 committed = PointOnPathAtS(
                    committedPath, committedStationS, expectedS);
                float dCommit = RaceMath.FlatDistance(n.Pos, committed);
                // The near future is a real commitment at racing speed; the
                // far future is allowed to fan out. Actor collision costs are
                // orders of magnitude larger, so a newly unsafe committed path
                // still yields immediately.
                float freeBand = layer <= 1 ? 0.75f : 1.25f;
                float layerFade = layer <= 1 ? 1f : (layer <= 3 ? 0.72f : 0.40f);
                float excess = Math.Max(0f, dCommit - freeBand);
                n.Cost += excess * excess * continuityWeight * layerFade;
            }

            n.Cost -= goalProgress * 1.35f;
            if (goalProgress < -1f)
                n.Cost += Math.Abs(goalProgress) * 5f;

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
            PhysicalSurfaceMap physicalSurface,
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
            int requiredGateProgress = Math.Min(2, routeGates.Count);
            string routeReject = "";
            if (leaf.GateMissed)
                routeReject = "missed-route-gate";
            else if (leaf.GateIndex < requiredGateProgress)
                routeReject = "insufficient-route-gate-progress";

            var c = new TrajectoryCandidate
            {
                CandidateIndex = index,
                Shape = $"Spatial:{leaf.FirstCurvature:+0.000;-0.000;0.000}",
                Path = path,
                StationS = BuildStationS(path),
                AimPoint = path[path.Count - 1],
                RejectReason = routeReject,
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
                MinRoadConfidence = 1f,
                Score = -leaf.Cost,
                FirstTangentErrDeg = 0f,
                RouteHeadErrDeg = route.HeadingErrorDeg,
                RouteGatesPassed = leaf.GateIndex,
                SurfaceComponentId = leaf.SurfaceComponentId,
                GateMissCost = leaf.GateMissCost,
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
                var pc = world.EvaluatePose(
                    c.Path[i], h, arrival[i], c.SurfaceComponentId);
                if (pc.RoadConfidence < c.MinRoadConfidence)
                    c.MinRoadConfidence = pc.RoadConfidence;
                if (pc.ClearanceM < minClear)
                    minClear = pc.ClearanceM;
                if (pc.OpposingSide) opposingSamples++;

                bool physicallyKnownFree = false;
                if (physicalSurface != null)
                {
                    PhysicalSurfaceMap.PointQuery pq;
                    if (physicalSurface.TryQueryPoint(c.Path[i], out pq))
                    {
                        if (pq.StaticObstacle && c.StationS[i] >= 1.5f
                            && string.IsNullOrEmpty(c.RejectReason))
                        {
                            c.RejectReason = "physical-map-static";
                        }
                        physicallyKnownFree = pq.Reachable
                            && !pq.StaticObstacle;
                    }
                }

                // Unknown means neither the legacy road supports NOR the
                // connected physical map can explain this sample.
                if (!pc.OnRoad && !physicallyKnownFree)
                    unknownSamples++;
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
            int surfaceComponent = root.SurfaceComponentId;
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
                    int projectedSurface;
                    if (world.TryProjectToSurface(
                        candidate, heading, pos.Z, surfaceComponent,
                        out projected, out flow, out conf, out opposing,
                        out projectedSurface))
                    {
                        pos = projected;
                        surfaceComponent = projectedSurface;
                    }
                    else
                    {
                        pos = candidate;
                    }

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
                long key = ((long)(qx & 0xFFFFF) << 40)
                    ^ ((long)(qy & 0xFFFFF) << 20)
                    ^ ((long)(qh & 0x3FF) << 10)
                    ^ ((long)(n.GateIndex & 0x1F) << 5)
                    ^ (long)(n.SurfaceComponentId & 0x1F);
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
                        + $"Z{c.ElevationDeltaM:F1}/G{c.RouteGatesPassed}/S{c.SurfaceComponentId}");
                }
                return string.Join("|", parts);
            }
            catch { return "?"; }
        }

        private void BuildRouteGates(RaceRoute route, Vector3 finishTarget)
        {
            routeGates.Clear();
            DebugGateCenters.Clear();
            if (route == null || !route.Built) return;

            float remaining = Math.Max(0f, route.TotalLength - route.AlongS);
            float[] offsets = { 12f, 25f, 40f, 57f, 75f };
            float lastS = -999f;
            for (int i = 0; i < offsets.Length; i++)
            {
                float s = Math.Min(route.TotalLength, route.AlongS + offsets[i]);
                if (s <= route.AlongS + 5f) continue;
                if (s - lastS < 6f) continue;

                Vector3 center = route.PointAtS(s);
                float h = route.HeadingAtS(s);
                Vector3 dir = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(h));
                routeGates.Add(new RouteGate
                {
                    Center = center,
                    Dir = dir,
                    S = s,
                    HalfWidthM = 11.5f,
                    ZToleranceM = 4.0f,
                });
                DebugGateCenters.Add(center);
                lastS = s;
                if (s >= route.TotalLength - 0.5f) break;
            }

            // GPS extraction may stop at the routable street endpoint while
            // the actual race checkpoint is still tens of metres away. Near
            // route end, make the real checkpoint the final ordered gate
            // instead of letting DrivingReference exhaustion become
            // RoadUncertain.
            if (remaining <= 70f)
            {
                Vector3 routeEnd = route.PointAtS(route.TotalLength);
                float finishGapFromRoute = RaceMath.FlatDistance(routeEnd, finishTarget);
                float finishGapFromEgoRoute = RaceMath.FlatDistance(
                    route.PointAtS(route.AlongS), finishTarget);
                if (finishGapFromRoute > 3f && finishGapFromRoute < 80f
                    && finishGapFromEgoRoute < 130f)
                {
                    Vector3 from = routeGates.Count > 0
                        ? routeGates[routeGates.Count - 1].Center
                        : routeEnd;
                    Vector3 d = new Vector3(
                        finishTarget.X - from.X,
                        finishTarget.Y - from.Y, 0f);
                    if (RaceMath.FlatLength(d) < 1f)
                        d = RaceMath.VectorFromHeading(route.HeadingAtS(route.TotalLength));
                    d = RaceMath.FlatNormalize(d);
                    routeGates.Add(new RouteGate
                    {
                        Center = finishTarget,
                        Dir = d,
                        S = route.TotalLength + finishGapFromRoute,
                        HalfWidthM = 13.5f,
                        ZToleranceM = 6.0f,
                    });
                    DebugGateCenters.Add(finishTarget);
                }
            }
        }

        private void ApplyGateProgress(
            ref SearchNode node, Vector3 previousPos, Vector3 currentPos)
        {
            while (node.GateIndex < routeGates.Count)
            {
                RouteGate g = routeGates[node.GateIndex];
                Vector3 relPrev = new Vector3(
                    previousPos.X - g.Center.X,
                    previousPos.Y - g.Center.Y, 0f);
                Vector3 relNow = new Vector3(
                    currentPos.X - g.Center.X,
                    currentPos.Y - g.Center.Y, 0f);
                float prevAlong = RaceMath.FlatDot(relPrev, g.Dir);
                float nowAlong = RaceMath.FlatDot(relNow, g.Dir);
                float lat = Math.Abs(RaceMath.FlatCross(g.Dir, relNow));
                float dz = Math.Abs(currentPos.Z - g.Center.Z);

                bool crossed = prevAlong <= 0.5f && nowAlong >= -0.5f
                    && lat <= g.HalfWidthM
                    && dz <= g.ZToleranceM;

                if (crossed)
                {
                    node.GateIndex++;
                    node.GateMissed = false;
                    continue;
                }

                // Crossing a required route cross-section outside its 3D
                // aperture is a TOPOLOGY failure. Charge it once and mark the
                // leaf invalid; the old code re-added the miss every 2 m,
                // producing gateMiss >1000 and violent beam switching.
                if (nowAlong > 3f && !node.GateMissed)
                {
                    float miss = 90f
                        + Math.Max(0f, lat - g.HalfWidthM) * 4f
                        + Math.Max(0f, dz - g.ZToleranceM) * 12f;
                    node.Cost += miss;
                    node.GateMissCost += miss;
                    node.GateMissed = true;
                }
                break;
            }
        }

        private static float ClosestStationOnPath(
            Vector3 p, IList<Vector3> path, IList<float> stationS, out float distance)
        {
            distance = 999f;
            float bestS = 0f;
            if (path == null || stationS == null || path.Count < 2
                || stationS.Count != path.Count) return bestS;

            for (int i = 0; i < path.Count - 1; i++)
            {
                var pr = RaceMath.ProjectOnSegment(p, path[i], path[i + 1]);
                if (pr.Dist >= distance) continue;
                distance = pr.Dist;
                bestS = stationS[i] + pr.Along;
            }
            return bestS;
        }

        private static Vector3 PointOnPathAtS(
            IList<Vector3> path, IList<float> stationS, float s)
        {
            if (path == null || path.Count == 0) return Vector3.Zero;
            if (stationS == null || stationS.Count != path.Count) return path[path.Count - 1];
            if (s <= stationS[0]) return path[0];
            if (s >= stationS[stationS.Count - 1]) return path[path.Count - 1];
            for (int i = 0; i < stationS.Count - 1; i++)
            {
                if (s < stationS[i] || s > stationS[i + 1]) continue;
                float ds = stationS[i + 1] - stationS[i];
                float t = ds > 1e-4f ? (s - stationS[i]) / ds : 0f;
                Vector3 a = path[i];
                Vector3 b = path[i + 1];
                return new Vector3(
                    a.X + (b.X - a.X) * t,
                    a.Y + (b.Y - a.Y) * t,
                    a.Z + (b.Z - a.Z) * t);
            }
            return path[path.Count - 1];
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
