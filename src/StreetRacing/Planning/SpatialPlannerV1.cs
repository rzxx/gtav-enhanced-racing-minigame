using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// First planner whose search space is world XY rather than lateral
    /// deformation of the GPS/reference ribbon.
    ///
    /// Search state: (world position, heading, time).
    /// Expansion: short constant-curvature motion primitives.
    /// Cost: local-world surface + dynamic actor occupancy + control effort,
    ///       with GTA route used only as a GLOBAL progress/goal objective.
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

        private sealed class Node
        {
            public Vector3 Pos;
            public float HeadingDeg;
            public float TimeS;
            public float Cost;
            public float GoalDist;
            public float FirstCurvature;
            public float LastCurvature;
            public readonly List<Vector3> Path = new List<Vector3>();
            public readonly List<float> Heading = new List<float>();
        }

        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public TrajectoryCandidate LastChosen;
        public bool HasChosen;
        public string LastDecision { get; private set; } = "";

        private float lastFirstCurvature;
        private bool hasLastCurvature;
        private int planId;

        private const float PrimitiveM = 8f;
        private const float SampleM = 2f;
        private const int Layers = 7;
        private const int BeamWidth = 32;

        public void Reset()
        {
            LastCandidates.Clear();
            LastChosen = new TrajectoryCandidate();
            HasChosen = false;
            LastDecision = "";
            lastFirstCurvature = 0f;
            hasLastCurvature = false;
            planId = 0;
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
            var root = new Node
            {
                Pos = egoPos,
                HeadingDeg = egoHeading,
                TimeS = 0f,
                Cost = 0f,
                GoalDist = RaceMath.FlatDistance(egoPos, spatialGoal),
                FirstCurvature = 0f,
                LastCurvature = 0f,
            };
            root.Path.Add(egoPos);
            root.Heading.Add(egoHeading);

            var beam = new List<Node> { root };
            for (int layer = 0; layer < Layers; layer++)
            {
                var expanded = new List<Node>(BeamWidth * 7);
                for (int i = 0; i < beam.Count; i++)
                {
                    var parent = beam[i];
                    var curvatures = CurvatureChoices(parent.LastCurvature, layer);
                    for (int k = 0; k < curvatures.Count; k++)
                    {
                        var child = Expand(parent, curvatures[k], searchSpeed, world, spatialGoal, aLat, layer);
                        if (child != null) expanded.Add(child);
                    }
                }

                if (expanded.Count == 0) break;
                expanded.Sort((a, b) => a.Cost.CompareTo(b.Cost));
                beam = DiverseBeam(expanded, BeamWidth);
            }

            if (beam.Count == 0)
            {
                result.Detail = "search-empty";
                return result;
            }

            beam.Sort((a, b) => a.Cost.CompareTo(b.Cost));
            int emit = Math.Min(10, beam.Count);
            for (int i = 0; i < emit; i++)
            {
                var c = FinalizeCandidate(beam[i], world, route, aLat, aBrake, cruise, egoSpeed, i);
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

            Vector3 ef = RaceMath.VectorFromHeading(egoHeading);
            ef = RaceMath.FlatNormalize(ef);
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
            result.Detail = $"intent={intent};score={chosen.Score:F1};meanV={chosen.MeanSpeed:F1};"
                + $"minV={chosen.MinSpeed:F1};clear={chosen.MinPredClearance:F1};"
                + $"constr={(chosen.ConstrainHandle != -1 ? chosen.ConstrainKind + "#" + chosen.ConstrainHandle : "none")};"
                + $"{world.Detail};beam={beam.Count};cand={LastCandidates.Count}";

            LastChosen = chosen;
            HasChosen = true;
            LastDecision = result.Detail;
            return result;
        }

        private Node Expand(
            Node parent,
            float curvature,
            float searchSpeed,
            LocalWorldModel world,
            Vector3 spatialGoal,
            float aLat,
            int layer)
        {
            var n = new Node
            {
                Pos = parent.Pos,
                HeadingDeg = parent.HeadingDeg,
                TimeS = parent.TimeS,
                Cost = parent.Cost,
                GoalDist = parent.GoalDist,
                FirstCurvature = layer == 0 ? curvature : parent.FirstCurvature,
                LastCurvature = curvature,
            };
            n.Path.AddRange(parent.Path);
            n.Heading.AddRange(parent.Heading);

            if (layer == 0 && hasLastCurvature)
                n.Cost += Math.Abs(curvature - lastFirstCurvature) * 55f;
            n.Cost += Math.Abs(curvature - parent.LastCurvature) * 18f;
            n.Cost += Math.Abs(curvature) * 5f;

            float feasibleV = Math.Abs(curvature) < 0.002f
                ? searchSpeed
                : (float)Math.Sqrt(aLat / Math.Abs(curvature));
            float primitiveSpeed = RaceMath.Clamp(Math.Min(searchSpeed, feasibleV), 4f, searchSpeed);
            float steps = PrimitiveM / SampleM;
            int count = Math.Max(1, (int)Math.Round(steps));

            for (int s = 0; s < count; s++)
            {
                float ds = PrimitiveM / count;
                float dHead = curvature * ds * 180f / (float)Math.PI;
                float midHeading = WrapHeading(n.HeadingDeg + dHead * 0.5f);
                Vector3 fwd = RaceMath.VectorFromHeading(midHeading);
                fwd = RaceMath.FlatNormalize(fwd);
                n.Pos = new Vector3(
                    n.Pos.X + fwd.X * ds,
                    n.Pos.Y + fwd.Y * ds,
                    n.Pos.Z);
                n.HeadingDeg = WrapHeading(n.HeadingDeg + dHead);
                n.TimeS += ds / Math.Max(primitiveSpeed, 1f);

                var pc = world.EvaluatePose(n.Pos, n.HeadingDeg, n.TimeS);
                n.Cost += pc.SurfaceCost * ds * 0.42f;
                n.Cost += pc.ActorCost * 0.12f;
                if (pc.HardCollision)
                    n.Cost += 90f; // expensive, but following must remain representable

                n.Path.Add(n.Pos);
                n.Heading.Add(n.HeadingDeg);
            }

            float newGoalDist = RaceMath.FlatDistance(n.Pos, spatialGoal);
            float goalProgress = parent.GoalDist - newGoalDist;
            n.GoalDist = newGoalDist;

            // Route is now only a global branch/goal cue. The search is free
            // to approach that goal through any low-cost local world space.
            n.Cost -= goalProgress * 3.4f;
            if (goalProgress < -1f) n.Cost += Math.Abs(goalProgress) * 9f;

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

        private static List<float> CurvatureChoices(float previous, int layer)
        {
            // Geometric action set. Speed feasibility is handled separately;
            // tight arcs are allowed if the resulting speed profile slows.
            float[] baseK = { -0.075f, -0.045f, -0.022f, 0f, 0.022f, 0.045f, 0.075f };
            var r = new List<float>(baseK.Length);
            for (int i = 0; i < baseK.Length; i++)
            {
                float k = baseK[i];
                // First layer avoids instant opposite-lock from previous plan.
                if (layer == 0 && Math.Abs(k - previous) > 0.10f) continue;
                r.Add(k);
            }
            return r;
        }

        private static List<Node> DiverseBeam(List<Node> sorted, int max)
        {
            var r = new List<Node>();
            var used = new HashSet<string>();
            for (int i = 0; i < sorted.Count && r.Count < max; i++)
            {
                var n = sorted[i];
                int qx = (int)Math.Round(n.Pos.X / 3.5f);
                int qy = (int)Math.Round(n.Pos.Y / 3.5f);
                int qh = (int)Math.Round(WrapHeading(n.HeadingDeg) / 12f);
                string key = qx + ":" + qy + ":" + qh;
                if (!used.Add(key)) continue;
                r.Add(n);
            }
            // If clustering was too aggressive, fill with best leftovers.
            for (int i = 0; i < sorted.Count && r.Count < max; i++)
                if (!r.Contains(sorted[i])) r.Add(sorted[i]);
            return r;
        }

        private TrajectoryCandidate FinalizeCandidate(
            Node node,
            LocalWorldModel world,
            RaceRoute route,
            float aLat,
            float aBrake,
            float cruise,
            float egoSpeed,
            int index)
        {
            var c = new TrajectoryCandidate
            {
                CandidateIndex = index,
                Shape = $"Spatial:{node.FirstCurvature:+0.000;-0.000;0.000}",
                Path = new List<Vector3>(node.Path),
                StationS = BuildStationS(node.Path),
                AimPoint = node.Path[node.Path.Count - 1],
                RejectReason = "",
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
                MinRoadConfidence = 1f,
                Score = -node.Cost,
                FirstTangentErrDeg = 0f,
                RouteHeadErrDeg = route.HeadingErrorDeg,
            };

            int n = c.Path.Count;
            var headings = node.Heading;
            var kappa = new float[n];
            var roadAllow = new float[n];
            float maxK = 0f;
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureAt(c.Path, i);
                kappa[i] = k;
                if (k > maxK) maxK = k;
                float v = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                roadAllow[i] = RaceMath.Clamp(v, 0f, cruise);
            }
            c.MaxKappa = maxK;

            var roadProfile = BackwardPass(roadAllow, c.StationS, aBrake);
            c.RoadTargetSpeed = roadProfile[0];

            // Predict actor occupancy using the curvature-feasible timing, then
            // turn actual overlap into a stop point. This is where real vehicle
            // length/width matters: rear-ending is visible metres earlier.
            var arrival = BuildArrivalTimes(c.StationS, roadProfile);
            var allow = new float[n];
            Array.Copy(roadAllow, allow, n);
            float earliestStopS = float.MaxValue;
            int blocker = -1;
            string blockerKind = "";
            float minClear = 999f;

            for (int i = 0; i < n; i++)
            {
                float h = i < headings.Count ? headings[i] : HeadingAt(c.Path, i);
                var pc = world.EvaluatePose(c.Path[i], h, arrival[i]);
                if (pc.RoadConfidence < c.MinRoadConfidence) c.MinRoadConfidence = pc.RoadConfidence;
                if (pc.ClearanceM < minClear) minClear = pc.ClearanceM;

                if (pc.BlockingHandle != -1 && pc.ClearanceM < 2.8f)
                {
                    TrackedActor a;
                    float actorV = 0f;
                    string kind = "Actor";
                    if (world.TryGetActor(pc.BlockingHandle, out a))
                    {
                        actorV = a.Speed;
                        kind = a.Kind.ToString();
                    }

                    if (pc.ClearanceM <= 0.35f)
                    {
                        float stopS = Math.Max(0f, c.StationS[i] - 3.0f);
                        if (stopS < earliestStopS)
                        {
                            earliestStopS = stopS;
                            blocker = pc.BlockingHandle;
                            blockerKind = kind;
                        }
                    }
                    else
                    {
                        allow[i] = Math.Min(allow[i], Math.Max(0f, actorV));
                        if (blocker == -1)
                        {
                            blocker = pc.BlockingHandle;
                            blockerKind = kind;
                            c.ConstrainS = c.StationS[i];
                        }
                    }
                }
            }

            if (earliestStopS < float.MaxValue)
            {
                for (int i = 0; i < n; i++)
                    if (c.StationS[i] >= earliestStopS) allow[i] = 0f;
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
            c.MeanSpeed = Mean(desired);
            c.MinSpeed = Min(desired, cruise);
            c.RequiredDecel = Math.Max(0f, egoSpeed - c.TargetSpeed);
            c.LookaheadM = c.StationS[c.StationS.Count - 1];
            c.LateralM = route.ProjectOntoRoute(c.AimPoint).Lateral;
            c.SpeedLimiting = blocker != -1 && c.TargetSpeed < c.RoadTargetSpeed - 0.25f
                ? $"Traffic:{blockerKind}#{blocker}"
                : c.RoadTargetSpeed < cruise - 0.5f ? "Curvature" : "Cruise";

            // Re-rank complete maneuvers with executable speed. A path through
            // traffic can survive search as a follow option, but a clear route
            // that preserves speed should win decisively.
            c.Score += c.MeanSpeed * 3.2f + c.MinSpeed * 0.8f
                + RaceMath.Clamp(minClear, -2f, 6f) * 1.2f;
            return c;
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
                float reach = (float)Math.Sqrt(Math.Max(0f, r[i + 1] * r[i + 1] + 2f * aBrake * ds));
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
            Vector3 v0 = new Vector3(path[b].X - path[a].X, path[b].Y - path[a].Y, 0f);
            Vector3 v1 = new Vector3(path[d].X - path[b].X, path[d].Y - path[b].Y, 0f);
            float l0 = RaceMath.FlatLength(v0);
            float l1 = RaceMath.FlatLength(v1);
            if (l0 < 0.4f || l1 < 0.4f) return 0f;
            float dh = Math.Abs(RaceMath.SignedAngleDeg(v0, v1)) * (float)Math.PI / 180f;
            return dh / Math.Max(0.5f, (l0 + l1) * 0.5f);
        }

        private static float HeadingAt(IList<Vector3> path, int i)
        {
            if (path.Count < 2) return 0f;
            int a = Math.Max(0, i - 1);
            int b = Math.Min(path.Count - 1, i + 1);
            Vector3 d = new Vector3(path[b].X - path[a].X, path[b].Y - path[a].Y, 0f);
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
            for (int i = 0; i < xs.Count; i++) if (xs[i] < m) m = xs[i];
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
                if (float.TryParse(shape.Substring(colon + 1),
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out v))
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
