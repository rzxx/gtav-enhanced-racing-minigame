using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// Local Planner V1 for the Simple driver.
    ///
    /// Global GPS says WHERE to go. DrivingReference supplies a smooth road
    /// spine. This planner decides WHERE ON THE ROAD to drive for the next
    /// few seconds.
    ///
    /// V1 intentionally has no semantic lane graph yet. It samples smooth
    /// lateral alternatives inside RoadCorridor, evaluates each candidate in
    /// WORLD SPACE against predicted actors, and commits to a useful side long
    /// enough to make a pass instead of twitching between candidates.
    internal sealed class LocalPlannerV1
    {
        internal sealed class Result
        {
            public bool Valid;
            public TrajectoryCandidate Chosen;
            public readonly List<TrajectoryCandidate> Candidates = new List<TrajectoryCandidate>();
            public string Intent = "Track";
            public string Detail = "";
            public float RoadDesired;
            public float Desired;
            public float TargetLateral;
            public bool Committed;
        }

        private float committedLat;
        private int commitUntilMs;
        private int commitStartedMs;
        private string committedIntent = "Track";
        private int planId;

        private const float VehicleHalfWidthM = 1.15f;
        private const float RoadMarginM = 0.55f;
        private const float MinCandidateSpacingM = 1.15f;

        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public TrajectoryCandidate LastChosen;
        public bool HasChosen;
        public string Intent { get; private set; } = "Track";
        public string LastDecision { get; private set; } = "";

        public void Reset()
        {
            committedLat = 0f;
            commitUntilMs = 0;
            commitStartedMs = 0;
            committedIntent = "Track";
            planId = 0;
            LastCandidates.Clear();
            LastChosen = new TrajectoryCandidate();
            HasChosen = false;
            Intent = "Track";
            LastDecision = "";
        }

        public Result Plan(
            DrivingReference.Result reference,
            RoadCorridor corridor,
            RaceRoute route,
            Perception perception,
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

            if (reference == null || !reference.Valid || reference.Path == null
                || reference.Path.Count < 4 || reference.StationS == null
                || reference.StationS.Count != reference.Path.Count)
            {
                result.Detail = "reference-invalid";
                return result;
            }

            float horizon = reference.StationS[reference.StationS.Count - 1];
            if (horizon < 18f)
            {
                result.Detail = "reference-too-short";
                return result;
            }

            float usable = corridor != null
                ? corridor.MinHalfWidthAhead(Math.Min(horizon, 70f)) - VehicleHalfWidthM - RoadMarginM
                : 3.5f;
            usable = RaceMath.Clamp(usable, 0.75f, 6.0f);

            var targetLats = BuildTargets(usable);
            float roadDesiredCenter = cruise;
            int candidateIndex = 0;
            foreach (float targetLat in targetLats)
            {
                var c = BuildCandidate(reference, corridor, route, perception, capability,
                    profile, egoPos, egoHeading, egoSpeed, cruise, targetLat, candidateIndex++);
                LastCandidates.Add(c);
                if (Math.Abs(targetLat) < 0.25f && string.IsNullOrEmpty(c.RejectReason))
                    roadDesiredCenter = c.RoadTargetSpeed;
            }

            // RequiredDecel is used only internally in V1 before selection to
            // carry roadDesired. Restore the selected candidate field later.
            int centerIndex = FindClosestIndex(LastCandidates, 0f);
            TrajectoryCandidate center = centerIndex >= 0 ? LastCandidates[centerIndex] : new TrajectoryCandidate();

            int bestIndex = -1;
            float bestScore = float.MinValue;
            for (int i = 0; i < LastCandidates.Count; i++)
            {
                var c = LastCandidates[i];
                if (!string.IsNullOrEmpty(c.RejectReason)) continue;
                if (c.Score > bestScore)
                {
                    bestScore = c.Score;
                    bestIndex = i;
                }
            }
            if (bestIndex < 0)
            {
                result.Detail = "no-viable-candidate";
                return result;
            }

            int committedIndex = FindClosestIndex(LastCandidates, committedLat);
            bool activeCommit = Math.Abs(committedLat) > 0.35f && nowMs < commitUntilMs;
            if (activeCommit && committedIndex >= 0)
            {
                var cc = LastCandidates[committedIndex];
                if (string.IsNullOrEmpty(cc.RejectReason))
                {
                    // While committed, only abandon for a materially safer/faster
                    // option. Small score jitter must never make us weave.
                    var challenger = LastCandidates[bestIndex];
                    bool committedUnsafe = cc.MinPredClearance < -0.25f || cc.MinSpeed < 1.0f;
                    bool challengerDecisive = challenger.Score > cc.Score + 9f
                        && challenger.MinPredClearance > cc.MinPredClearance + 0.7f;
                    if (!committedUnsafe && !challengerDecisive)
                        bestIndex = committedIndex;
                }
            }

            var chosen = LastCandidates[bestIndex];

            // Starting a side maneuver requires a meaningful speed advantage
            // over center. Otherwise center wins even if a side candidate has a
            // tiny numeric score edge.
            if (Math.Abs(committedLat) <= 0.35f && Math.Abs(chosen.LateralM) > 0.35f
                && centerIndex >= 0 && string.IsNullOrEmpty(center.RejectReason))
            {
                float speedGain = chosen.MeanSpeed - center.MeanSpeed;
                float targetGain = chosen.TargetSpeed - center.TargetSpeed;
                bool worthPassing = speedGain >= 2.0f || targetGain >= 2.5f
                    || (center.MinSpeed < 3f && chosen.MinSpeed > 6f);
                if (!worthPassing)
                {
                    bestIndex = centerIndex;
                    chosen = center;
                }
            }

            // Commit / return state.
            if (Math.Abs(chosen.LateralM) > 0.35f)
            {
                if (Math.Abs(committedLat) <= 0.35f
                    || Math.Abs(chosen.LateralM - committedLat) > 0.75f)
                {
                    committedLat = chosen.LateralM;
                    commitStartedMs = nowMs;
                    commitUntilMs = nowMs + 1800;
                    committedIntent = committedLat > 0f ? "PassLeft" : "PassRight";
                }
                else if (nowMs >= commitUntilMs)
                {
                    // Keep extending while center is still substantially worse.
                    if (centerIndex >= 0 && string.IsNullOrEmpty(center.RejectReason)
                        && chosen.MeanSpeed > center.MeanSpeed + 1.2f)
                        commitUntilMs = nowMs + 700;
                }
            }
            else if (Math.Abs(committedLat) > 0.35f)
            {
                bool minCommitSatisfied = nowMs - commitStartedMs >= 1100;
                if (minCommitSatisfied)
                {
                    committedLat = 0f;
                    committedIntent = "Return";
                    commitUntilMs = nowMs + 700;
                }
            }

            if (Math.Abs(committedLat) <= 0.35f)
            {
                if (committedIntent == "Return" && nowMs < commitUntilMs)
                    Intent = "Return";
                else
                    Intent = chosen.ConstrainHandle != -1 && chosen.TargetSpeed < cruise - 1f ? "Follow" : "Track";
            }
            else
            {
                Intent = committedIntent;
            }

            // Find candidate nearest the newly committed lateral so telemetry and
            // execution agree after a commitment transition.
            if (Math.Abs(committedLat) > 0.35f)
            {
                int ci = FindClosestIndex(LastCandidates, committedLat);
                if (ci >= 0 && string.IsNullOrEmpty(LastCandidates[ci].RejectReason))
                    chosen = LastCandidates[ci];
            }

            float roadDesired = chosen.RoadTargetSpeed;
            chosen.RequiredDecel = Math.Max(0f, egoSpeed - chosen.TargetSpeed);

            result.Valid = true;
            result.Chosen = chosen;
            result.Candidates.AddRange(LastCandidates);
            result.Intent = Intent;
            result.RoadDesired = roadDesired;
            result.Desired = chosen.TargetSpeed;
            result.TargetLateral = chosen.LateralM;
            result.Committed = Math.Abs(committedLat) > 0.35f;
            result.Detail = $"intent={Intent};lat={chosen.LateralM:F1};score={chosen.Score:F1};"
                + $"meanV={chosen.MeanSpeed:F1};minV={chosen.MinSpeed:F1};"
                + $"constr={(chosen.ConstrainHandle != -1 ? chosen.ConstrainKind + "#" + chosen.ConstrainHandle : "none")};"
                + $"clear={chosen.MinPredClearance:F1};usable={usable:F1}";

            LastChosen = chosen;
            HasChosen = true;
            LastDecision = result.Detail;
            return result;
        }

        private static List<float> BuildTargets(float usable)
        {
            var r = new List<float> { 0f };
            float inner = Math.Min(2.25f, usable * 0.48f);
            float outer = Math.Min(4.4f, usable * 0.88f);

            if (inner >= MinCandidateSpacingM)
            {
                r.Add(inner);
                r.Add(-inner);
            }
            if (outer >= inner + MinCandidateSpacingM && outer >= 2.0f)
            {
                r.Add(outer);
                r.Add(-outer);
            }
            return r;
        }

        private TrajectoryCandidate BuildCandidate(
            DrivingReference.Result reference,
            RoadCorridor corridor,
            RaceRoute route,
            Perception perception,
            VehicleCapability capability,
            DriverProfile profile,
            Vector3 egoPos,
            float egoHeading,
            float egoSpeed,
            float cruise,
            float targetLat,
            int index)
        {
            var c = new TrajectoryCandidate
            {
                CandidateIndex = index,
                LateralM = targetLat,
                LookaheadM = reference.StationS[reference.StationS.Count - 1],
                RejectReason = "",
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
                Score = -10000f,
            };

            try
            {
                var path = new List<Vector3>(reference.Path.Count);
                var ss = new List<float>(reference.Path.Count);
                var lats = new List<float>(reference.Path.Count);

                Vector3 baseDir0 = DirectionAt(reference.Path, 0);
                Vector3 left0 = new Vector3(-baseDir0.Y, baseDir0.X, 0f);
                Vector3 rel0 = new Vector3(
                    egoPos.X - reference.Path[0].X,
                    egoPos.Y - reference.Path[0].Y, 0f);
                float startLat = RaceMath.FlatDot(rel0, left0);
                float baseHeading0 = RaceMath.HeadingFromVector(baseDir0);
                float headErr = RaceMath.HeadingDiffDeg(baseHeading0, egoHeading);
                float m0 = PoseConnector.LateralSlopeForHeadErrDeg(headErr);
                m0 = RaceMath.Clamp(m0, -1.35f, 1.35f);

                float totalS = reference.StationS[reference.StationS.Count - 1];
                float transitionS = RaceMath.Clamp(28f + egoSpeed * 1.4f, 30f, 62f);
                transitionS = Math.Min(transitionS, Math.Max(18f, totalS * 0.78f));

                for (int i = 0; i < reference.Path.Count; i++)
                {
                    float baseS = reference.StationS[i];
                    float lat = HermiteLateral(startLat, m0, targetLat, baseS, transitionS);
                    float allowed = corridor != null
                        ? corridor.HalfWidthAt(baseS) - VehicleHalfWidthM - RoadMarginM
                        : 4f;
                    if (allowed < 0.65f) allowed = 0.65f;
                    if (Math.Abs(lat) > allowed + 0.05f)
                    {
                        c.RejectReason = $"road-boundary@{baseS:F0}";
                        return c;
                    }

                    Vector3 dir = DirectionAt(reference.Path, i);
                    Vector3 left = new Vector3(-dir.Y, dir.X, 0f);
                    Vector3 bp = reference.Path[i];
                    var p = new Vector3(bp.X + left.X * lat, bp.Y + left.Y * lat, bp.Z);
                    if (i == 0) p = egoPos;
                    path.Add(p);
                    lats.Add(lat);
                }

                RebuildStationS(path, ss);
                if (ss.Count < 3 || ss[ss.Count - 1] < 15f)
                {
                    c.RejectReason = "short";
                    return c;
                }

                var kappa = new List<float>(path.Count);
                float maxKappa = 0f;
                for (int i = 0; i < path.Count; i++)
                {
                    float k = CurvatureAt(path, i);
                    kappa.Add(k);
                    if (k > maxKappa) maxKappa = k;
                }

                float firstTang = FirstTangentError(path, egoHeading);
                if (firstTang > 35f)
                {
                    c.RejectReason = $"pose-discontinuity:{firstTang:F0}";
                    return c;
                }
                if (maxKappa > 0.20f && egoSpeed > 7f)
                {
                    c.RejectReason = $"kappa:{maxKappa:F3}";
                    return c;
                }

                float aLat = capability != null ? capability.UsableLat(profile.GripFactor) : 7f;
                float aBrake = capability != null ? capability.UsableBrake(profile.GripFactor) : 6f;
                if (aLat < 2f) aLat = 2f;
                if (aBrake < 3f) aBrake = 3f;
                float top = 60f;
                try { if (capability != null) top = capability.TopSpeedEst; } catch { }

                float[] roadAllow = new float[path.Count];
                for (int i = 0; i < path.Count; i++)
                {
                    float vc = kappa[i] < 1e-5f
                        ? cruise
                        : (float)Math.Sqrt(aLat / kappa[i]);
                    if (vc > cruise) vc = cruise;
                    if (top > 5f && vc > top) vc = top;
                    roadAllow[i] = Math.Max(0f, vc);
                }
                float[] roadProfile = BackwardPass(roadAllow, ss, aBrake);
                float roadDesired = roadProfile[0];

                int constrainHandle;
                string constrainKind;
                float constrainS;
                float minClear;
                float[] actorAllow = BuildWorldActorEnvelope(path, ss, perception, egoSpeed,
                    cruise, profile, out constrainHandle, out constrainKind, out constrainS, out minClear);

                float[] allow = new float[path.Count];
                for (int i = 0; i < allow.Length; i++)
                    allow[i] = Math.Min(roadAllow[i], actorAllow[i]);
                float[] desired = BackwardPass(allow, ss, aBrake);

                float mean = 0f;
                float min = cruise;
                for (int i = 0; i < desired.Length; i++)
                {
                    mean += desired[i];
                    if (desired[i] < min) min = desired[i];
                }
                mean /= Math.Max(1, desired.Length);

                float roadMargin = float.MaxValue;
                for (int i = 0; i < lats.Count; i++)
                {
                    float allowed = corridor != null
                        ? corridor.HalfWidthAt(reference.StationS[Math.Min(i, reference.StationS.Count - 1)])
                            - VehicleHalfWidthM
                        : 5f;
                    float m = allowed - Math.Abs(lats[i]);
                    if (m < roadMargin) roadMargin = m;
                }
                if (roadMargin == float.MaxValue) roadMargin = 0f;

                // Progress dominates. Center is mildly preferred, but a clear
                // side path that preserves several m/s easily wins.
                float score = mean * 5.0f
                    + min * 1.5f
                    + RaceMath.Clamp(minClear, -2f, 6f) * 1.2f
                    + RaceMath.Clamp(roadMargin, -2f, 5f) * 0.7f
                    - Math.Abs(targetLat) * 0.75f
                    - maxKappa * 22f
                    - firstTang * 0.08f;

                // Small continuity preference around the current commitment.
                if (Math.Abs(committedLat) > 0.35f)
                    score -= Math.Abs(targetLat - committedLat) * 0.65f;

                string limiting;
                if (constrainHandle != -1 && desired[0] < roadDesired - 0.25f)
                    limiting = $"Traffic:{constrainKind}#{constrainHandle}";
                else if (roadDesired < cruise - 0.5f)
                    limiting = "Curvature";
                else
                    limiting = "Cruise";

                c.Path = path;
                c.StationS = ss;
                c.SpeedProfile = new List<float>(desired);
                c.ArrivalT = BuildArrivalTimes(ss, desired);
                c.AimPoint = path[path.Count - 1];
                c.MinMarginM = roadMargin;
                c.MaxKappa = maxKappa;
                c.FirstTangentErrDeg = firstTang;
                c.RouteHeadErrDeg = route != null ? route.HeadingErrorDeg : 0f;
                c.TargetSpeed = desired[0];
                c.SpeedLimiting = limiting;
                c.ConstrainHandle = constrainHandle;
                c.ConstrainKind = constrainKind;
                c.ConstrainS = constrainS;
                c.MinPredClearance = minClear;
                c.MeanSpeed = mean;
                c.MinSpeed = min;
                c.RoadTargetSpeed = roadDesired;
                c.RequiredDecel = Math.Max(0f, egoSpeed - desired[0]);
                c.Score = score;
                return c;
            }
            catch (Exception ex)
            {
                c.RejectReason = "exc:" + ex.Message;
                return c;
            }
        }

        private static float[] BuildWorldActorEnvelope(
            IList<Vector3> path,
            IList<float> ss,
            Perception perception,
            float egoSpeed,
            float cruise,
            DriverProfile profile,
            out int constrainHandle,
            out string constrainKind,
            out float constrainS,
            out float minClearance)
        {
            int n = path.Count;
            var allow = new float[n];
            for (int i = 0; i < n; i++) allow[i] = cruise;
            constrainHandle = -1;
            constrainKind = "";
            constrainS = -1f;
            minClearance = 999f;
            if (perception == null || perception.Actors.Count == 0) return allow;

            var arrival = new float[n];
            for (int i = 0; i < n; i++)
                arrival[i] = RaceMath.Clamp(ss[i] / Math.Max(egoSpeed, 6f), 0f, 5f);

            float gapStop = (profile != null ? profile.SafetyMarginM : 3f) + Math.Max(0f, egoSpeed) * 0.25f;
            float earliestStopS = float.MaxValue;
            int earliestStopHandle = -1;
            string earliestStopKind = "";

            for (int i = 0; i < n; i++)
            {
                Vector3 dir = DirectionAt(path, i);
                foreach (var a in perception.Actors)
                {
                    if (!a.Valid) continue;
                    if (a.Dist > 130f) continue;

                    Vector3 pred;
                    try { pred = perception.Predict(a, arrival[i]); }
                    catch { pred = a.Position; }

                    // Grade-separated roads and bridges can overlap in XY.
                    if (Math.Abs(pred.Z - path[i].Z) > 4.0f) continue;

                    float d = RaceMath.FlatDistance(path[i], pred);
                    float radius = VehicleHalfWidthM + SpeedPlanner.ActorHalfWidth(a.Kind) + 0.45f;
                    float clearance = d - (VehicleHalfWidthM + SpeedPlanner.ActorHalfWidth(a.Kind));
                    if (clearance < minClearance) minClearance = clearance;
                    if (d >= radius) continue;

                    Vector3 av = new Vector3(a.Velocity.X, a.Velocity.Y, 0f);
                    float along = RaceMath.FlatDot(av, dir);
                    Vector3 left = new Vector3(-dir.Y, dir.X, 0f);
                    float latV = RaceMath.FlatDot(av, left);

                    bool sameFlow = along > 1.5f && Math.Abs(latV) < Math.Max(2.5f, along * 0.65f);
                    float followV = sameFlow ? along : 0f;

                    if (followV < 0.6f)
                    {
                        float stopS = ss[i] - gapStop;
                        if (stopS < earliestStopS)
                        {
                            earliestStopS = stopS;
                            earliestStopHandle = a.Handle;
                            earliestStopKind = a.Kind.ToString();
                        }
                    }
                    else if (followV < allow[i])
                    {
                        allow[i] = followV;
                        if (constrainHandle == -1 || ss[i] < constrainS)
                        {
                            constrainHandle = a.Handle;
                            constrainKind = a.Kind.ToString();
                            constrainS = ss[i];
                        }
                    }
                }
            }

            if (earliestStopS < float.MaxValue)
            {
                for (int i = 0; i < n; i++)
                    if (ss[i] >= earliestStopS) allow[i] = 0f;
                constrainHandle = earliestStopHandle;
                constrainKind = earliestStopKind;
                constrainS = Math.Max(0f, earliestStopS + gapStop);
            }

            return allow;
        }

        private static float[] BackwardPass(float[] allow, IList<float> ss, float aBrake)
        {
            int n = allow.Length;
            var r = new float[n];
            r[n - 1] = allow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(0.5f, ss[i + 1] - ss[i]);
                float reachable = (float)Math.Sqrt(Math.Max(0f,
                    r[i + 1] * r[i + 1] + 2f * aBrake * ds));
                r[i] = Math.Min(allow[i], reachable);
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
                float ds = Math.Max(0.5f, ss[i] - ss[i - 1]);
                float v = Math.Max(1.2f, (speed[i - 1] + speed[i]) * 0.5f);
                r.Add(r[i - 1] + ds / v);
            }
            return r;
        }

        private static float HermiteLateral(float d0, float m0, float d1, float s, float S)
        {
            if (S <= 1f || s >= S) return d1;
            if (s <= 0f) return d0;
            float t = RaceMath.Clamp(s / S, 0f, 1f);
            float t2 = t * t;
            float t3 = t2 * t;
            float h00 = 2f * t3 - 3f * t2 + 1f;
            float h10 = t3 - 2f * t2 + t;
            float h01 = -2f * t3 + 3f * t2;
            return h00 * d0 + h10 * S * m0 + h01 * d1;
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

        private static void RebuildStationS(IList<Vector3> path, List<float> ss)
        {
            ss.Clear();
            ss.Add(0f);
            float acc = 0f;
            for (int i = 1; i < path.Count; i++)
            {
                acc += RaceMath.FlatDistance(path[i - 1], path[i]);
                ss.Add(acc);
            }
        }

        private static float CurvatureAt(IList<Vector3> path, int i)
        {
            if (path == null || path.Count < 3) return 0f;
            int a = Math.Max(0, i - 1);
            int b = i;
            int c = Math.Min(path.Count - 1, i + 1);
            if (a == b) c = Math.Min(path.Count - 1, b + 2);
            if (b == c) a = Math.Max(0, b - 2);
            var d0 = new Vector3(path[b].X - path[a].X, path[b].Y - path[a].Y, 0f);
            var d1 = new Vector3(path[c].X - path[b].X, path[c].Y - path[b].Y, 0f);
            float l0 = RaceMath.FlatLength(d0);
            float l1 = RaceMath.FlatLength(d1);
            if (l0 < 0.3f || l1 < 0.3f) return 0f;
            float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
            return dh / Math.Max((l0 + l1) * 0.5f, 0.5f);
        }

        private static float FirstTangentError(IList<Vector3> path, float egoHeading)
        {
            if (path == null || path.Count < 2) return 180f;
            var d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, 0f);
            if (RaceMath.FlatLength(d) < 0.2f) return 180f;
            float h = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
            return Math.Abs(RaceMath.HeadingDiffDeg(h, egoHeading));
        }

        private static int FindClosestIndex(IList<TrajectoryCandidate> candidates, float lat)
        {
            int best = -1;
            float bd = float.MaxValue;
            for (int i = 0; i < candidates.Count; i++)
            {
                float d = Math.Abs(candidates[i].LateralM - lat);
                if (d < bd)
                {
                    bd = d;
                    best = i;
                }
            }
            return best;
        }
    }
}
