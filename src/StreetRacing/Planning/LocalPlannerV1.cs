using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// Local Planner V2 for the Simple driver.
    ///
    /// Global GPS says WHERE to go. DrivingReference supplies a smooth road
    /// spine. This planner decides WHERE ON THE ROAD to drive for the next
    /// few seconds.
    ///
    /// V2 still has no semantic lane graph, but its action set is no longer
    /// a handful of parallel rails. It samples multi-stage lateral trajectories
    /// in the SAME frame as DrivingReference's measured road envelope:
    /// hold-side passes, pass+return maneuvers, and outside-apex-outside corner
    /// shapes. Actor conflicts remain WORLD-SPACE authoritative.
    internal sealed class LocalPlannerV2
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
        private string committedShape = "";
        private int planId;

        private const float VehicleHalfWidthM = 1.15f;
        private const float RoadMarginM = 0.55f;
        private const float MinCandidateSpacingM = 1.15f;

        private struct ShapeSpec
        {
            public string Name;
            public float K1;
            public float K2;
            public float K3;
            public float CharacteristicLat;
        }

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
            committedShape = "";
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

            float leftUsable;
            float rightUsable;
            ReferenceUsableSides(reference, Math.Min(horizon, 75f), out leftUsable, out rightUsable);

            var shapes = BuildShapes(reference, leftUsable, rightUsable);
            int candidateIndex = 0;
            foreach (var shape in shapes)
            {
                var c = BuildCandidate(reference, route, perception, capability,
                    profile, egoPos, egoHeading, egoSpeed, cruise, shape, candidateIndex++);
                LastCandidates.Add(c);
            }

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

            int committedIndex = !string.IsNullOrEmpty(committedShape)
                ? FindShapeIndex(LastCandidates, committedShape)
                : FindClosestIndex(LastCandidates, committedLat);
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
                else if (centerIndex >= 0 && string.IsNullOrEmpty(center.RejectReason))
                {
                    // Road narrowed / committed corridor disappeared: unwind
                    // through center instead of snapping across to the other side.
                    bestIndex = centerIndex;
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
                float scoreGain = chosen.Score - center.Score;
                bool apexChoice = chosen.Shape != null && chosen.Shape.StartsWith("Apex");
                bool worthPassing = speedGain >= 2.0f || targetGain >= 2.5f
                    || scoreGain >= 7.0f
                    || (apexChoice && scoreGain >= 3.0f)
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
                    committedShape = chosen.Shape ?? "";
                    commitStartedMs = nowMs;
                    commitUntilMs = nowMs + (chosen.Shape != null && chosen.Shape.StartsWith("Apex") ? 1100 : 1800);
                    if (chosen.Shape != null && chosen.Shape.StartsWith("Apex"))
                        committedIntent = "Apex";
                    else
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
                    committedShape = "";
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
                int ci = !string.IsNullOrEmpty(committedShape)
                    ? FindShapeIndex(LastCandidates, committedShape)
                    : FindClosestIndex(LastCandidates, committedLat);
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
                + $"clear={chosen.MinPredClearance:F1};roadLR={leftUsable:F1}/{rightUsable:F1};shape={chosen.Shape};"
                + $"cands={SummarizeCandidates(LastCandidates)}";

            LastChosen = chosen;
            HasChosen = true;
            LastDecision = result.Detail;
            return result;
        }

        private static List<ShapeSpec> BuildShapes(
            DrivingReference.Result reference, float leftUsable, float rightUsable)
        {
            var r = new List<ShapeSpec>();
            r.Add(new ShapeSpec { Name = "Center", K1 = 0f, K2 = 0f, K3 = 0f, CharacteristicLat = 0f });

            float lInner = Math.Min(2.2f, leftUsable * 0.55f);
            float lOuter = Math.Min(4.4f, leftUsable * 0.90f);
            float rInner = Math.Min(2.2f, rightUsable * 0.55f);
            float rOuter = Math.Min(4.4f, rightUsable * 0.90f);

            if (lInner >= MinCandidateSpacingM)
            {
                r.Add(new ShapeSpec { Name = "HoldL", K1 = lInner, K2 = lInner, K3 = lInner, CharacteristicLat = lInner });
                r.Add(new ShapeSpec { Name = "PassReturnL", K1 = lInner, K2 = lInner, K3 = 0f, CharacteristicLat = lInner });
            }
            if (rInner >= MinCandidateSpacingM)
            {
                r.Add(new ShapeSpec { Name = "HoldR", K1 = -rInner, K2 = -rInner, K3 = -rInner, CharacteristicLat = -rInner });
                r.Add(new ShapeSpec { Name = "PassReturnR", K1 = -rInner, K2 = -rInner, K3 = 0f, CharacteristicLat = -rInner });
            }
            if (lOuter >= lInner + MinCandidateSpacingM && lOuter >= 2.4f)
                r.Add(new ShapeSpec { Name = "BoldL", K1 = lOuter, K2 = lOuter, K3 = lOuter * 0.65f, CharacteristicLat = lOuter });
            if (rOuter >= rInner + MinCandidateSpacingM && rOuter >= 2.4f)
                r.Add(new ShapeSpec { Name = "BoldR", K1 = -rOuter, K2 = -rOuter, K3 = -rOuter * 0.65f, CharacteristicLat = -rOuter });

            try
            {
                int mid = Math.Max(1, reference.Path.Count / 2);
                Vector3 d0 = DirectionAt(reference.Path, 0);
                Vector3 dm = DirectionAt(reference.Path, mid);
                float turnDeg = RaceMath.SignedAngleDeg(d0, dm);
                if (Math.Abs(turnDeg) >= 12f)
                {
                    float sign = Math.Sign(turnDeg);
                    float outsideAvail = sign > 0 ? rightUsable : leftUsable;
                    float insideAvail = sign > 0 ? leftUsable : rightUsable;
                    float outside = Math.Min(2.8f, outsideAvail * 0.72f);
                    float inside = Math.Min(3.2f, insideAvail * 0.82f);
                    if (outside >= 1.0f && inside >= 1.0f)
                    {
                        r.Add(new ShapeSpec
                        {
                            Name = sign > 0 ? "ApexL" : "ApexR",
                            K1 = -sign * outside,
                            K2 = sign * inside,
                            K3 = -sign * outside * 0.55f,
                            CharacteristicLat = sign * inside,
                        });
                    }
                }
            }
            catch { }
            return r;
        }

        private static void ReferenceUsableSides(
            DrivingReference.Result reference, float horizon,
            out float left, out float right)
        {
            left = 4f;
            right = 4f;
            try
            {
                var ls = new List<float>();
                var rs = new List<float>();
                for (int i = 0; i < reference.Path.Count; i++)
                {
                    float s = reference.StationS[i];
                    if (s > horizon) break;
                    if (s < 6f) continue; // current pinch is checked point-by-point
                    if (i < reference.LeftRoadM.Count)
                        ls.Add(reference.LeftRoadM[i] - VehicleHalfWidthM - RoadMarginM);
                    if (i < reference.RightRoadM.Count)
                        rs.Add(reference.RightRoadM[i] - VehicleHalfWidthM - RoadMarginM);
                }
                if (ls.Count > 0)
                {
                    ls.Sort();
                    left = ls[(int)Math.Floor((ls.Count - 1) * 0.65f)];
                }
                if (rs.Count > 0)
                {
                    rs.Sort();
                    right = rs[(int)Math.Floor((rs.Count - 1) * 0.65f)];
                }
            }
            catch { }
            left = RaceMath.Clamp(left, 0.4f, 6f);
            right = RaceMath.Clamp(right, 0.4f, 6f);
        }

        private TrajectoryCandidate BuildCandidate(
            DrivingReference.Result reference,
            RaceRoute route,
            Perception perception,
            VehicleCapability capability,
            DriverProfile profile,
            Vector3 egoPos,
            float egoHeading,
            float egoSpeed,
            float cruise,
            ShapeSpec shape,
            int index)
        {
            var c = new TrajectoryCandidate
            {
                CandidateIndex = index,
                Shape = shape.Name,
                LateralM = shape.CharacteristicLat,
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
                float s1 = totalS * 0.30f;
                float s2 = totalS * 0.62f;
                float s3 = totalS;

                for (int i = 0; i < reference.Path.Count; i++)
                {
                    float baseS = reference.StationS[i];
                    float lat = PiecewiseLateral(startLat, m0, shape.K1, shape.K2, shape.K3,
                        baseS, s1, s2, s3);
                    Vector3 dir = DirectionAt(reference.Path, i);
                    Vector3 left = new Vector3(-dir.Y, dir.X, 0f);
                    Vector3 bp = reference.Path[i];
                    var p = new Vector3(bp.X + left.X * lat, bp.Y + left.Y * lat, bp.Z);
                    if (i == 0) p = egoPos;

                    float leftAvail;
                    float rightAvail;
                    ReferenceRoadAt(reference, baseS, out leftAvail, out rightAvail);
                    float leftAllow = Math.Max(0.4f, leftAvail - VehicleHalfWidthM - RoadMarginM);
                    float rightAllow = Math.Max(0.4f, rightAvail - VehicleHalfWidthM - RoadMarginM);
                    if (lat > leftAllow + 0.05f || lat < -rightAllow - 0.05f)
                    {
                        c.RejectReason = $"road-boundary@{baseS:F0}:lat={lat:F1}/allow=-{rightAllow:F1}..{leftAllow:F1}";
                        return c;
                    }

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
                // Curvature is normally a SPEED constraint, not a reason to
                // throw the trajectory away. Only reject near-cusps that the
                // low-level controller cannot represent at any useful speed.
                if (maxKappa > 0.45f)
                {
                    c.RejectReason = $"kappa-pathological:{maxKappa:F3}";
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
                float[] actorAllow = BuildWorldActorEnvelope(path, ss, roadProfile,
                    perception, egoSpeed, cruise, profile,
                    out constrainHandle, out constrainKind, out constrainS, out minClear);

                float[] allow = new float[path.Count];
                for (int i = 0; i < allow.Length; i++)
                    allow[i] = Math.Min(roadAllow[i], actorAllow[i]);
                float[] desired = BackwardPass(allow, ss, aBrake);

                // One fixed-point refinement: once the first pass decides we
                // will slow, actor arrival prediction must use that slower
                // trajectory instead of pretending we keep current speed.
                int cHandle2;
                string cKind2;
                float cS2;
                float minClear2;
                float[] actorAllow2 = BuildWorldActorEnvelope(path, ss, desired,
                    perception, egoSpeed, cruise, profile,
                    out cHandle2, out cKind2, out cS2, out minClear2);
                for (int i = 0; i < allow.Length; i++)
                    allow[i] = Math.Min(roadAllow[i], actorAllow2[i]);
                desired = BackwardPass(allow, ss, aBrake);
                constrainHandle = cHandle2;
                constrainKind = cKind2;
                constrainS = cS2;
                minClear = minClear2;

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
                    float leftAvail;
                    float rightAvail;
                    ReferenceRoadAt(reference, reference.StationS[Math.Min(i, reference.StationS.Count - 1)],
                        out leftAvail, out rightAvail);
                    float m = lats[i] >= 0f
                        ? leftAvail - VehicleHalfWidthM - Math.Abs(lats[i])
                        : rightAvail - VehicleHalfWidthM - Math.Abs(lats[i]);
                    if (m < roadMargin) roadMargin = m;
                }
                if (roadMargin == float.MaxValue) roadMargin = 0f;

                // Racing objective: maximize ROUTE progress per predicted
                // travel time, then break ties with clearance/margin/smoothness.
                // This lets an apex/shortcut win because it gets farther along
                // the route sooner, not because of a hard-coded "corner line".
                var predictedArrival = BuildArrivalTimes(ss, desired);
                float eta = predictedArrival.Count > 0 ? predictedArrival[predictedArrival.Count - 1] : 99f;
                float referenceProgressRate = totalS / Math.Max(eta, 0.75f);
                float score = referenceProgressRate * 5.5f
                    + min * 1.15f
                    + RaceMath.Clamp(minClear, -2f, 6f) * 1.25f
                    + RaceMath.Clamp(roadMargin, -2f, 5f) * 0.75f
                    - Math.Abs(shape.CharacteristicLat) * 0.55f
                    - maxKappa * 20f
                    - firstTang * 0.08f;

                // Small continuity preference around the current commitment.
                if (Math.Abs(committedLat) > 0.35f)
                    score -= Math.Abs(shape.CharacteristicLat - committedLat) * 0.65f;

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
                c.ArrivalT = predictedArrival;
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
            IList<float> provisionalSpeed,
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

            var arrivalList = BuildArrivalTimes(ss, provisionalSpeed);
            var arrival = new float[n];
            for (int i = 0; i < n; i++)
            {
                float t = i < arrivalList.Count
                    ? arrivalList[i]
                    : ss[i] / Math.Max(egoSpeed, 6f);
                arrival[i] = RaceMath.Clamp(t, 0f, 5f);
            }

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

                    // A car already beside us is not an immediate collision
                    // merely because every candidate starts at egoPos. Ignore
                    // the shared s~0 point when the actor is lateral, not ahead,
                    // and not rapidly closing. Future stations still evaluate it.
                    if (ss[i] < 3.0f
                        && a.Longitudinal < 2.0f
                        && Math.Abs(a.Lateral) > 1.35f
                        && a.ClosingSpeed < 2.0f)
                        continue;

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

        private static float PiecewiseLateral(
            float d0, float m0, float k1, float k2, float k3,
            float s, float s1, float s2, float s3)
        {
            if (s <= 0f) return d0;
            if (s <= s1)
                return HermiteSegment(d0, m0, k1, 0f, s, 0f, s1);
            if (s <= s2)
                return HermiteSegment(k1, 0f, k2, 0f, s, s1, s2);
            return HermiteSegment(k2, 0f, k3, 0f, Math.Min(s, s3), s2, s3);
        }

        private static float HermiteSegment(
            float d0, float m0, float d1, float m1,
            float s, float s0, float s1)
        {
            float S = Math.Max(0.5f, s1 - s0);
            float t = RaceMath.Clamp((s - s0) / S, 0f, 1f);
            float t2 = t * t;
            float t3 = t2 * t;
            float h00 = 2f * t3 - 3f * t2 + 1f;
            float h10 = t3 - 2f * t2 + t;
            float h01 = -2f * t3 + 3f * t2;
            float h11 = t3 - t2;
            return h00 * d0 + h10 * S * m0 + h01 * d1 + h11 * S * m1;
        }

        private static void ReferenceRoadAt(
            DrivingReference.Result reference, float s, out float left, out float right)
        {
            left = 4f;
            right = 4f;
            try
            {
                if (reference.StationS.Count == 0) return;
                if (s <= 0f)
                {
                    left = reference.LeftRoadM[0];
                    right = reference.RightRoadM[0];
                    return;
                }
                int last = reference.StationS.Count - 1;
                if (s >= reference.StationS[last])
                {
                    left = reference.LeftRoadM[last];
                    right = reference.RightRoadM[last];
                    return;
                }
                for (int i = 0; i < last; i++)
                {
                    float a = reference.StationS[i];
                    float b = reference.StationS[i + 1];
                    if (s < a || s > b) continue;
                    float t = b > a ? (s - a) / (b - a) : 0f;
                    left = reference.LeftRoadM[i] + (reference.LeftRoadM[i + 1] - reference.LeftRoadM[i]) * t;
                    right = reference.RightRoadM[i] + (reference.RightRoadM[i + 1] - reference.RightRoadM[i]) * t;
                    return;
                }
            }
            catch { }
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

        private static string SummarizeCandidates(IList<TrajectoryCandidate> candidates)
        {
            try
            {
                var parts = new List<string>();
                for (int i = 0; i < candidates.Count; i++)
                {
                    var c = candidates[i];
                    if (!string.IsNullOrEmpty(c.RejectReason))
                    {
                        parts.Add($"{c.Shape}@{c.LateralM:+0.0;-0.0;0.0}:X({c.RejectReason})");
                        continue;
                    }
                    string lim = c.ConstrainHandle != -1 ? "T" + c.ConstrainHandle : "-";
                    parts.Add($"{c.Shape}@{c.LateralM:+0.0;-0.0;0.0}:S{c.Score:F0}/V{c.MeanSpeed:F1}/M{c.MinSpeed:F1}/{lim}/C{c.MinPredClearance:F1}");
                }
                return string.Join("|", parts);
            }
            catch { return "?"; }
        }

        private static int FindShapeIndex(IList<TrajectoryCandidate> candidates, string shape)
        {
            if (string.IsNullOrEmpty(shape)) return -1;
            for (int i = 0; i < candidates.Count; i++)
                if (string.Equals(candidates[i].Shape, shape, StringComparison.Ordinal))
                    return i;
            return -1;
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
