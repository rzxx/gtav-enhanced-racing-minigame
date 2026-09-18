using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    internal struct TrajectoryCandidate
    {
        public float LateralM;     // at lookahead
        public float LookaheadM;
        public Vector3 AimPoint;
        public float Score;
        public float ClearanceM;
        public float CurveCost;
        public float TacticalBias;
        public string RejectReason; // empty = viable
    }

    /// Generates lateral path options across the full usable road width and
    /// scores them. This replaces blind GTA node-following: the actuator is
    /// commanded to the *chosen* aim point, and rejected alternatives are
    /// logged so bad choices are explainable.
    ///
    /// Street-racing rules: the whole carriageway (both lanes + shoulders
    /// inside the road boundary) is drivable. Oncoming-lane use is allowed
    /// but penalised by risk unless the tactical state explicitly commits to
    /// an overtake and the lane is predicted clear.
    internal sealed class TrajectoryPlanner
    {
        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public TrajectoryCandidate Chosen;
        public bool HasChosen;

        public TrajectoryCandidate Plan(
            RaceRoute route,
            RoadCorridor corridor,
            Perception perception,
            Tactics.RaceTactics tactics,
            DriverProfile profile,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            float lookaheadM)
        {
            LastCandidates.Clear();
            HasChosen = false;

            float half = corridor.HalfWidth;
            if (half < 2.5f) half = 2.5f;
            if (half > 18f) half = 18f;

            // 5 options spanning the road. Tactical bias shifts the set toward
            // the desired overtake/defend side without removing alternatives.
            float bias = tactics.DesiredLateral; // -1..+1
            float[] fracs = { -0.8f, -0.4f, 0f, 0.4f, 0.8f };
            float arriveT = lookaheadM / Math.Max(egoSpeed, 6f);
            arriveT = RaceMath.Clamp(arriveT, 0.8f, 5f);

            // Corner anticipation: prefer the inside before a bend.
            float curveAhead = route.CurvatureAhead(lookaheadM);
            float headNow = route.HeadingAhead(0f);
            float headAhead = route.HeadingAhead(lookaheadM);
            float turnDir = Math.Sign(RaceMath.HeadingDiffDeg(headAhead, headNow)); // + = left
            float insideBias = 0f;
            if (curveAhead > 0.002f)
                insideBias = turnDir * RaceMath.Clamp(curveAhead * 900f, 0f, 0.5f);

            foreach (float f in fracs)
            {
                float lat = (f + bias * 0.35f + insideBias * 0.5f) * half;
                lat = RaceMath.Clamp(lat, -half * 1.05f, half * 1.05f);
                Vector3 aim = corridor.PointAtLateral(lat, lookaheadM, route);

                // Clearance to predicted actor positions at arrival time.
                float clearance = 999f;
                string blockReason = "";
                foreach (var a in perception.Actors)
                {
                    Vector3 pred = perception.Predict(a, arriveT);
                    float d = RaceMath.FlatDistance(aim, pred);
                    // Also consider the path corridor, not just the endpoint:
                    // actors near the straight ego->aim line threaten the run.
                    var proj = RaceMath.ProjectOnSegment(pred, egoPos, aim);
                    float pathDist = proj.Dist;
                    float threat = Math.Min(d, pathDist + 2f);
                    if (threat < clearance) clearance = threat;
                    if (threat < 3.5f && a.IsAhead && blockReason == "")
                        blockReason = a.Kind == ActorKind.Ped ? "ped" : (a.Kind == ActorKind.Obstacle ? "obstacle" : "traffic");
                }

                // Sharp lateral jumps cost more at speed.
                float curveCost = Math.Abs(lat - route.Lateral) / Math.Max(lookaheadM, 10f);
                curveCost += curveAhead * 220f * (Math.Abs(lat) / Math.Max(half, 1f)) * 0.3f;

                float tacticalBias = 0f;
                if (Math.Abs(bias) > 0.05f)
                {
                    float wantLat = bias * half;
                    tacticalBias = -(Math.Abs(lat - wantLat) / half) * 1.2f;
                }
                tacticalBias += -Math.Abs(lat / half - insideBias) * 0.25f;

                // Risk model: clearance below need is heavily penalised unless
                // committed to an overtake with a still-safe TTC.
                float need = profile.ClearanceNeed(5f + egoSpeed * 0.12f);
                float clearScore;
                if (clearance >= need) clearScore = 2f;
                else if (clearance >= need * 0.55f)
                    clearScore = (clearance / need) * 2f - 1f;
                else
                    clearScore = -4f + (clearance / Math.Max(need, 1f)) * 2f;

                if (tactics.Mode == Tactics.TacticalMode.Commit && blockReason == "" && clearance > need * 0.6f)
                    clearScore += 0.8f; // committed: hold the gap, don't swerve
                if (tactics.Mode == Tactics.TacticalMode.Abort)
                    clearScore += (Math.Abs(lat) < half * 0.35f ? 1.0f : 0f); // tuck back in

                float score = 2f - curveCost * 6f + clearScore + tacticalBias;
                // Slight preference for the race line (center-inside) when free.
                score -= Math.Abs(f) * 0.1f;

                var c = new TrajectoryCandidate
                {
                    LateralM = lat,
                    LookaheadM = lookaheadM,
                    AimPoint = aim,
                    Score = score,
                    ClearanceM = clearance,
                    CurveCost = curveCost,
                    TacticalBias = tacticalBias,
                    RejectReason = blockReason,
                };
                LastCandidates.Add(c);
            }

            LastCandidates.Sort((a, b) => b.Score.CompareTo(a.Score));
            // If the best is blocked and we are not committed, the second-best
            // (usually a tuck-in) wins — the planner visibly "aborts".
            Chosen = LastCandidates[0];
            HasChosen = true;
            return Chosen;
        }
    }
}
