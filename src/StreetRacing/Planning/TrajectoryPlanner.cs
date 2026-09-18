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
        public float ClearanceM;   // swept min distance to any predicted actor
        public float CurveCost;
        public float TacticalBias;
        public string RejectReason; // empty = viable
        // Sampled path through the corridor (world points, ego -> aim).
        public List<Vector3> Path;
        public float MinMarginM;    // min (halfWidth - |lat|) along path (+ = inside)
        public float MaxKappa;      // max path curvature rad/m
    }

    /// Generates sampled candidate paths through the corridor and scores the
    /// swept path over time — not just endpoints.
    ///
    /// Each candidate is a smooth lateral blend from the current ego lateral
    /// to a target lateral at lookahead (smoothstep), resampled every ~10 m
    /// in route coordinates. Scoring integrates, along the whole path:
    ///   road boundaries (per-station corridor slices),
    ///   path curvature (lateral-g demand at current speed),
    ///   predicted traffic / peds / rival / obstacles (constant-velocity
    ///     predictions vs distance to the path polyline, not just the aim).
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

        private const float StationDs = 10f;

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

            float halfAtLook = corridor.HalfWidthAt(lookaheadM);
            if (halfAtLook < 2.5f) halfAtLook = 2.5f;
            if (halfAtLook > 18f) halfAtLook = 18f;

            float startLat = 0f;
            try { startLat = route.Lateral; } catch { }
            startLat = RaceMath.Clamp(startLat, -18f, 18f);

            // 7 options spanning the road. Tactical bias shifts the set toward
            // the desired overtake/defend side without removing alternatives.
            float bias = tactics.DesiredLateral; // -1..+1
            float[] fracs = { -0.85f, -0.55f, -0.30f, 0f, 0.30f, 0.55f, 0.85f };
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

            int nStations = Math.Max(4, Math.Min(17, (int)Math.Ceiling(lookaheadM / StationDs) + 1));

            foreach (float f in fracs)
            {
                float endLat = (f + bias * 0.35f + insideBias * 0.5f) * halfAtLook;
                endLat = RaceMath.Clamp(endLat, -halfAtLook * 1.05f - 1f, halfAtLook * 1.05f + 1f);

                // Build smooth path: smoothstep blend start->end in route frame.
                var path = new List<Vector3>(nStations);
                var pathLats = new List<float>(nStations);
                var pathS = new List<float>(nStations);
                for (int k = 0; k < nStations; k++)
                {
                    float s = k == nStations - 1 ? lookaheadM : k * StationDs;
                    if (s > lookaheadM) s = lookaheadM;
                    float t = lookaheadM > 1f ? s / lookaheadM : 1f;
                    float sm = t * t * (3f - 2f * t);
                    float lat = startLat + (endLat - startLat) * sm;
                    Vector3 rp = route.PointAtS(route.AlongS + s);
                    float h = route.HeadingAtS(route.AlongS + s);
                    var dir = RaceMath.VectorFromHeading(h);
                    var leftV = new Vector3(-dir.Y, dir.X, 0f);
                    var wp = new Vector3(rp.X + leftV.X * lat, rp.Y + leftV.Y * lat, rp.Z);
                    path.Add(wp);
                    pathLats.Add(lat);
                    pathS.Add(s);
                    if (s >= lookaheadM - 0.01f) break;
                }
                // Pin the first point to the true ego position for continuity
                // (route lateral can be stale by a plan tick).
                if (path.Count > 0) path[0] = new Vector3(egoPos.X, egoPos.Y, path[0].Z);

                Vector3 aim = path[path.Count - 1];

                // --- Swept corridor margin + path curvature.
                float minMargin = float.MaxValue;
                float maxKappa = 0f;
                float curveLenCost = 0f;
                for (int k = 0; k < path.Count; k++)
                {
                    float s = pathS[k];
                    float half = corridor.HalfWidthAt(s);
                    float margin = half - Math.Abs(pathLats[k]);
                    if (margin < minMargin) minMargin = margin;
                    if (k >= 1)
                    {
                        var d0 = new Vector3(path[k].X - path[k - 1].X, path[k].Y - path[k - 1].Y, 0f);
                        Vector3 d1;
                        if (k + 1 < path.Count)
                            d1 = new Vector3(path[k + 1].X - path[k].X, path[k + 1].Y - path[k].Y, 0f);
                        else
                            d1 = d0;
                        float l0 = RaceMath.FlatLength(d0);
                        float l1 = RaceMath.FlatLength(d1);
                        if (l0 > 0.5f && l1 > 0.5f)
                        {
                            float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
                            float kappa = dh / Math.Max((l0 + l1) * 0.5f, 1f);
                            if (kappa > maxKappa) maxKappa = kappa;
                            curveLenCost += kappa * l0;
                        }
                    }
                }
                // Sharp lateral jumps cost more at speed (endpoint term, kept).
                float curveCost = Math.Abs(endLat - startLat) / Math.Max(lookaheadM, 10f);
                curveCost += curveAhead * 220f * (Math.Abs(endLat) / Math.Max(halfAtLook, 1f)) * 0.3f;
                curveCost += curveLenCost * 0.5f;
                // Extra cost for curvature the tyres must actually hold.
                float latNeed = egoSpeed * egoSpeed * maxKappa;

                // --- Swept clearance to predicted actors.
                float clearance = 999f;
                string blockReason = "";
                float blockDist = 999f;
                foreach (var a in perception.Actors)
                {
                    // Time at which ego reaches each station; use the closest
                    // approach over stations (actor moves during our transit).
                    float worstForActor = 999f;
                    for (int k = 0; k < path.Count; k++)
                    {
                        float s = pathS[k];
                        float t = s / Math.Max(egoSpeed, 6f);
                        t = RaceMath.Clamp(t, 0f, 5f);
                        Vector3 pred = perception.Predict(a, t);
                        float d = RaceMath.FlatDistance(path[k], pred);
                        if (d < worstForActor) worstForActor = d;
                    }
                    // Also consider mid-segment distance for actors between
                    // stations (cheap: distance to ego->aim line as before is
                    // now redundant; use path polyline min instead).
                    float threat = worstForActor;
                    if (threat < clearance) clearance = threat;
                    bool ahead = a.RouteValid
                        ? (a.RouteDist > -6f && a.RouteDist < lookaheadM + 40f)
                        : a.IsAhead;
                    float blockThresh = a.Kind == ActorKind.Ped ? 4.2f : 3.5f;
                    if (a.Kind == ActorKind.Obstacle) blockThresh = 3.2f;
                    if (threat < blockThresh && ahead && blockReason == "")
                    {
                        blockReason = a.Kind == ActorKind.Ped ? "ped" : (a.Kind == ActorKind.Obstacle ? "obstacle" : (a.Kind == ActorKind.Rival ? "rival" : "traffic"));
                        blockDist = threat;
                    }
                }

                float tacticalBias = 0f;
                if (Math.Abs(bias) > 0.05f)
                {
                    float wantLat = bias * halfAtLook;
                    tacticalBias = -(Math.Abs(endLat - wantLat) / halfAtLook) * 1.2f;
                }
                tacticalBias += -Math.Abs(endLat / halfAtLook - insideBias) * 0.25f;

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
                    clearScore += (Math.Abs(endLat) < halfAtLook * 0.35f ? 1.0f : 0f); // tuck back in

                // Road-boundary term: leaving the surface is worse than traffic.
                float offPenalty = 0f;
                if (minMargin < 0f) offPenalty = 6f + (-minMargin) * 2.5f;
                else if (minMargin < 1f) offPenalty = (1f - minMargin) * 0.8f;

                // Curvature feasibility: if the path demands more lateral-g
                // than the car can plausibly hold, penalise (speed planner
                // will also slow, but a straighter candidate is better).
                float kappaPenalty = 0f;
                if (latNeed > 9f) kappaPenalty = (latNeed - 9f) * 0.25f;

                float score = 2f - curveCost * 6f + clearScore + tacticalBias - offPenalty - kappaPenalty;
                // Slight preference for the race line (center-inside) when free.
                score -= Math.Abs(f) * 0.1f;

                var c = new TrajectoryCandidate
                {
                    LateralM = endLat,
                    LookaheadM = lookaheadM,
                    AimPoint = aim,
                    Score = score,
                    ClearanceM = clearance,
                    CurveCost = curveCost,
                    TacticalBias = tacticalBias,
                    RejectReason = blockReason,
                    Path = path,
                    MinMarginM = minMargin == float.MaxValue ? 99f : minMargin,
                    MaxKappa = maxKappa,
                };
                // Off-road paths are marked (still sortable, but never silently chosen).
                if (minMargin < -1.5f && c.RejectReason == "")
                    c.RejectReason = "offroad";
                LastCandidates.Add(c);
            }

            LastCandidates.Sort((a, b) => b.Score.CompareTo(a.Score));
            // If the best is blocked and we are not committed, the first viable
            // (usually a tuck-in) wins — the planner visibly "aborts".
            Chosen = LastCandidates[0];
            if (!string.IsNullOrEmpty(Chosen.RejectReason) && tactics.Mode != Tactics.TacticalMode.Commit)
            {
                foreach (var c in LastCandidates)
                {
                    if (string.IsNullOrEmpty(c.RejectReason)) { Chosen = c; break; }
                }
            }
            HasChosen = true;
            return Chosen;
        }
    }
}
