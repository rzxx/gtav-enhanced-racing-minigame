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

        // --- Joint maneuver fields: this candidate WITH its executable speed.
        public List<float> StationS;      // s ahead (m) per path station
        public List<float> SpeedProfile;  // executable target speed per station
        public List<float> ArrivalT;      // predicted arrival time per station
        public float TargetSpeed;         // SpeedProfile[0] (what to drive now)
        public string SpeedLimiting;      // Cruise/Curvature/Follow/.../Obstacle:<kind>#h@s
        public int ConstrainHandle;       // actor handle that binds this path (-1 none)
        public string ConstrainKind;      // actor kind string for telemetry
        public float ConstrainS;          // station s (m) of the binding constraint
        public float MinPredClearance;    // min predicted envelope clearance (m)
        public float MeanSpeed;           // mean of SpeedProfile
        public float MinSpeed;            // min of SpeedProfile
        public float RequiredDecel;       // + = need to slow now
        public int CandidateIndex;
    }

    /// JOINT trajectory + speed planner.
    ///
    /// The old architecture selected a path (trajectory scorer) and THEN
    /// applied one global corridor-wide obstacle speed (speed planner). A
    /// path that avoided an actor was still braked for it: the planner could
    /// steer around while the speed planner stopped for the same actor.
    ///
    /// Now every candidate carries its own physically executable maneuver:
    ///   curvature speed profile -> station arrival times -> predict actors
    ///   at those times -> swept-envelope test along THAT path -> constrain
    ///   speed only for actors conflicting with THAT path -> backwards
    ///   braking pass (+ forward accel feasibility) -> score the complete
    ///   maneuver (safety, progress, smoothness, racing intent, road margin).
    ///
    /// The winner is a path+speed TOGETHER. An avoiding path keeps speed and
    /// beats a stopped center path on progress — the deadlock is structurally
    /// gone, not tuned around.
    ///
    /// Street-racing rules: the whole carriageway is drivable. Oncoming-lane
    /// use is allowed but penalised by risk unless tactics commits.
    internal sealed class TrajectoryPlanner
    {
        public readonly List<TrajectoryCandidate> LastCandidates = new List<TrajectoryCandidate>();
        public TrajectoryCandidate Chosen;
        public bool HasChosen;
        public float BrakingPointS = -1f;
        public int PlanId;

        private const float StationDs = 10f;
        private float lastEndLat;
        private bool hasLast;

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
            // Back-compat shim: joint plan with a neutral capability.
            // Prefer PlanJoint (capability-aware). This path exists so stale
            // callers cannot silently reintroduce decoupled planning.
            var cap = new VehicleCapability();
            try { cap.ALatMax = 7.5f; cap.ABrakeMax = 7f; cap.TopSpeedEst = 60f; } catch { }
            return PlanJoint(route, corridor, perception, tactics, profile, cap,
                egoPos, egoFwd, egoSpeed, lookaheadM, 47f);
        }

        public TrajectoryCandidate PlanJoint(
            RaceRoute route,
            RoadCorridor corridor,
            Perception perception,
            Tactics.RaceTactics tactics,
            DriverProfile profile,
            VehicleCapability cap,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            float lookaheadM,
            float cruise)
        {
            LastCandidates.Clear();
            HasChosen = false;
            BrakingPointS = -1f;
            PlanId++;

            float halfAtLook = corridor.HalfWidthAt(lookaheadM);
            if (halfAtLook < 2.5f) halfAtLook = 2.5f;
            if (halfAtLook > 18f) halfAtLook = 18f;

            float startLat = 0f;
            try { startLat = route.Lateral; } catch { }
            startLat = RaceMath.Clamp(startLat, -18f, 18f);

            float bias = tactics.DesiredLateral;
            float[] fracs = { -0.85f, -0.55f, -0.30f, 0f, 0.30f, 0.55f, 0.85f };

            float curveAhead = route.CurvatureAhead(lookaheadM);
            float headNow = route.HeadingAhead(0f);
            float headAhead = route.HeadingAhead(lookaheadM);
            float turnDir = Math.Sign(RaceMath.HeadingDiffDeg(headAhead, headNow));
            float insideBias = 0f;
            if (curveAhead > 0.002f)
                insideBias = turnDir * RaceMath.Clamp(curveAhead * 900f, 0f, 0.5f);

            int nStations = Math.Max(4, Math.Min(17, (int)Math.Ceiling(lookaheadM / StationDs) + 1));

            float aLatRaw = cap.UsableLat(profile.GripFactor);
            float cc = profile.CornerCaution;
            if (cc < 0.5f) cc = 0.5f;
            if (cc > 2f) cc = 2f;
            float aLatEff = aLatRaw / (cc * cc);
            float aBrake = cap.UsableBrake(profile.GripFactor);
            float topSpeed = 60f;
            try { topSpeed = cap.TopSpeedEst; } catch { }

            float tacticalCap = TacticalCap(tactics.Mode, perception, route, corridor, cruise, egoSpeed);

            for (int ci = 0; ci < fracs.Length; ci++)
            {
                float f = fracs[ci];
                float endLat = (f + bias * 0.35f + insideBias * 0.5f) * halfAtLook;
                endLat = RaceMath.Clamp(endLat, -halfAtLook * 1.05f - 1f, halfAtLook * 1.05f + 1f);

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
                if (path.Count > 0) path[0] = new Vector3(egoPos.X, egoPos.Y, path[0].Z);
                Vector3 aim = path[path.Count - 1];

                // Swept margin + per-station curvature.
                float minMargin = float.MaxValue;
                float maxKappa = 0f;
                float curveLenCost = 0f;
                var kappa = new List<float>(path.Count);
                for (int k = 0; k < path.Count; k++) kappa.Add(0f);
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
                            float kk = dh / Math.Max((l0 + l1) * 0.5f, 1f);
                            kappa[k] = kk;
                            if (kk > maxKappa) maxKappa = kk;
                            curveLenCost += kk * l0;
                        }
                    }
                }
                float curveCost = Math.Abs(endLat - startLat) / Math.Max(lookaheadM, 10f);
                curveCost += curveAhead * 220f * (Math.Abs(endLat) / Math.Max(halfAtLook, 1f)) * 0.3f;
                curveCost += curveLenCost * 0.5f;
                float latNeed = egoSpeed * egoSpeed * maxKappa;

                // --- JOINT speed: profile constrained ONLY by this path's
                // conflicts (SpeedPlanner.ProfileForPath).
                float[] vAllow;
                float[] vTgt;
                float[] arrivalT;
                int cHandle;
                string cKind;
                float cS;
                float minPredClear;
                try
                {
                    float egoRouteS = 0f;
                    try { egoRouteS = route.AlongS; } catch { }
                    SpeedPlanner.ProfileForPath(path, pathS, kappa, pathLats, egoRouteS,
                        perception, corridor,
                        egoSpeed, cruise, aLatEff, aBrake, topSpeed, profile, tacticalCap,
                        out vAllow, out vTgt, out arrivalT, out cHandle, out cKind, out cS, out minPredClear);
                }
                catch
                {
                    vAllow = new float[path.Count];
                    vTgt = new float[path.Count];
                    arrivalT = new float[path.Count];
                    for (int i = 0; i < path.Count; i++) { vAllow[i] = cruise; vTgt[i] = cruise; arrivalT[i] = pathS[i] / Math.Max(egoSpeed, 6f); }
                    cHandle = -1; cKind = ""; cS = -1f; minPredClear = 999f;
                }

                float meanV = 0f;
                float minV = float.MaxValue;
                for (int i = 0; i < vTgt.Length; i++)
                {
                    meanV += vTgt[i];
                    if (vTgt[i] < minV) minV = vTgt[i];
                }
                if (vTgt.Length > 0) meanV /= vTgt.Length;
                else { meanV = cruise; minV = cruise; }

                float targetNow = vTgt.Length > 0 ? vTgt[0] : cruise;
                float reqDecel = egoSpeed > targetNow
                    ? (egoSpeed - targetNow) * (egoSpeed - targetNow) / Math.Max(2f * Math.Max(lookaheadM * 0.5f, 12f), 1f)
                    : 0f;

                string speedLimiting = "Cruise";
                if (cHandle != -1 || cKind != "")
                    speedLimiting = "Obstacle:" + cKind + "#" + cHandle + "@" + (cS >= 0 ? cS.ToString("F0") : "?");
                else if (tacticalCap < cruise - 0.5f)
                    speedLimiting = tactics.Mode.ToString();
                else if (targetNow < cruise - 0.5f)
                    speedLimiting = "Curvature";

                // --- Score the COMPLETE maneuver.
                float need = profile.ClearanceNeed(5f + egoSpeed * 0.12f);
                float clearance = minPredClear;
                float clearScore;
                if (clearance >= need) clearScore = 2f;
                else if (clearance >= need * 0.55f)
                    clearScore = (clearance / need) * 2f - 1f;
                else
                    clearScore = -4f + (clearance / Math.Max(need, 1f)) * 2f;
                // Hard near-collision veto level (still scored, veto below).
                if (clearance < 0.6f) clearScore -= 6f;

                if (tactics.Mode == Tactics.TacticalMode.Commit && clearance > need * 0.6f)
                    clearScore += 0.8f;
                if (tactics.Mode == Tactics.TacticalMode.Abort)
                    clearScore += (Math.Abs(endLat) < halfAtLook * 0.35f ? 1.0f : 0f);

                float offPenalty = 0f;
                if (minMargin < 0f) offPenalty = 6f + (-minMargin) * 2.5f;
                else if (minMargin < 1f) offPenalty = (1f - minMargin) * 0.8f;

                float kappaPenalty = 0f;
                if (latNeed > 9f) kappaPenalty = (latNeed - 9f) * 0.25f;

                // Longitudinal feasibility: demanding more decel than the
                // tyres hold is not a plan.
                float longPenalty = 0f;
                if (reqDecel > aBrake * 1.1f) longPenalty = (reqDecel - aBrake) * 1.5f + 3f;
                else if (reqDecel > aBrake * 0.7f) longPenalty = (reqDecel - aBrake * 0.7f) * 0.4f;

                float tacticalBias = 0f;
                if (Math.Abs(bias) > 0.05f)
                {
                    float wantLat = bias * halfAtLook;
                    tacticalBias = -(Math.Abs(endLat - wantLat) / halfAtLook) * 1.2f;
                }
                tacticalBias += -Math.Abs(endLat / halfAtLook - insideBias) * 0.25f;

                float progressScore = (meanV / Math.Max(cruise, 1f)) * 3.0f;

                // Hysteresis against perception flicker: small bonus for
                // staying near the previous choice so equal maneuvers don't
                // oscillate and repath every tick.
                float hystBonus = 0f;
                if (hasLast && Math.Abs(endLat - lastEndLat) < 2f) hystBonus = 0.35f;

                float score = 2f + progressScore - curveCost * 6f + clearScore + tacticalBias
                    - offPenalty - kappaPenalty - longPenalty + hystBonus;
                score -= Math.Abs(f) * 0.1f;

                string blockReason = "";
                if (minMargin < -1.5f) blockReason = "offroad";
                else if (clearance < 1.0f)
                {
                    blockReason = !string.IsNullOrEmpty(cKind)
                        ? cKind.ToLowerInvariant().Contains("ped") ? "ped"
                          : cKind.ToLowerInvariant().Contains("obstacle") ? "obstacle"
                          : cKind.ToLowerInvariant().Contains("rival") ? "rival" : "traffic"
                        : "traffic";
                }

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
                    StationS = new List<float>(pathS),
                    SpeedProfile = new List<float>(vTgt),
                    ArrivalT = new List<float>(arrivalT),
                    TargetSpeed = targetNow,
                    SpeedLimiting = speedLimiting,
                    ConstrainHandle = cHandle,
                    ConstrainKind = cKind ?? "",
                    ConstrainS = cS,
                    MinPredClearance = minPredClear,
                    MeanSpeed = meanV,
                    MinSpeed = minV == float.MaxValue ? cruise : minV,
                    RequiredDecel = reqDecel,
                    CandidateIndex = ci,
                };
                LastCandidates.Add(c);
            }

            LastCandidates.Sort((a, b) => b.Score.CompareTo(a.Score));

            // Safety veto: a hard-collision best never beats a safe viable
            // alternative on progress alone. Off-road best yields similarly.
            TrajectoryCandidate best = LastCandidates[0];
            if (!string.IsNullOrEmpty(best.RejectReason) && best.RejectReason != "offroad"
                && best.MinPredClearance < 0.6f)
            {
                foreach (var c in LastCandidates)
                {
                    if (c.MinPredClearance >= 1.2f && c.MinMarginM >= -1.5f) { best = c; break; }
                }
            }
            if (best.MinMarginM < -1.5f)
            {
                foreach (var c in LastCandidates)
                {
                    if (c.MinMarginM >= -1.5f) { best = c; break; }
                }
            }
            Chosen = best;
            HasChosen = true;
            lastEndLat = Chosen.LateralM;
            hasLast = true;

            // Braking point of the CHOSEN maneuver (for viz/telemetry).
            BrakingPointS = -1f;
            try
            {
                if (Chosen.SpeedProfile != null)
                {
                    for (int i = 1; i < Chosen.SpeedProfile.Count; i++)
                    {
                        if (Chosen.SpeedProfile[i] < egoSpeed - 0.75f &&
                            (Chosen.SpeedProfile[0] < egoSpeed - 0.5f))
                        {
                            BrakingPointS = Chosen.StationS[i];
                            break;
                        }
                    }
                }
            }
            catch { }

            return Chosen;
        }

        private static float TacticalCap(Tactics.TacticalMode mode, Perception perception,
            RaceRoute route, RoadCorridor corridor, float cruise, float egoSpeed)
        {
            switch (mode)
            {
                case Tactics.TacticalMode.Follow:
                    try
                    {
                        TrackedActor lead;
                        float egoS = route != null ? route.AlongS : 0f;
                        if (perception.TryGetLeadOnRoute(out lead, egoS, corridor, 45f))
                            return Math.Min(cruise, Math.Max(lead.SpeedAlong + 1.5f, 6f));
                        else if (perception.TryGetClosestThreat(out lead) && lead.IsAhead && lead.Dist < 45f)
                            return Math.Min(cruise, Math.Max(lead.Speed + 1.5f, 6f));
                    }
                    catch { }
                    return cruise;
                case Tactics.TacticalMode.CornerPrep:
                    return cruise; // curvature profile already caps
                case Tactics.TacticalMode.Commit:
                case Tactics.TacticalMode.Defend:
                case Tactics.TacticalMode.Cruise:
                case Tactics.TacticalMode.AttackSetup:
                case Tactics.TacticalMode.OvertakeLeft:
                case Tactics.TacticalMode.OvertakeRight:
                    return cruise;
                case Tactics.TacticalMode.Abort:
                case Tactics.TacticalMode.SideBySide:
                    return Math.Min(cruise, Math.Max(egoSpeed - 1f, 8f));
                case Tactics.TacticalMode.Recovery:
                case Tactics.TacticalMode.Crashed:
                    return Math.Min(cruise, 11f);
                default:
                    return cruise;
            }
        }
    }
}
