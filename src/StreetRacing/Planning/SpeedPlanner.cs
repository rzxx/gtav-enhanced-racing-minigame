using System;

namespace StreetRacing
{
    /// Speed planning from curvature + grip + obstacles + tactics.
    /// v = min(cruise, curve limit, obstacle limit, tactical limit).
    /// All limits derive from the measured capability (brake / lateral g),
    /// scaled by the driver profile — no hardcoded vehicle classes.
    internal sealed class SpeedPlanner
    {
        public float TargetSpeed;
        public string Limiting = "Cruise";
        public float CurveLimit = 99f;
        public float ObstacleLimit = 99f;
        public float RequiredDecel;  // + = need to slow (m/s^2)
        public float BrakingNeed;    // 0..1

        public float Plan(
            float cruise,
            float egoSpeed,
            RaceRoute route,
            RoadCorridor corridor,
            Perception perception,
            VehicleCapability cap,
            DriverProfile profile,
            Tactics.TacticalMode mode,
            float lookaheadM)
        {
            float aLat = cap.UsableLat(profile.GripFactor) * profile.CornerCaution;
            float aBrake = cap.UsableBrake(profile.GripFactor);

            // --- Curve limit: check several stations ahead, each enforced via
            // braking-distance feasibility from current speed.
            CurveLimit = cruise;
            float[] stations = { 40f, 80f, 120f };
            foreach (float s in stations)
            {
                if (s > lookaheadM + 40f) continue;
                float kappa = route.CurvatureAhead(s);
                if (kappa < 1e-5f) continue;
                float vAllow = (float)Math.Sqrt(aLat / kappa);
                // Must be able to reach vAllow within distance s (with margin).
                float vFeasible = (float)Math.Sqrt(vAllow * vAllow + 2f * aBrake * Math.Max(s - 8f, 0f));
                float cand = Math.Min(vAllow, vFeasible);
                if (cand < CurveLimit) CurveLimit = cand;
            }
            CurveLimit = Math.Min(CurveLimit, cruise);

            // --- Obstacle limit: worst actor ahead by stopping-distance logic.
            ObstacleLimit = cruise;
            foreach (var a in perception.Actors)
            {
                if (!a.IsAhead) continue;
                if (Math.Abs(a.Lateral) > 5.5f) continue; // different lane, planner handles laterally
                if (a.Dist > lookaheadM + 60f) continue;
                float gapNeed = profile.SafetyMarginM + egoSpeed * profile.SafetyTimeS * 0.5f;
                float avail = a.Dist - gapNeed;
                if (avail < 0f) avail = 0f;
                // Speed that still stops before the actor (constant-vel prediction).
                float actorSpeedAlong = Math.Max(0f, a.Speed * 0.7f);
                float vStop = (float)Math.Sqrt(actorSpeedAlong * actorSpeedAlong + 2f * aBrake * avail);
                // TTC guard: imminent (<1.6 s) threats clamp hard.
                if (a.Ttc < 1.6f && a.ClosingSpeed > 2f)
                    vStop = Math.Min(vStop, Math.Max(0f, egoSpeed - aBrake * 0.8f));
                else if (a.Ttc < 2.8f && a.ClosingSpeed > 2f)
                    vStop = Math.Min(vStop, Math.Max(actorSpeedAlong, egoSpeed - aBrake * 0.35f));
                if (vStop < ObstacleLimit) ObstacleLimit = vStop;
            }

            // --- Tactical limit.
            float tactical = cruise;
            string tacReason = "Cruise";
            switch (mode)
            {
                case Tactics.TacticalMode.Follow:
                    // Match the leader ahead: never faster than leader + small.
                    tactical = cruise;
                    tacReason = "Follow";
                    if (perception.TryGetClosestThreat(out var lead) && lead.IsAhead && lead.Dist < 45f)
                        tactical = Math.Min(cruise, Math.Max(lead.Speed + 1.5f, 6f));
                    break;
                case Tactics.TacticalMode.CornerPrep:
                    tactical = Math.Min(cruise, CurveLimit);
                    tacReason = "CornerPrep";
                    break;
                case Tactics.TacticalMode.Commit:
                    tactical = cruise; // committed overtake: use everything
                    tacReason = "Commit";
                    break;
                case Tactics.TacticalMode.Abort:
                case Tactics.TacticalMode.SideBySide:
                    tactical = Math.Min(cruise, Math.Max(egoSpeed - 1f, 8f));
                    tacReason = mode.ToString();
                    break;
                case Tactics.TacticalMode.Recovery:
                case Tactics.TacticalMode.Crashed:
                    tactical = 11f;
                    tacReason = "Recovery";
                    break;
                case Tactics.TacticalMode.Defend:
                    tactical = cruise;
                    tacReason = "Defend";
                    break;
            }

            float target = cruise;
            Limiting = "Cruise";
            if (CurveLimit < target) { target = CurveLimit; Limiting = "Curvature"; }
            if (ObstacleLimit < target) { target = ObstacleLimit; Limiting = "Obstacle"; }
            if (tactical < target) { target = tactical; Limiting = tacReason; }
            if (mode == Tactics.TacticalMode.CornerPrep && Limiting == "Cruise") Limiting = "CornerPrep";

            target = Math.Max(0f, target);
            TargetSpeed = target;

            // Braking need for telemetry + actuator urgency.
            float dv = egoSpeed - target;
            RequiredDecel = dv <= 0f ? 0f : (dv * dv) / Math.Max(2f * Math.Max(lookaheadM * 0.6f, 15f), 1f);
            BrakingNeed = RaceMath.Clamp(RequiredDecel / Math.Max(aBrake, 1f), 0f, 1f);
            return target;
        }
    }
}
