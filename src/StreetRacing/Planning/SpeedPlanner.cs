using System;
using System.Collections.Generic;

namespace StreetRacing
{
    /// Speed planning from curvature + grip + obstacles + tactics.
    /// v = min(cruise, curve limit, obstacle limit, tactical limit).
    /// All limits derive from the measured capability (brake / lateral g),
    /// scaled by the driver profile — no hardcoded vehicle classes.
    ///
    /// Corner profile: local curvature is sampled every ~10 m out to
    /// lookahead+80 m (vAllow(s) = sqrt(aLatEff/kappa)/CornerCaution) and a
    /// backwards braking pass enforces
    ///   v[i] = min(vAllow[i], sqrt(v[i+1]^2 + 2*aBrake*ds))
    /// so a slow corner 120 m ahead constrains current speed through braking
    /// distance — not just the corner you are already in.
    ///
    /// CORNER-CAUTION SEMANTICS (fixed): CornerCaution > 1 = more cautious =
    /// slower (vAllow divided by it). Cautious=1.15 slows ~13%, Aggressive=
    /// 0.92 quickens ~9%. The old code multiplied grip by it, which inverted
    /// the meaning (Cautious was faster). Profile numbers are unchanged; the
    /// formula now matches their intent.
    ///
    /// Obstacle speeds use projected actor velocity along the route
    /// (SpeedAlong), not actor.Speed*0.7. Oncoming traffic projects negative
    /// and is clamped to 0 for stopping logic (assume worst); same-direction
    /// traffic projects positive and correctly raises the follow speed.
    internal sealed class SpeedPlanner
    {
        public float TargetSpeed;
        public string Limiting = "Cruise";
        public float CurveLimit = 99f;
        public float ObstacleLimit = 99f;
        public float RequiredDecel;  // + = need to slow (m/s^2)
        public float BrakingNeed;    // 0..1

        // Full profile for debug viz + telemetry (s ahead, m).
        public readonly List<float> ProfileS = new List<float>();
        public readonly List<float> ProfileAllowed = new List<float>();
        public readonly List<float> ProfileTarget = new List<float>();
        public float BrakingPointS = -1f;

        private const float StationDs = 10f;

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
            float aLatRaw = cap.UsableLat(profile.GripFactor);
            // Fixed semantics: divide, don't multiply.
            float cc = profile.CornerCaution;
            if (cc < 0.5f) cc = 0.5f;
            if (cc > 2f) cc = 2f;
            float aLatEff = aLatRaw / (cc * cc);
            float aBrake = cap.UsableBrake(profile.GripFactor);

            ProfileS.Clear();
            ProfileAllowed.Clear();
            ProfileTarget.Clear();
            BrakingPointS = -1f;

            // --- Curve profile with backwards braking pass.
            float horizon = Math.Min(lookaheadM + 80f, 230f);
            int n = Math.Max(5, (int)(horizon / StationDs) + 1);
            if (n > 24) n = 24;
            float[] vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = i * StationDs;
                ProfileS.Add(s);
                float kappa = 0f;
                try { kappa = route.CurvatureAtS(route.AlongS + s, 20f); }
                catch { kappa = 0f; }
                float va;
                if (kappa < 1e-5f) va = cruise;
                else
                {
                    va = (float)Math.Sqrt(aLatEff / kappa);
                    if (va > cruise) va = cruise;
                }
                // Capability top speed caps everything (DriveV AI caps aside).
                try { if (cap.TopSpeedEst > 5f && va > cap.TopSpeedEst) va = cap.TopSpeedEst; }
                catch { }
                vAllow[i] = va;
                ProfileAllowed.Add(va);
            }
            float[] vTgt = new float[n];
            vTgt[n - 1] = vAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float vReach = (float)Math.Sqrt(vTgt[i + 1] * vTgt[i + 1] + 2f * aBrake * StationDs);
                vTgt[i] = Math.Min(vAllow[i], vReach);
                ProfileTarget.Add(0f); // placeholder, filled below in order
            }
            ProfileTarget.Clear();
            for (int i = 0; i < n; i++) ProfileTarget.Add(vTgt[i]);

            CurveLimit = vTgt[0];
            if (CurveLimit > cruise) CurveLimit = cruise;

            // First station ahead that forces braking from current speed.
            for (int i = 1; i < n; i++)
            {
                if (vTgt[i] < egoSpeed - 0.75f)
                {
                    // Only report it if we are currently faster than allowed
                    // here OR will need to shed speed to make it.
                    if (vTgt[0] < egoSpeed - 0.5f || vAllow[i] < egoSpeed - 1f)
                    {
                        BrakingPointS = i * StationDs;
                        break;
                    }
                }
            }

            // --- Obstacle limit: worst actor ahead by stopping-distance logic,
            // in route coordinates with projected along-route speed.
            ObstacleLimit = cruise;
            foreach (var a in perception.Actors)
            {
                float routeDist;
                float routeLat;
                float actorAlong;
                float ttc;
                bool usable = false;
                if (a.RouteValid)
                {
                    routeDist = a.RouteDist;
                    routeLat = a.RouteLateral;
                    actorAlong = Math.Max(0f, a.SpeedAlong);
                    ttc = Math.Min(a.Ttc, a.RouteTtc);
                    usable = true;
                }
                else
                {
                    if (!a.IsAhead) continue;
                    if (Math.Abs(a.Lateral) > 5.5f) continue;
                    routeDist = a.Dist;
                    routeLat = a.Lateral;
                    // Fallback projection onto ego forward when route is lost.
                    actorAlong = 0f;
                    try
                    {
                        // a.Velocity is flat; ego fwd approx from closing geometry.
                        // Conservative: treat non-route actors as static unless
                        // clearly receding (closing<0) — never invent speed.
                        if (a.ClosingSpeed < -1f) actorAlong = Math.Max(0f, egoSpeed + a.ClosingSpeed);
                    }
                    catch { }
                    ttc = a.Ttc;
                    usable = true;
                }
                if (!usable) continue;
                if (routeDist < -3f || routeDist > lookaheadM + 60f) continue;
                float halfAt = corridor != null ? corridor.HalfWidthAt(Math.Max(0f, routeDist)) : 7f;
                if (Math.Abs(routeLat) > halfAt + 2f) continue; // different lane, planner handles laterally
                float gapNeed = profile.SafetyMarginM + egoSpeed * profile.SafetyTimeS * 0.5f;
                float avail = routeDist - gapNeed;
                if (avail < 0f) avail = 0f;
                float vStop = (float)Math.Sqrt(actorAlong * actorAlong + 2f * aBrake * avail);
                // TTC guard: imminent (<1.6 s) threats clamp hard.
                float closingForTtc = a.RouteValid ? a.ClosingAlong : a.ClosingSpeed;
                if (ttc < 1.6f && closingForTtc > 2f)
                    vStop = Math.Min(vStop, Math.Max(0f, egoSpeed - aBrake * 0.8f));
                else if (ttc < 2.8f && closingForTtc > 2f)
                    vStop = Math.Min(vStop, Math.Max(actorAlong, egoSpeed - aBrake * 0.35f));
                if (vStop < ObstacleLimit) ObstacleLimit = vStop;
            }

            // --- Anti-deadlock creep: a full stop behind a static non-ped
            // blocker (parked car, prop, stopped rival) with nobody touching
            // us must not pin v_tgt at 0 forever. The trajectory scorer steers
            // around when width allows; creeping at 2.5 m/s lets the servo
            // close the gap and either pass or stop again at the 3.5 m gate.
            // Peds veto creep entirely — never nudge into pedestrians.
            bool creeping = false;
            if (egoSpeed < 1.5f && ObstacleLimit < 0.5f)
            {
                bool pedNear = false;
                float nearestBlocker = float.MaxValue;
                foreach (var a in perception.Actors)
                {
                    float rd = a.RouteValid ? a.RouteDist : (a.IsAhead ? a.Dist : 999f);
                    float rl = a.RouteValid ? a.RouteLateral : a.Lateral;
                    if (rd < -2f || rd > 14f) continue;
                    float halfAt = corridor != null ? corridor.HalfWidthAt(Math.Max(0f, rd)) : 7f;
                    if (Math.Abs(rl) > halfAt + 2f) continue;
                    if (a.Kind == ActorKind.Ped && a.Dist < 14f) { pedNear = true; break; }
                    if (a.Dist < nearestBlocker) nearestBlocker = a.Dist;
                }
                if (!pedNear && nearestBlocker > 3.5f && nearestBlocker < 200f)
                    creeping = true;
            }

            // --- Tactical limit.
            float tactical = cruise;
            string tacReason = "Cruise";
            switch (mode)
            {
                case Tactics.TacticalMode.Follow:
                    // Match the leader ahead on the route: never faster than
                    // leader's along-route speed + small.
                    tactical = cruise;
                    tacReason = "Follow";
                    try
                    {
                        TrackedActor lead;
                        float egoS = route != null ? route.AlongS : 0f;
                        if (perception.TryGetLeadOnRoute(out lead, egoS, corridor, 45f))
                            tactical = Math.Min(cruise, Math.Max(lead.SpeedAlong + 1.5f, 6f));
                        else if (perception.TryGetClosestThreat(out lead) && lead.IsAhead && lead.Dist < 45f)
                            tactical = Math.Min(cruise, Math.Max(lead.Speed + 1.5f, 6f));
                    }
                    catch { }
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
            if (creeping && target < 2.5f) { target = 2.5f; Limiting = "Creep"; }

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
