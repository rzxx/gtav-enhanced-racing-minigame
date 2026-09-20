using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// Per-path speed profiler for the JOINT maneuver planner.
    ///
    /// The old global planner computed one corridor-wide obstacle speed
    /// (worst actor anywhere in the lane) and applied it after trajectory
    /// selection. A path that cleanly avoided an actor was still braked for
    /// it — the architectural deadlock this pass removes.
    ///
    /// Now: for a GIVEN path, constrain speed only for actors that actually
    /// conflict with that path's swept envelope at the predicted arrival
    /// times, then run the backwards braking pass. Each candidate gets its
    /// own physically executable speed profile; the maneuver scorer picks
    /// the best path+speed together.
    ///
    /// No creep heuristic: a stopped maneuver simply scores badly on
    /// progress and loses to an avoiding maneuver. Creep compensated for the
    /// deadlock instead of removing it.
    internal sealed class SpeedPlanner
    {
        // Kept for debug-viz / telemetry compat: mirrors the CHOSEN
        // maneuver's profile (set by RaceBrain from the joint planner).
        public float TargetSpeed;
        public string Limiting = "Cruise";
        public float CurveLimit = 99f;
        public float ObstacleLimit = 99f;
        public float RequiredDecel;
        public float BrakingNeed;

        public readonly List<float> ProfileS = new List<float>();
        public readonly List<float> ProfileAllowed = new List<float>();
        public readonly List<float> ProfileTarget = new List<float>();
        public float BrakingPointS = -1f;

        public void Reset()
        {
            TargetSpeed = 0f;
            Limiting = "Cruise";
            CurveLimit = 99f;
            ObstacleLimit = 99f;
            RequiredDecel = 0f;
            BrakingNeed = 0f;
            ProfileS.Clear();
            ProfileAllowed.Clear();
            ProfileTarget.Clear();
            BrakingPointS = -1f;
        }

        public const float VehicleHalfWidthM = 1.15f;

        public static float ActorHalfWidth(ActorKind kind)
        {
            switch (kind)
            {
                case ActorKind.Ped: return 0.45f;
                case ActorKind.Debris: return 0.30f;
                case ActorKind.Obstacle: return 0.9f;
                default: return 1.1f;
            }
        }

        public static float ConflictRadius(ActorKind kind)
        {
            // Swept-envelope conflict distance (center to center).
            return VehicleHalfWidthM + ActorHalfWidth(kind) + 0.4f;
        }

        /// Per-path profile: vAllow from curvature, vObs only from actors
        /// whose predicted position at arrival time penetrates THIS path's
        /// envelope, backwards braking pass, forward accel feasibility pass.
        ///
        /// Two complementary conflict tests (both path-specific):
        ///   (a) Euclidean swept envelope: predicted actor center vs path
        ///       station (catches cut-ins, crossing, side swipe);
        ///   (b) Path-frame (route lateral vs this path's lateral + predicted
        ///       longitudinal overlap): catches same-lane following and fast
        ///       head-on where the Euclidean snapshot at 10 m stations can
        ///       straddle the meeting point.
        public static void ProfileForPath(
            IList<Vector3> path,
            IList<float> stationS,
            IList<float> kappa,
            IList<float> pathLats,
            float egoRouteS,
            Perception perception,
            RoadCorridor corridor,
            float egoSpeed,
            float cruise,
            float aLatEff,
            float aBrake,
            float topSpeed,
            DriverProfile profile,
            float tacticalCap,
            out float[] vAllow,
            out float[] vTgt,
            out float[] arrivalT,
            out int constrainHandle,
            out string constrainKind,
            out float constrainS,
            out float minPredClearance)
        {
            int n = path.Count;
            vAllow = new float[n];
            vTgt = new float[n];
            arrivalT = new float[n];
            constrainHandle = -1;
            constrainKind = "";
            constrainS = -1f;
            minPredClearance = 999f;

            float[] vCurve = new float[n];
            for (int i = 0; i < n; i++)
            {
                float k = (kappa != null && i < kappa.Count) ? kappa[i] : 0f;
                float vc;
                if (k < 1e-5f) vc = cruise;
                else
                {
                    vc = (float)Math.Sqrt(aLatEff / k);
                    if (vc > cruise) vc = cruise;
                }
                if (topSpeed > 5f && vc > topSpeed) vc = topSpeed;
                vCurve[i] = vc;
            }

            // Free-flow arrival estimate for conflict detection (uses current
            // speed, clamped — stable across ticks, no circular dependency).
            float[] tFree = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = stationS[i];
                float t = s / Math.Max(egoSpeed, 6f);
                tFree[i] = RaceMath.Clamp(t, 0f, 5f);
            }

            float[] vObs = new float[n];
            for (int i = 0; i < n; i++) vObs[i] = cruise;

            float gapStop = profile.SafetyMarginM + egoSpeed * 0.35f;

            // Per-station conflict test against THIS path only.
            // Off-roadway peds/props can never intersect an on-road path:
            // skip them explicitly (they still constrain off-road paths).
            int[] hitHandle = new int[n];
            string[] hitKind = new string[n];
            for (int i = 0; i < n; i++) { hitHandle[i] = int.MinValue; hitKind[i] = ""; }
            float[] hitFollowV = new float[n];
            for (int i = 0; i < n; i++) hitFollowV[i] = float.MaxValue;

            for (int k = 0; k < n; k++)
            {
                float s = stationS[k];
                Vector3 pk = path[k];
                Vector3 pdir = PathDirAt(path, k);
                float pathLat = (pathLats != null && k < pathLats.Count) ? pathLats[k] : 0f;
                foreach (var a in perception.Actors)
                {
                    // Small debris is deliberately handled as a soft spatial
                    // cost, never as a zero-speed longitudinal blocker.
                    if (a.Kind == ActorKind.Debris) continue;

                    if (a.RouteValid)
                    {
                        if (a.RouteDist < -8f || a.RouteDist > s + 90f) continue;
                        // Actor far behind/ahead of this station in route
                        // distance cannot conflict here (cheap reject). Both
                        // tests below are authoritative within the window.
                        if (Math.Abs(a.RouteDist - s) > 45f) continue;
                    }
                    else
                    {
                        if (!a.IsAhead && a.Dist > 25f) continue;
                        if (a.Dist > 120f) continue;
                    }
                    if (a.OffRoadway)
                    {
                        // Sidewalk rule: far off-roadway clutter never
                        // influences any path; near off-roadway actors only
                        // matter when the path actually goes near them
                        // (radius test below decides).
                        if (a.Dist > 12f) continue;
                    }

                    // Every Join/path candidate shares the immutable ego
                    // pose at station zero. A side-by-side or rear actor is not
                    // a longitudinal blocker merely because its inflated
                    // footprint overlaps the origin. This is the same semantic
                    // rule used by SpatialPlannerV1.
                    if (s < 3.0f)
                    {
                        float latReach = VehicleHalfWidthM + ActorHalfWidth(a.Kind) + 0.55f;
                        bool genuinelyAhead = a.Longitudinal > 0.75f
                            && Math.Abs(a.Lateral) < latReach;
                        if (!genuinelyAhead)
                            continue;
                    }

                    // Defensive self-zone guard: an actor coincident with ego
                    // at path station s=0 cannot create a stop constraint
                    // unless it is a real external collision threat.
                    // Source bug was the AI's own driver at ego center
                    // (Dist~0, RouteDist~0, clearance 0-(1.15+0.45)=-1.6m).
                    // A false hit at s=0 gives earliestStopS=0-gapStop<0,
                    // which via propagation (stationS>=earliestStopS for all)
                    // zeros vObs[*] and via vTgt[0]=0 zeros target speed.
                    // Primary fix is Perception exclusion (IsInVehicle+handle
                    // with track purge); this guard ensures any residual
                    // coincident track cannot zero the plan. Tight 1.2m/1.5m
                    // radius preserves real bumper blockers (d~2m+).
                    if (s < 2.5f && a.Dist < 1.2f)
                    {
                        bool routeCoincident = !a.RouteValid || Math.Abs(a.RouteDist) < 1.5f;
                        if (routeCoincident && a.Kind != ActorKind.TrafficVehicle && a.Kind != ActorKind.Rival)
                        {
                            bool fastIndependent = a.ClosingSpeed > 4f && Math.Abs(a.Speed - egoSpeed) > 4f;
                            if (!fastIndependent) continue;
                        }
                    }

                    bool conflict = false;
                    float followV = 0f;

                    // (a) Euclidean swept envelope at arrival time.
                    Vector3 pred;
                    try { pred = perception.Predict(a, tFree[k]); }
                    catch { pred = a.Position; }
                    float d = RaceMath.FlatDistance(pk, pred);
                    float clearance = d - (VehicleHalfWidthM + ActorHalfWidth(a.Kind));
                    if (clearance < minPredClearance) minPredClearance = clearance;
                    float confR = ConflictRadius(a.Kind);
                    if (d < confR)
                    {
                        float along = RaceMath.FlatDot(new Vector3(a.Velocity.X, a.Velocity.Y, 0f), pdir);
                        followV = along > 2f ? along : 0f;
                        conflict = true;
                    }

                    // (b) Path-frame overlap: same-lane following + fast
                    // head-on that a 10 m Euclidean snapshot can straddle.
                    // Predicted actor route position vs this path station.
                    if (!conflict && a.RouteValid)
                    {
                        float predRouteS = a.RouteS + a.SpeedAlong * tFree[k];
                        float egoRouteAtStation = egoRouteS + s;
                        float longGap = predRouteS - egoRouteAtStation;
                        float latGap = Math.Abs(a.RouteLateral - pathLat);
                        float latTol = VehicleHalfWidthM + ActorHalfWidth(a.Kind) - 0.15f;
                        if (latTol < 1.4f) latTol = 1.4f;
                        // Longitudinal window covers car length + one station
                        // step so meetings between stations still bind.
                        if (Math.Abs(longGap) < 8f && latGap < latTol)
                        {
                            followV = a.SpeedAlong > 2f ? a.SpeedAlong : 0f;
                            conflict = true;
                            float pc = (float)Math.Sqrt(longGap * longGap + latGap * latGap)
                                - (VehicleHalfWidthM + ActorHalfWidth(a.Kind));
                            if (pc < minPredClearance) minPredClearance = pc;
                        }
                    }

                    if (conflict)
                    {
                        // Keep the most restrictive (lowest) at this station.
                        if (hitKind[k] == "" || followV < hitFollowV[k])
                        {
                            hitHandle[k] = a.Handle;
                            hitKind[k] = a.Kind.ToString();
                            hitFollowV[k] = followV;
                            if (followV < vObs[k]) vObs[k] = followV;
                        }
                    }
                }
            }

            // Gap handling: a stop conflict at s_c forbids everything beyond
            // s_c - gap (must stop BEFORE the envelope, not inside it).
            // NOTE: a false conflict at s=0 produces earliestStopS<0 and via
            // the loop below zeros the ENTIRE maneuver (all stationS>=neg).
            // That is why the s=0 self-zone guard above must run BEFORE any
            // hit is recorded — fix the source, never add creep around it.
            float earliestStopS = float.MaxValue;
            for (int k = 0; k < n; k++)
            {
                if (hitKind[k] != "" && hitFollowV[k] < 0.5f)
                {
                    float sc = stationS[k] - gapStop;
                    if (sc < earliestStopS) earliestStopS = sc;
                }
            }

            bool[] isHit = new bool[n];
            for (int k = 0; k < n; k++)
                isHit[k] = hitKind[k] != "" && vObs[k] < cruise - 0.01f;

            if (earliestStopS < float.MaxValue)
            {
                for (int k = 0; k < n; k++)
                {
                    if (stationS[k] >= earliestStopS && vObs[k] > 0f)
                    {
                        vObs[k] = 0f;
                        if (!isHit[k])
                        {
                            isHit[k] = true;
                            // Attribute the propagated stop to the earliest
                            // conflicting actor for telemetry.
                            for (int j = 0; j < n; j++)
                            {
                                if (hitKind[j] != "" && hitFollowV[j] < 0.5f &&
                                    Math.Abs((stationS[j] - gapStop) - earliestStopS) < 11f)
                                {
                                    hitHandle[k] = hitHandle[j];
                                    hitKind[k] = hitKind[j];
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                // Following gaps: soften the station just before a follow
                // constraint so we don't tailgate (small, no tuning).
                for (int k = 0; k < n; k++)
                {
                    if (isHit[k] && hitFollowV[k] >= 0.5f)
                    {
                        // No forward propagation for following; the backwards
                        // pass handles the braking distance.
                    }
                }
            }

            for (int i = 0; i < n; i++)
            {
                float va = vCurve[i];
                if (vObs[i] < va) va = vObs[i];
                if (tacticalCap < va) va = tacticalCap;
                vAllow[i] = Math.Max(0f, va);
            }

            // Backwards braking pass.
            vTgt[n - 1] = vAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float ds = Math.Max(1f, stationS[i + 1] - stationS[i]);
                float vReach = (float)Math.Sqrt(vTgt[i + 1] * vTgt[i + 1] + 2f * aBrake * ds);
                vTgt[i] = Math.Min(vAllow[i], vReach);
            }
            // Forward accel feasibility (cannot teleport to speed).
            // If we must brake from egoSpeed, vFwd[0] stays at vTgt[0] (the
            // braking pass is already feasible); the forward pass only limits
            // acceleration toward distant fast stations.
            float aAccel = Math.Max(2f, aBrake * 0.55f);
            float[] vFwd = new float[n];
            vFwd[0] = vTgt[0] > egoSpeed ? egoSpeed : vTgt[0];
            if (vTgt[0] <= egoSpeed) vFwd[0] = vTgt[0];
            else vFwd[0] = Math.Min(vTgt[0], egoSpeed + 1f);
            for (int i = 1; i < n; i++)
            {
                float ds = Math.Max(1f, stationS[i] - stationS[i - 1]);
                float vReach = (float)Math.Sqrt(vFwd[i - 1] * vFwd[i - 1] + 2f * aAccel * ds);
                vFwd[i] = Math.Min(vTgt[i], vReach);
            }
            for (int i = 0; i < n; i++) vTgt[i] = vFwd[i];

            // Final arrival times from the executable profile.
            arrivalT[0] = 0f;
            for (int i = 1; i < n; i++)
            {
                float ds = Math.Max(0.5f, stationS[i] - stationS[i - 1]);
                float vAvg = (vTgt[i - 1] + vTgt[i]) * 0.5f;
                if (vAvg < 1.2f) vAvg = 1.2f;
                arrivalT[i] = arrivalT[i - 1] + ds / vAvg;
            }

            // Recompute clearance at EXECUTABLE arrival times for scoring.
            // Keep the conflict-time minimum too: a head-on blocked between
            // 10 m stations can look Euclidean-far at executable times while
            // the path-frame test correctly stopped for it.
            float minConflict = minPredClearance;
            minPredClearance = 999f;
            foreach (var a in perception.Actors)
            {
                for (int k = 0; k < n; k++)
                {
                    // Same self-zone guard as the conflict pass: a coincident
                    // occupant echo at s=0 must not veto scoring via clearance.
                    // Real bumper blockers (d~2m+) are outside the 1.2m radius
                    // and still veto correctly.
                    try
                    {
                        float sk = stationS[k];
                        if (sk < 2.5f && a.Dist < 1.2f)
                        {
                            bool rc = !a.RouteValid || Math.Abs(a.RouteDist) < 1.5f;
                            if (rc && a.Kind != ActorKind.TrafficVehicle && a.Kind != ActorKind.Rival)
                            {
                                bool fastInd = a.ClosingSpeed > 4f && Math.Abs(a.Speed - egoSpeed) > 4f;
                                if (!fastInd) continue;
                            }
                        }
                    }
                    catch { }
                    Vector3 pred;
                    try { pred = perception.Predict(a, arrivalT[k]); }
                    catch { pred = a.Position; }
                    float d = RaceMath.FlatDistance(path[k], pred);
                    float c = d - (VehicleHalfWidthM + ActorHalfWidth(a.Kind));
                    if (c < minPredClearance) minPredClearance = c;
                }
            }

            // Constraining actor: earliest station where obstacle binds.
            if (minConflict < minPredClearance) minPredClearance = minConflict;
            for (int k = 0; k < n; k++)
            {
                if (isHit[k] && vObs[k] < vCurve[k] - 0.01f)
                {
                    constrainHandle = hitHandle[k];
                    constrainKind = hitKind[k];
                    constrainS = stationS[k];
                    break;
                }
            }
            // gapFollow documents follow-gap intent (no forward stop needed;
            // the backwards pass handles braking distance for following).
        }

        private static Vector3 PathDirAt(IList<Vector3> path, int k)
        {
            try
            {
                Vector3 d;
                if (k <= 0)
                    d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, 0f);
                else if (k >= path.Count - 1)
                    d = new Vector3(path[k].X - path[k - 1].X, path[k].Y - path[k - 1].Y, 0f);
                else
                    d = new Vector3(path[k + 1].X - path[k - 1].X, path[k + 1].Y - path[k - 1].Y, 0f);
                if (RaceMath.FlatLength(d) < 0.3f) return new Vector3(0f, 1f, 0f);
                return RaceMath.FlatNormalize(d);
            }
            catch { return new Vector3(0f, 1f, 0f); }
        }

        /// Legacy global planner: RETIRED. Kept compiling for tooling; the
        /// joint planner (ProfileForPath per candidate) is the only speed
        /// authority. This shim returns a curvature-only profile with NO
        /// obstacle logic and NO creep, so any accidental caller cannot
        /// reintroduce the corridor-wide deadlock.
        [System.Obsolete("Use ProfileForPath per candidate (joint maneuver planner).")]
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
            float cc = profile.CornerCaution;
            if (cc < 0.5f) cc = 0.5f;
            if (cc > 2f) cc = 2f;
            float aLatEff = aLatRaw / (cc * cc);
            float aBrake = cap.UsableBrake(profile.GripFactor);

            ProfileS.Clear();
            ProfileAllowed.Clear();
            ProfileTarget.Clear();
            BrakingPointS = -1f;

            float horizon = Math.Min(lookaheadM + 80f, 230f);
            int n = Math.Max(5, (int)(horizon / 10f) + 1);
            if (n > 24) n = 24;
            float[] vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = i * 10f;
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
                try { if (cap.TopSpeedEst > 5f && va > cap.TopSpeedEst) va = cap.TopSpeedEst; }
                catch { }
                vAllow[i] = va;
                ProfileAllowed.Add(va);
            }
            float[] vTgt = new float[n];
            vTgt[n - 1] = vAllow[n - 1];
            for (int i = n - 2; i >= 0; i--)
            {
                float vReach = (float)Math.Sqrt(vTgt[i + 1] * vTgt[i + 1] + 2f * aBrake * 10f);
                vTgt[i] = Math.Min(vAllow[i], vReach);
            }
            ProfileTarget.Clear();
            for (int i = 0; i < n; i++) ProfileTarget.Add(vTgt[i]);

            CurveLimit = vTgt[0];
            if (CurveLimit > cruise) CurveLimit = cruise;
            ObstacleLimit = cruise;

            float tactical = cruise;
            string tacReason = "Cruise";
            switch (mode)
            {
                case Tactics.TacticalMode.Follow:
                    tactical = cruise; tacReason = "Follow";
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
                    tactical = Math.Min(cruise, CurveLimit); tacReason = "CornerPrep"; break;
                case Tactics.TacticalMode.Commit:
                    tactical = cruise; tacReason = "Commit"; break;
                case Tactics.TacticalMode.Abort:
                case Tactics.TacticalMode.SideBySide:
                    tactical = Math.Min(cruise, Math.Max(egoSpeed - 1f, 8f)); tacReason = mode.ToString(); break;
                case Tactics.TacticalMode.Recovery:
                case Tactics.TacticalMode.Crashed:
                    tactical = 11f; tacReason = "Recovery"; break;
                case Tactics.TacticalMode.Defend:
                    tactical = cruise; tacReason = "Defend"; break;
            }

            float target = cruise;
            Limiting = "Cruise";
            if (CurveLimit < target) { target = CurveLimit; Limiting = "Curvature"; }
            if (tactical < target) { target = tactical; Limiting = tacReason; }
            if (mode == Tactics.TacticalMode.CornerPrep && Limiting == "Cruise") Limiting = "CornerPrep";

            target = Math.Max(0f, target);
            TargetSpeed = target;
            float dv = egoSpeed - target;
            RequiredDecel = dv <= 0f ? 0f : (dv * dv) / Math.Max(2f * Math.Max(lookaheadM * 0.6f, 15f), 1f);
            BrakingNeed = RaceMath.Clamp(RequiredDecel / Math.Max(aBrake, 1f), 0f, 1f);
            return target;
        }
    }
}
