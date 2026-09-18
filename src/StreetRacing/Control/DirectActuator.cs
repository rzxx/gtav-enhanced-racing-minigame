using System;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing.Control
{
    /// Intended controller: executes the joint planner's sampled path + speed
    /// profile directly (steering/throttle/brake), with no Rockstar
    /// pathfinding in the loop. The planner ticks at ~10 Hz; this controller
    /// runs every script tick (~20 Hz) against the latest maneuver:
    ///   - local speed-dependent lookahead point on the SELECTED path;
    ///   - pure-pursuit steering to that point (cross-track + heading error);
    ///   - PI longitudinal tracking of the LOCAL planned speed.
    ///
    /// COLLAPSED rules (Phases 1-3):
    ///   - Reverse is EXPLICIT via ManeuverCommand.Reverse (recovery primitive
    ///     only). Direct never infers reverse from heading error, even at
    ///     >130 deg with zero speed: a zero-speed maneuver must HOLD.
    ///   - Longitudinal uses BOTH Vehicle.Throttle and Vehicle.ThrottlePower.
    ///     Legacy code set Throttle only; DriveV hardware diagnosis (DirectDiag)
    ///     showed some cars need ThrottlePower to physically accelerate. Both
    ///     are now commanded together and both are logged by the diag brain.
    ///
    /// GtaDriverActuator remains only as an emergency low-speed rejoin tool.
    /// Direct is the default isolation test AND the intended actuator.
    internal sealed class DirectActuator : IVehicleActuator
    {
        private Ped driver;
        private Vehicle vehicle;
        private int stuckMs;
        private int refreshMs;

        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount { get; private set; }
        public string LastReason { get; private set; } = "";
        public string ActuatorName => "Direct";
        public PathFollowingError LastError { get; private set; } = new PathFollowingError();

        private ManeuverCommand cmd;
        private bool hasManeuver;
        private int planId;

        // Longitudinal state (simple PI + anti-windup via clamp).
        private float speedInt;
        private float lastSteer;
        private float lastThr;
        private float lastBrk;

        public void Attach(Ped driver, Vehicle vehicle, float cruise, int style, int refreshMs, int stuckMs)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.CurrentCruise = cruise;
            this.CurrentStyle = style;
            this.refreshMs = refreshMs;
            this.stuckMs = stuckMs;

            driver.IsPersistent = true;
            vehicle.IsPersistent = true;

            Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, true);
            Function.Call(Hash.SET_PED_KEEP_TASK, driver, true);
            Function.Call(Hash.SET_DRIVER_ABILITY, driver, 1.0f);
            Function.Call(Hash.SET_DRIVER_AGGRESSIVENESS, driver, 1.0f);
            Function.Call(Hash.SET_DRIVER_RACING_MODIFIER, driver, 1.0f);

            try { driver.Task.ClearAll(); } catch { }
            HasPlan = false;
            hasManeuver = false;
            ReissueCount = 0;
            planId = 0;
            cmd = new ManeuverCommand();
            LastError = new PathFollowingError { Valid = false };
            speedInt = 0f;
            lastSteer = 0f;
            lastThr = 0f;
            lastBrk = 0f;
        }

        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = Math.Max(0f, targetSpeed);
            CurrentStyle = style;
            LastReason = reason ?? "";
            HasPlan = true;
            // Legacy seam: single-point maneuver at constant speed.
            try
            {
                Vector3 ego = vehicle != null && vehicle.Exists() ? vehicle.Position : aimPoint;
                cmd = new ManeuverCommand
                {
                    Path = new System.Collections.Generic.List<Vector3> { ego, aimPoint },
                    StationS = new System.Collections.Generic.List<float> { 0f, RaceMath.FlatDistance(ego, aimPoint) },
                    SpeedProfile = new System.Collections.Generic.List<float> { CurrentCruise, CurrentCruise },
                    AimPoint = aimPoint,
                    TargetSpeed = CurrentCruise,
                    Style = style,
                    Reason = LastReason,
                    PlanId = planId,
                };
                hasManeuver = cmd.Path.Count >= 2;
            }
            catch { hasManeuver = false; }
        }

        public void SetManeuver(ManeuverCommand c)
        {
            cmd = c;
            planId = c.PlanId;
            hasManeuver = c.Path != null && c.Path.Count >= 2;
            HasPlan = hasManeuver;
            try
            {
                CurrentAim = c.AimPoint;
                CurrentCruise = Math.Max(0f, c.TargetSpeed);
                CurrentStyle = c.Style;
                LastReason = c.Reason ?? "";
            }
            catch { }
        }

        public void Clear()
        {
            HasPlan = false;
            hasManeuver = false;
            try
            {
                if (vehicle != null && vehicle.Exists())
                {
                    try { vehicle.Throttle = 0f; } catch { }
                    try { vehicle.ThrottlePower = 0f; } catch { }
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                }
            }
            catch { }
        }

        public bool Valid()
        {
            return driver != null && driver.Exists() && !driver.IsDead
                && vehicle != null && vehicle.Exists() && !vehicle.IsDead;
        }

        public void Stop()
        {
            try
            {
                if (vehicle != null && vehicle.Exists())
                {
                    try { vehicle.Throttle = 0f; } catch { }
                    try { vehicle.ThrottlePower = 0f; } catch { }
                    try { vehicle.BrakePower = 1f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                    vehicle.IsPersistent = false;
                }
                if (driver != null && driver.Exists())
                {
                    try { driver.Task.ClearAll(); } catch { }
                    Function.Call(Hash.SET_PED_KEEP_TASK, driver, false);
                    Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, false);
                    driver.IsPersistent = false;
                }
            }
            catch { }
        }

        public bool OnTick(bool forceReissue, string tacticalName)
        {
            if (!HasPlan || !Valid() || !hasManeuver) return false;
            Vector3 egoPos;
            float egoHeading;
            float egoSpeed;
            try
            {
                egoPos = vehicle.Position;
                egoHeading = vehicle.Heading;
                egoSpeed = vehicle.Speed;
            }
            catch { return false; }

            // --- Locate ego on the selected path.
            float sEgo;
            float crossTrack;
            float pathHeading;
            Vector3 closest;
            try
            {
                ClosestOnPath(egoPos, cmd.Path, out sEgo, out crossTrack, out pathHeading, out closest);
            }
            catch { return false; }

            // Local speed-dependent lookahead on the SELECTED path.
            float ld = 6f + egoSpeed * 0.7f;
            ld = RaceMath.Clamp(ld, 8f, 28f);
            Vector3 lookPt;
            float lookS;
            float lookSpeed;
            try
            {
                lookS = sEgo + ld;
                lookPt = PointAtS(cmd.Path, StationSToCumulative(cmd), lookS);
                lookSpeed = SpeedAtS(cmd, sEgo);
            }
            catch
            {
                lookPt = CurrentAim;
                lookS = sEgo + ld;
                lookSpeed = CurrentCruise;
            }

            // --- Lateral: pure pursuit to the local lookahead point.
            var to = new Vector3(lookPt.X - egoPos.X, lookPt.Y - egoPos.Y, 0f);
            float distToLook = RaceMath.FlatLength(to);
            float desiredHeading = distToLook > 1f
                ? RaceMath.HeadingFromVector(RaceMath.FlatNormalize(to))
                : pathHeading;
            float headErr = RaceMath.HeadingDiffDeg(desiredHeading, egoHeading);

            // Pure-pursuit curvature -> wheel angle (wheelbase ~2.7 m).
            float alphaDeg = headErr;
            float alphaRad = alphaDeg * (float)Math.PI / 180f;
            float wheelbase = 2.7f;
            float steerPursuit = 0f;
            if (distToLook > 1f)
            {
                float kappa = 2f * (float)Math.Sin(alphaRad) / Math.Max(distToLook, 3f);
                steerPursuit = (float)(Math.Atan(wheelbase * kappa) * 180.0 / Math.PI);
            }
            // Blend pursuit angle with heading error for low-speed authority,
            // plus a small cross-track correction so we rejoin after slides.
            float steerDeg = steerPursuit * 1.4f + headErr * 0.35f - crossTrack * 1.1f;
            // Speed-scheduled clamp (authority falls with speed).
            if (egoSpeed > 25f) steerDeg = RaceMath.Clamp(steerDeg, -18f, 18f);
            else if (egoSpeed > 15f) steerDeg = RaceMath.Clamp(steerDeg, -24f, 24f);
            else steerDeg = RaceMath.Clamp(steerDeg, -32f, 32f);

            // Explicit reverse ONLY: commanded by the recovery primitive via
            // ManeuverCommand.Reverse. Never infer from heading error here —
            // a maneuver that commanded zero speed must hold position, not
            // back into traffic because the route is sideways.
            bool reversing = false;
            try { reversing = cmd.Reverse; } catch { reversing = false; }
            if (reversing) steerDeg = -steerDeg;

            try { vehicle.SteeringAngle = steerDeg; } catch { }
            try { vehicle.SteeringScale = 1f; } catch { }

            // --- Longitudinal: PI on LOCAL planned speed error.
            float speedErr = lookSpeed - egoSpeed;
            float dt = 0.05f; // 20 Hz script tick
            speedInt = RaceMath.Clamp(speedInt + speedErr * dt, -6f, 6f);
            float u = speedErr * 0.35f + speedInt * 0.12f;

            bool stalled = egoSpeed < 1.5f && lookSpeed > 3f;

            float thr = 0f;
            float brk = 0f;
            try
            {
                if (reversing)
                {
                    // Explicit recovery reverse: controlled, never inferred.
                    try { vehicle.Throttle = -0.6f; } catch { }
                    try { vehicle.ThrottlePower = -0.6f; } catch { }
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                    thr = -0.6f;
                }
                else if (u >= 0f)
                {
                    thr = RaceMath.Clamp(u, stalled ? 0.8f : 0f, 1f);
                    if (stalled && thr < 0.8f) thr = 0.8f;
                    try { vehicle.Throttle = thr; } catch { }
                    try { vehicle.ThrottlePower = thr; } catch { }
                    try { vehicle.BrakePower = 0f; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = false; } catch { }
                }
                else
                {
                    try { vehicle.Throttle = 0f; } catch { }
                    try { vehicle.ThrottlePower = 0f; } catch { }
                    brk = RaceMath.Clamp(-u, 0.15f, 1f);
                    if (lookSpeed < 0.5f && egoSpeed > 1f) brk = 1f;
                    try { vehicle.BrakePower = brk; } catch { }
                    try { vehicle.IsHandbrakeForcedOn = lookSpeed < 0.5f && egoSpeed < 3f; }
                    catch { }
                }
            }
            catch { }

            lastSteer = steerDeg;
            lastThr = thr;
            lastBrk = brk;

            var e = new PathFollowingError
            {
                Valid = true,
                LateralErrM = crossTrack,
                HeadingErrDeg = -headErr,
                SpeedErrMps = speedErr,
                DistToPathM = Math.Abs(crossTrack),
                SteerDeg = steerDeg,
                Throttle01 = thr,
                Brake01 = brk,
                LocalTargetMps = lookSpeed,
                LookaheadM = ld,
                PlanId = planId,
            };
            LastError = e;

            ReissueCount++;
            return true;
        }

        public void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed)
        {
            // Direct already maintains LastError every control tick in OnTick.
            // This path only fills a fallback when OnTick hasn't run yet
            // (first plan tick) so telemetry never gaps.
            try
            {
                if (LastError.Valid && LastError.PlanId == planId) return;
                if (!hasManeuver || cmd.Path == null || cmd.Path.Count < 2)
                {
                    var fb = new PathFollowingError { Valid = false };
                    LastError = fb;
                    return;
                }
                float sEgo;
                float cross;
                float pHead;
                Vector3 closest;
                ClosestOnPath(egoPos, cmd.Path, out sEgo, out cross, out pHead, out closest);
                float localV = SpeedAtS(cmd, sEgo);
                float headErr = RaceMath.HeadingDiffDeg(pHead, egoHeadingDeg);
                LastError = new PathFollowingError
                {
                    Valid = true,
                    LateralErrM = cross,
                    HeadingErrDeg = -headErr,
                    SpeedErrMps = localV - egoSpeed,
                    DistToPathM = Math.Abs(cross),
                    SteerDeg = lastSteer,
                    Throttle01 = lastThr,
                    Brake01 = lastBrk,
                    LocalTargetMps = localV,
                    LookaheadM = RaceMath.Clamp(6f + egoSpeed * 0.7f, 8f, 28f),
                    PlanId = planId,
                };
            }
            catch { }
        }

        private static void ClosestOnPath(Vector3 egoPos, System.Collections.Generic.List<Vector3> path,
            out float sEgo, out float crossTrack, out float pathHeading, out Vector3 closest)
        {
            sEgo = 0f;
            crossTrack = 999f;
            pathHeading = 0f;
            closest = path[0];
            float best = float.MaxValue;
            float accum = 0f;
            int bestSeg = 0;
            RaceMath.Projection bestPr = new RaceMath.Projection();
            for (int i = 0; i < path.Count - 1; i++)
            {
                var pr = RaceMath.ProjectOnSegment(egoPos, path[i], path[i + 1]);
                if (pr.Dist < best)
                {
                    best = pr.Dist;
                    bestSeg = i;
                    bestPr = pr;
                }
            }
            // Arclength to projection.
            accum = 0f;
            for (int i = 0; i < bestSeg; i++)
                accum += RaceMath.FlatDistance(path[i], path[i + 1]);
            sEgo = accum + bestPr.Along;
            closest = bestPr.Closest;
            var segDir = new Vector3(
                path[Math.Min(bestSeg + 1, path.Count - 1)].X - path[bestSeg].X,
                path[Math.Min(bestSeg + 1, path.Count - 1)].Y - path[bestSeg].Y, 0f);
            if (RaceMath.FlatLength(segDir) < 0.3f) segDir = new Vector3(0f, 1f, 0f);
            segDir = RaceMath.FlatNormalize(segDir);
            pathHeading = RaceMath.HeadingFromVector(segDir);
            float cross = RaceMath.FlatCross(segDir, new Vector3(egoPos.X - closest.X, egoPos.Y - closest.Y, 0f));
            crossTrack = cross; // + = ego left of path direction
        }

        private static System.Collections.Generic.List<float> StationSToCumulative(ManeuverCommand c)
        {
            if (c.StationS != null && c.StationS.Count == c.Path.Count)
                return c.StationS;
            var cum = new System.Collections.Generic.List<float>(c.Path.Count);
            float acc = 0f;
            cum.Add(0f);
            for (int i = 1; i < c.Path.Count; i++)
            {
                acc += RaceMath.FlatDistance(c.Path[i - 1], c.Path[i]);
                cum.Add(acc);
            }
            return cum;
        }

        private static Vector3 PointAtS(System.Collections.Generic.List<Vector3> path,
            System.Collections.Generic.List<float> cum, float s)
        {
            if (path.Count == 0) return Vector3.Zero;
            if (s <= 0f) return path[0];
            if (s >= cum[cum.Count - 1]) return path[path.Count - 1];
            for (int i = 0; i < cum.Count - 1; i++)
            {
                if (s >= cum[i] && s <= cum[i + 1])
                {
                    float seg = cum[i + 1] - cum[i];
                    float t = seg > 1e-4f ? (s - cum[i]) / seg : 0f;
                    var a = path[i];
                    var b = path[i + 1];
                    return new Vector3(
                        a.X + (b.X - a.X) * t,
                        a.Y + (b.Y - a.Y) * t,
                        a.Z + (b.Z - a.Z) * t);
                }
            }
            return path[path.Count - 1];
        }

        private static float SpeedAtS(ManeuverCommand c, float s)
        {
            try
            {
                if (c.SpeedProfile == null || c.SpeedProfile.Count == 0) return c.TargetSpeed;
                var cum = StationSToCumulative(c);
                if (s <= 0f) return c.SpeedProfile[0];
                if (s >= cum[cum.Count - 1]) return c.SpeedProfile[c.SpeedProfile.Count - 1];
                for (int i = 0; i < cum.Count - 1; i++)
                {
                    if (s >= cum[i] && s <= cum[i + 1])
                    {
                        float seg = cum[i + 1] - cum[i];
                        float t = seg > 1e-4f ? (s - cum[i]) / seg : 0f;
                        return c.SpeedProfile[i] * (1f - t) + c.SpeedProfile[i + 1] * t;
                    }
                }
                return c.SpeedProfile[c.SpeedProfile.Count - 1];
            }
            catch { return c.TargetSpeed; }
        }
    }

    /// Retired seam placeholder. Use DirectActuator (Actuator=Direct) instead.
    [System.Obsolete("Use Control.DirectActuator.")]
    internal sealed class DirectActuatorStub : IVehicleActuator
    {
        public Vector3 CurrentAim { get; private set; } = Vector3.Zero;
        public float CurrentCruise { get; private set; }
        public int CurrentStyle { get; private set; }
        public bool HasPlan { get; private set; }
        public int ReissueCount => 0;
        public string ActuatorName => "DirectStub(retired)";
        public string LastReason => "";
        public PathFollowingError LastError => new PathFollowingError();
        public void Attach(GTA.Ped driver, GTA.Vehicle vehicle, float cruise, int style, int refreshMs, int stuckMs) { }
        public void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason)
        {
            CurrentAim = aimPoint;
            CurrentCruise = targetSpeed;
            CurrentStyle = style;
            HasPlan = true;
            throw new NotImplementedException("DirectActuatorStub is retired; use Control.DirectActuator.");
        }
        public void SetManeuver(ManeuverCommand cmd) { throw new NotImplementedException(); }
        public void Clear() { HasPlan = false; }
        public bool Valid() => false;
        public void Stop() { }
        public bool OnTick(bool forceReissue, string tacticalName) => false;
        public void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed, RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed) { }
    }
}
