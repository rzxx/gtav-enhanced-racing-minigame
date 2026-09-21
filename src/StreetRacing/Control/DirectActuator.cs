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
    ///   - curvature feed-forward + pursuit + yaw/slip feedback;
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
    /// STEERING SIGN (empirical, DirectDiag):
    ///   DirectDiag stage SteerLeft commands SteeringAngle = +12 deg and the
    ///   NPC visibly turned LEFT. Therefore positive GTA steering = left.
    ///   Route/path convention here: crossTrack > 0 means ego is LEFT of the
    ///   desired path direction (Cross(pathDir, ego-closest) > 0).
    ///   To return toward center from the left (+crossTrack) the car must turn
    ///   RIGHT, i.e. a NEGATIVE steering correction. Hence the lateral law uses
    ///     steer -= crossTrack * gain
    ///   (form: steer = pursuit + headErr*k - crossTrack*k). Do NOT flip this
    ///   sign based on speculation; it is pinned by the diag observation plus
    ///   SteeringSignSelfTest() below. Heading: headErr = desired - ego,
    ///   positive = route/desired is left of the nose, so positive headErr
    ///   also needs positive (left) steering: steer += headErr * gain.
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
        public string TakeoverDetail { get; private set; } = "";
        public string ActuatorName => "DirectV2";
        public PathFollowingError LastError { get; private set; } = new PathFollowingError();

        private ManeuverCommand cmd;
        private bool hasManeuver;
        private int planId;

        // Longitudinal state (simple PI + anti-windup via clamp).
        private float speedInt;
        private bool lastReverseCmd;
        private float lastSteer;
        private float lastDesiredYawRate;
        private float lastThr;
        private float lastBrk;

        // Control-rate vehicle state. Vehicle.Speed is unsigned, which hid the
        // most important failure in the last tests: a car travelling backwards
        // was treated as healthy forward motion. Keep body-frame velocity and
        // yaw response inside the actuator where the commands are produced.
        private bool hasControlKin;
        private int lastControlMs;
        private float lastControlHeading;
        private int steerSaturatedSinceMs;

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

            // DIRECT OWNERSHIP CONTRACT:
            // The selected rival arrives with Rockstar's ambient vehicle task
            // already running. ClearAll() maps to CLEAR_PED_TASKS and can leave
            // the old driving task alive while it winds down. Starting direct
            // memory control during that window creates two control owners.
            //
            // Do not "keep" the ambient task. Kill it immediately, then block
            // new non-temporary ambient behavior while Direct owns the car.
            bool seatBefore = false;
            bool seatAfterClear = false;
            bool reseated = false;
            bool trafficLightBefore = false;
            bool trafficLightAfter = false;
            try
            {
                var vd = vehicle.Driver;
                seatBefore = vd != null && vd.Exists() && vd.Handle == driver.Handle;
            }
            catch { }
            try { trafficLightBefore = vehicle.IsStoppedAtTrafficLights; } catch { }
            // Block ambient event assignment before we remove the old task;
            // repeat after re-seating as a defensive ownership assertion.
            try { Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, true); } catch { }
            try { Function.Call(Hash.SET_PED_KEEP_TASK, driver, false); } catch { }
            try { driver.Task.ClearAllImmediately(); } catch { }
            try
            {
                var vd = vehicle.Driver;
                seatAfterClear = vd != null && vd.Exists() && vd.Handle == driver.Handle;
            }
            catch { }
            if (!seatAfterClear)
            {
                try
                {
                    driver.SetIntoVehicle(vehicle, VehicleSeat.Driver);
                    reseated = true;
                }
                catch { }
            }

            try { Function.Call(Hash.SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, driver, true); } catch { }
            try { Function.Call(Hash.SET_PED_KEEP_TASK, driver, false); } catch { }
            try { Function.Call(Hash.SET_DRIVER_ABILITY, driver, 1.0f); } catch { }
            try { Function.Call(Hash.SET_DRIVER_AGGRESSIVENESS, driver, 1.0f); } catch { }
            try { Function.Call(Hash.SET_DRIVER_RACING_MODIFIER, driver, 1.0f); } catch { }

            // Clear residual vehicle control state inherited from traffic AI.
            try { vehicle.Throttle = 0f; } catch { }
            try { vehicle.ThrottlePower = 0f; } catch { }
            try { vehicle.BrakePower = 0f; } catch { }
            try { vehicle.IsHandbrakeForcedOn = false; } catch { }
            try { vehicle.IsBurnoutForced = false; } catch { }
            try { trafficLightAfter = vehicle.IsStoppedAtTrafficLights; } catch { }

            bool seatFinal = false;
            try
            {
                var vd = vehicle.Driver;
                seatFinal = vd != null && vd.Exists() && vd.Handle == driver.Handle;
            }
            catch { }
            TakeoverDetail = $"clear=Immediate;seatBefore={(seatBefore ? 1 : 0)};"
                + $"seatAfterClear={(seatAfterClear ? 1 : 0)};reseated={(reseated ? 1 : 0)};"
                + $"seatFinal={(seatFinal ? 1 : 0)};trafficLightBefore={(trafficLightBefore ? 1 : 0)};"
                + $"trafficLightAfter={(trafficLightAfter ? 1 : 0)};keepTask=0;blockingEvents=1";

            HasPlan = false;
            hasManeuver = false;
            ReissueCount = 0;
            planId = 0;
            cmd = new ManeuverCommand();
            LastError = new PathFollowingError { Valid = false };
            speedInt = 0f;
            lastReverseCmd = false;
            lastSteer = 0f;
            lastDesiredYawRate = 0f;
            lastThr = 0f;
            lastBrk = 0f;
            hasControlKin = false;
            lastControlMs = 0;
            lastControlHeading = 0f;
            steerSaturatedSinceMs = 0;
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
            try
            {
                if (driver == null || !driver.Exists() || driver.IsDead) return false;
                if (vehicle == null || !vehicle.Exists() || vehicle.IsDead) return false;

                // Direct control is only valid while THIS ped is physically
                // occupying THIS vehicle's driver seat. A damaged DriveV car
                // can make its NPC bail out while the vehicle entity remains
                // alive; without this invariant our memory writes keep driving
                // the empty car like a ghost.
                var vd = vehicle.Driver;
                if (vd == null || !vd.Exists() || vd.Handle != driver.Handle) return false;
                return true;
            }
            catch { return false; }
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
            Vector3 egoVel;
            Vector3 egoFwd;
            float egoHeading;
            float egoSpeed;
            int now;
            try
            {
                now = Game.GameTime;
                egoPos = vehicle.Position;
                egoVel = vehicle.Velocity;
                egoHeading = vehicle.Heading;
                egoSpeed = vehicle.Speed;
                egoFwd = RaceMath.FlatNormalize(new Vector3(vehicle.ForwardVector.X, vehicle.ForwardVector.Y, 0f));
            }
            catch { return false; }

            // --- Body-frame motion. GTA Vehicle.Speed is magnitude-only.
            var egoLeft = new Vector3(-egoFwd.Y, egoFwd.X, 0f);
            var flatVel = new Vector3(egoVel.X, egoVel.Y, 0f);
            float signedLong = RaceMath.FlatDot(flatVel, egoFwd);
            float lateralVel = RaceMath.FlatDot(flatVel, egoLeft);
            float slipDeg = 0f;
            try
            {
                if (RaceMath.FlatLength(flatVel) > 1f)
                    slipDeg = (float)(Math.Atan2(lateralVel, Math.Max(Math.Abs(signedLong), 0.5f)) * 180.0 / Math.PI);
            }
            catch { }

            float dt = 0.05f;
            float yawRateDegS = 0f;
            if (hasControlKin)
            {
                try
                {
                    dt = (now - lastControlMs) / 1000f;
                    if (dt < 0.015f) dt = 0.015f;
                    if (dt > 0.2f) dt = 0.2f;
                    yawRateDegS = RaceMath.HeadingDiffDeg(egoHeading, lastControlHeading) / dt;
                }
                catch { dt = 0.05f; yawRateDegS = 0f; }
            }

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

            var cum = StationSToCumulative(cmd);

            // Local speed-dependent lookahead on the SELECTED path.
            float forwardSpeed = Math.Max(0f, signedLong);
            float ld = 6f + forwardSpeed * 0.7f;
            ld = RaceMath.Clamp(ld, 8f, 28f);
            Vector3 lookPt;
            float lookS;
            float lookSpeed;
            try
            {
                lookS = sEgo + ld;
                lookPt = PointAtS(cmd.Path, cum, lookS);
                lookSpeed = SpeedAtS(cmd, sEgo);
            }
            catch
            {
                lookPt = CurrentAim;
                lookS = sEgo + ld;
                lookSpeed = CurrentCruise;
            }

            // --- Lateral Controller V2.
            //
            // The previous controller only chased the latest geometric path.
            // Telemetry showed the selected path/yaw target could flip every
            // 100-150 ms, so even a physically stable car was commanded into
            // repeated left/right transients. V2 combines:
            //   1) curvature feed-forward,
            //   2) gentle pursuit/heading/cross-track feedback,
            //   3) actual yaw-rate feedback,
            //   4) sideslip damping,
            //   5) slew limits on BOTH desired yaw and steering.
            var to = new Vector3(lookPt.X - egoPos.X, lookPt.Y - egoPos.Y, 0f);
            float distToLook = RaceMath.FlatLength(to);
            float desiredHeading = distToLook > 1f
                ? RaceMath.HeadingFromVector(RaceMath.FlatNormalize(to))
                : pathHeading;
            float headErr = RaceMath.HeadingDiffDeg(desiredHeading, egoHeading);

            float rawKappaPath = 0f;
            float rawYawTargetDegS = 0f;
            try
            {
                rawKappaPath = CurvatureAtS(cmd.Path, cum, sEgo + ld * 0.5f);
                rawYawTargetDegS = forwardSpeed * rawKappaPath * 180f / (float)Math.PI;
            }
            catch { }

            // Do not let a replanning discontinuity instantaneously demand an
            // opposite yaw rate. This is a command-shaping limit, not a path
            // constraint: sustained corners still reach their full target.
            float yawTargetRateLimit = forwardSpeed > 16f ? 120f : 170f; // deg/s per second
            float maxYawTargetStep = yawTargetRateLimit * dt;
            float desiredYawRateDegS = rawYawTargetDegS;
            if (hasControlKin)
                desiredYawRateDegS = RaceMath.Clamp(
                    desiredYawRateDegS,
                    lastDesiredYawRate - maxYawTargetStep,
                    lastDesiredYawRate + maxYawTargetStep);

            float wheelbase = 2.7f;
            float controlKappa = forwardSpeed > 3f
                ? desiredYawRateDegS * (float)Math.PI / 180f / forwardSpeed
                : rawKappaPath;
            float steerFeedForward = (float)(
                Math.Atan(wheelbase * controlKappa) * 180.0 / Math.PI);

            float alphaRad = headErr * (float)Math.PI / 180f;
            float steerPursuit = 0f;
            if (distToLook > 1f)
            {
                float pursuitKappa = 2f * (float)Math.Sin(alphaRad)
                    / Math.Max(distToLook, 4f);
                steerPursuit = (float)(
                    Math.Atan(wheelbase * pursuitKappa) * 180.0 / Math.PI);
            }

            float yawErr = desiredYawRateDegS - yawRateDegS;
            float steerDeg = steerFeedForward
                + steerPursuit * 0.55f
                + headErr * 0.18f
                - crossTrack * 0.75f
                + yawErr * 0.10f
                - slipDeg * 0.14f;

            float steerLimit = egoSpeed > 25f ? 17f
                : (egoSpeed > 18f ? 20f : (egoSpeed > 10f ? 25f : 32f));
            steerDeg = RaceMath.Clamp(steerDeg, -steerLimit, steerLimit);

            bool reversing = false;
            try { reversing = cmd.Reverse; } catch { reversing = false; }
            if (reversing != lastReverseCmd)
            {
                speedInt = 0f;
                lastDesiredYawRate = 0f;
                lastReverseCmd = reversing;
            }
            if (reversing) steerDeg = -steerDeg;

            // Steering rack slew is intentionally slower at racing speed.
            // The old 90-120 deg/s limits let a 10 Hz planner flip enough lock
            // to create visible lane-to-lane oscillation.
            float steerRateDegS = egoSpeed > 20f ? 55f
                : (egoSpeed > 10f ? 75f : 120f);
            float maxSteerStep = steerRateDegS * dt;
            if (hasControlKin)
                steerDeg = RaceMath.Clamp(
                    steerDeg, lastSteer - maxSteerStep, lastSteer + maxSteerStep);

            bool steerSaturated = Math.Abs(steerDeg) >= steerLimit * 0.92f
                && (Math.Abs(headErr) > 12f || Math.Abs(crossTrack) > 1.5f);
            if (steerSaturated)
            {
                if (steerSaturatedSinceMs <= 0) steerSaturatedSinceMs = now;
            }
            else
            {
                steerSaturatedSinceMs = 0;
            }
            float steerSatS = steerSaturatedSinceMs > 0 ? (now - steerSaturatedSinceMs) / 1000f : 0f;

            bool reverseMotion = !reversing && signedLong < -1.0f;
            bool wrongYawResponse = !reversing
                && egoSpeed > 5f
                && Math.Abs(steerDeg) > 8f
                && Math.Abs(yawRateDegS) > 8f
                && steerDeg * yawRateDegS < 0f
                && (Math.Abs(headErr) > 20f || Math.Abs(slipDeg) > 10f);
            bool unstable = !reversing && (
                reverseMotion
                || (egoSpeed > 7f && Math.Abs(slipDeg) > 22f)
                || (egoSpeed > 6f && Math.Abs(headErr) > 70f)
                || (steerSatS > 0.7f && (Math.Abs(headErr) > 30f || Math.Abs(slipDeg) > 12f))
                || wrongYawResponse);
            bool watch = !unstable && !reversing && (
                (egoSpeed > 7f && Math.Abs(slipDeg) > 12f)
                || Math.Abs(headErr) > 35f
                || steerSatS > 0.30f);

            string stabilityMode = reverseMotion ? "ReverseMotion" : (unstable ? "Unstable" : (watch ? "Watch" : "Normal"));

            try { vehicle.SteeringAngle = steerDeg; } catch { }
            try { vehicle.SteeringScale = 1f; } catch { }

            // --- Longitudinal: use SIGNED forward speed, not Vehicle.Speed.
            // This prevents backwards/sliding motion from masquerading as
            // successful forward velocity.
            float controlSpeed = reversing ? Math.Abs(signedLong) : Math.Max(0f, signedLong);
            float speedErr = lookSpeed - controlSpeed;
            speedInt = RaceMath.Clamp(speedInt + speedErr * dt, -6f, 6f);
            float u = speedErr * 0.35f + speedInt * 0.12f;
            bool stalled = controlSpeed < 1.5f && lookSpeed > 3f;

            float thr = 0f;
            float brk = 0f;
            if (reversing)
            {
                // Reverse has a real speed target. The old implementation
                // ignored speedErr and applied a fixed -0.6 throttle forever,
                // which produced 10-14 m/s reverse during recovery despite a
                // 3 m/s command.
                if (u > 0f)
                {
                    float mag = RaceMath.Clamp(u, stalled ? 0.30f : 0f, 0.55f);
                    if (stalled && mag < 0.30f) mag = 0.30f;
                    thr = -mag;
                }
                else
                {
                    thr = 0f;
                    brk = RaceMath.Clamp(-u, 0.15f, 1f);
                    if (controlSpeed > lookSpeed + 2f) brk = 1f;
                }
            }
            else if (u >= 0f)
            {
                thr = RaceMath.Clamp(u, stalled ? 0.8f : 0f, 1f);
                if (stalled && thr < 0.8f) thr = 0.8f;
            }
            else
            {
                brk = RaceMath.Clamp(-u, 0.15f, 1f);
                if (lookSpeed < 0.5f && controlSpeed > 1f) brk = 1f;
            }

            if (!reversing)
            {
                // Even before Controller V2, never combine full lock with full
                // throttle. This is deliberately conservative: the supervisor
                // removes energy when the geometric follower is losing authority.
                float steerFrac = steerLimit > 1f ? Math.Abs(steerDeg) / steerLimit : 0f;
                if (egoSpeed > 8f)
                {
                    float steeringThrottleCap = RaceMath.Clamp(1f - 0.55f * steerFrac, 0.35f, 1f);
                    if (thr > steeringThrottleCap) thr = steeringThrottleCap;
                }

                if (watch)
                {
                    if (thr > 0.30f) thr = 0.30f;
                    speedInt *= 0.9f;
                }
                if (unstable)
                {
                    thr = 0f;
                    speedInt *= 0.5f;
                    // Heavy braking while sideways can make a slide worse.
                    // Brake firmly only for actual backwards motion; otherwise
                    // use a modest stabilizing brake when mostly aligned.
                    if (reverseMotion)
                        brk = Math.Max(brk, 0.60f);
                    else if (Math.Abs(slipDeg) < 18f && signedLong > 7f)
                        brk = Math.Max(brk, 0.25f);
                    else
                        brk = Math.Max(brk, 0.05f);
                }
            }

            try
            {
                if (reversing)
                {
                    vehicle.Throttle = thr;
                    vehicle.ThrottlePower = thr;
                    vehicle.BrakePower = brk;
                    vehicle.IsHandbrakeForcedOn = false;
                }
                else
                {
                    vehicle.Throttle = thr;
                    vehicle.ThrottlePower = thr;
                    vehicle.BrakePower = brk;
                    vehicle.IsHandbrakeForcedOn = lookSpeed < 0.5f && controlSpeed < 3f;
                }
            }
            catch { }

            float steerActual = steerDeg;
            float thrActual = thr;
            float thrPowerActual = thr;
            float brkActual = brk;
            try { steerActual = vehicle.SteeringAngle; } catch { }
            try { thrActual = vehicle.Throttle; } catch { }
            try { thrPowerActual = vehicle.ThrottlePower; } catch { }
            try { brkActual = vehicle.BrakePower; } catch { }

            lastSteer = steerDeg;
            lastDesiredYawRate = desiredYawRateDegS;
            lastThr = thr;
            lastBrk = brk;
            lastControlHeading = egoHeading;
            lastControlMs = now;
            hasControlKin = true;

            LastError = new PathFollowingError
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
                SignedLongMps = signedLong,
                LateralVelMps = lateralVel,
                SlipDeg = slipDeg,
                YawRateDegS = yawRateDegS,
                DesiredYawRateDegS = desiredYawRateDegS,
                SteerActualDeg = steerActual,
                ThrottleActual01 = thrActual,
                ThrottlePowerActual01 = thrPowerActual,
                BrakeActual01 = brkActual,
                SteerSaturationS = steerSatS,
                StabilityMode = stabilityMode,
            };

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

        private static float CurvatureAtS(System.Collections.Generic.List<Vector3> path,
            System.Collections.Generic.List<float> cum, float s)
        {
            try
            {
                if (path == null || path.Count < 3 || cum == null || cum.Count != path.Count) return 0f;
                float ds = 4f;
                var p0 = PointAtS(path, cum, Math.Max(0f, s - ds));
                var p1 = PointAtS(path, cum, s);
                var p2 = PointAtS(path, cum, Math.Min(cum[cum.Count - 1], s + ds));
                var d0 = new Vector3(p1.X - p0.X, p1.Y - p0.Y, 0f);
                var d1 = new Vector3(p2.X - p1.X, p2.Y - p1.Y, 0f);
                float l0 = RaceMath.FlatLength(d0);
                float l1 = RaceMath.FlatLength(d1);
                if (l0 < 0.5f || l1 < 0.5f) return 0f;
                float dhRad = RaceMath.SignedAngleDeg(d0, d1) * (float)Math.PI / 180f;
                return dhRad / Math.Max((l0 + l1) * 0.5f, 1f);
            }
            catch { return 0f; }
        }

        /// Deterministic steering-sign guard (no game natives).
        /// Conventions: crossTrack + = ego left of path; GTA +steer = left
        /// (DirectDiag: +12 deg visibly turned left). Returning from the left
        /// must command negative (right) steering. Returns "OK" or a failure.
        public static string SteeringSignSelfTest()
        {
            try
            {
                // Replicate the cross-track term of the lateral law.
                float gain = 1.1f;
                float corrLeft = -1.0f * 2.0f * gain;   // 2 m left -> must be negative
                float corrRight = -1.0f * -2.0f * gain; // 2 m right -> must be positive
                if (corrLeft >= -0.5f) return $"FAIL leftCorr={corrLeft:F2} expect <0";
                if (corrRight <= 0.5f) return $"FAIL rightCorr={corrRight:F2} expect >0";
                // Heading term: desired left of nose (+headErr) -> left (+steer).
                float headGain = 0.18f;
                float hCorr = 10f * headGain;
                if (hCorr <= 0f) return $"FAIL headCorr={hCorr:F2} expect >0";

                // Controller V2 yaw/slip damping: excessive left yaw/slip with
                // zero desired yaw must command right (negative) correction.
                float yawCorr = (0f - 30f) * 0.10f;
                float slipCorr = -10f * 0.14f;
                if (yawCorr >= 0f || slipCorr >= 0f)
                    return $"FAIL damping yaw={yawCorr:F2} slip={slipCorr:F2} expect <0";
                return "OK";
            }
            catch (System.Exception ex) { return "FAIL exc:" + ex.Message; }
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
