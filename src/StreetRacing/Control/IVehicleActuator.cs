using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing.Control
{
    /// Path-following error: desired maneuver vs actual vehicle state.
    /// For Direct this is measured against the SELECTED sampled path at a
    /// local speed-dependent lookahead (not a distant aim point). For
    /// GtaDriver it remains the diagnostic of what the game pathfinder did
    /// with our (point, speed) servo command.
    internal struct PathFollowingError
    {
        public bool Valid;
        public float LateralErrM;   // signed cross-track to selected path (+ left)
        public float HeadingErrDeg; // ego heading - desired path heading
        public float SpeedErrMps;   // local planned speed - actual speed
        public float DistToPathM;   // min flat distance to chosen path polyline
        // Controller outputs + local plan state (telemetry).
        public float SteerDeg;
        public float Throttle01;
        public float Brake01;
        public float LocalTargetMps;
        public float LookaheadM;
        public int PlanId;

        // Vehicle-state/control observability for the direct controller.
        // SignedLongMps is positive forward, negative when the car is actually
        // travelling backwards even though Vehicle.Speed stays positive.
        public float SignedLongMps;
        public float LateralVelMps;     // + = moving left in vehicle frame
        public float SlipDeg;           // velocity direction vs nose (+ left)
        public float YawRateDegS;       // + = heading rotating left
        public float DesiredYawRateDegS; // filtered Controller V2 target
        public float RawDesiredYawRateDegS; // planner/path target before slew limiting
        public float SteerActualDeg;
        public float ThrottleActual01;
        public float ThrottlePowerActual01;
        public float BrakeActual01;
        public float SteerSaturationS;
        public string StabilityMode;    // Normal / Watch / Unstable / ReverseMotion
    }

    /// Complete maneuver handed through the actuator seam. Direct executes
    /// the sampled path + speed profile itself; GtaDriver (diagnostic) still
    /// servo-tracks the distant aim point + cruise.
    internal struct ManeuverCommand
    {
        public List<Vector3> Path;
        public List<float> StationS;
        public List<float> SpeedProfile;
        public Vector3 AimPoint;
        public float TargetSpeed;
        public int Style;
        public string Reason;
        public int PlanId;
        // Explicit reverse command (recovery primitive ONLY). Direct must
        // NEVER infer reverse from heading error: a maneuver that commanded
        // zero speed must hold, not back up. Default false = forward only.
        public bool Reverse;
    }

    /// Actuator abstraction: the joint planner outputs a full maneuver
    /// (sampled path + per-station speed profile) and the actuator executes
    /// it. SetPlan (aim, speed) is retained for the GtaDriver baseline and
    /// recovery paths; SetManeuver is the primary seam for Direct.
    internal interface IVehicleActuator
    {
        Vector3 CurrentAim { get; }
        float CurrentCruise { get; }
        int CurrentStyle { get; }
        bool HasPlan { get; }
        int ReissueCount { get; }
        string ActuatorName { get; }
        string LastReason { get; }
        PathFollowingError LastError { get; }
        void Attach(GTA.Ped driver, GTA.Vehicle vehicle, float cruise, int style, int refreshMs, int stuckMs);
        void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason);
        void SetManeuver(ManeuverCommand cmd);
        void Clear();
        bool Valid();
        void Stop();
        /// Direct: applies steering/throttle/brake EVERY tick (~20 Hz).
        /// GtaDriver: rate-limited servo refresh; true only on repath.
        bool OnTick(bool forceReissue, string tacticalName);
        void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed);
    }
}
