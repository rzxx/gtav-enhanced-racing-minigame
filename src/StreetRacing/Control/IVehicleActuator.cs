using GTA.Math;

namespace StreetRacing.Control
{
    /// Path-following error: desired trajectory vs actual vehicle state.
    /// This is THE experiment readout for the GtaDriver servo: if GTA's
    /// pathfinding cannot hold our path/speed, these errors stay large and
    /// the DirectActuator (steering/throttle/brake) becomes mandatory.
    internal struct PathFollowingError
    {
        public bool Valid;
        public float LateralErrM;   // actual route lateral - desired path lateral
        public float HeadingErrDeg; // ego heading - desired path heading
        public float SpeedErrMps;   // target speed - actual speed
        public float DistToPathM;   // min flat distance to chosen path polyline
    }

    /// Actuator abstraction: the planner outputs (aim point, target speed,
    /// style) and the actuator executes it with whatever low-level driver is
    /// configured. The planner architecture above this seam never changes;
    /// only the servo does (GTA pathfinding vs direct controls).
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
        void Clear();
        bool Valid();
        void Stop();
        /// Returns true when a full control re-issue happened this tick
        /// (GTA driver) or when direct controls were applied (Direct).
        bool OnTick(bool forceReissue, string tacticalName);
        void UpdatePathError(Vector3 egoPos, float egoHeadingDeg, float egoSpeed,
            RaceRoute route, TrajectoryCandidate chosen, bool hasChosen, float targetSpeed);
    }
}
