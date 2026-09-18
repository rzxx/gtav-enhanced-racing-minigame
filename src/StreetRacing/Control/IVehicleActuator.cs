using GTA.Math;

namespace StreetRacing.Control
{
    /// Actuator abstraction: the planner outputs (aim point, target speed,
    /// style) and the actuator executes it with whatever low-level driver is
    /// configured. Today that is the stock GTA task driver used as a
    /// receding-horizon servo; tomorrow it can be a direct
    /// steering/throttle/brake controller with no planner changes.
    internal interface IVehicleActuator
    {
        Vector3 CurrentAim { get; }
        float CurrentCruise { get; }
        int CurrentStyle { get; }
        bool HasPlan { get; }
        int ReissueCount { get; }
        void SetPlan(Vector3 aimPoint, float targetSpeed, int style, string reason);
        void Clear();
    }
}
