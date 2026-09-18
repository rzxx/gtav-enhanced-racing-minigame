using System;
using GTA;
using GTA.Math;
using StreetRacing.Control;
using StreetRacing.Debug;
using StreetRacing.Tactics;

namespace StreetRacing.Race
{
    /// Orchestrator for the street-racing AI:
    ///   route -> corridor -> persistent perception/prediction -> tactics ->
    ///   JOINT maneuver (path + speed together) -> actuator.
    ///
    /// Architecture (this pass):
    ///   - Perception tracks entities persistently by handle (short
    ///     expiry/hysteresis), always retaining route-relevant + near safety
    ///     + rival. No TTC-sorted top-24 rebuild.
    ///   - TrajectoryPlanner plans JOINT maneuvers: every candidate path gets
    ///     its own curvature profile, arrival times, per-path swept-envelope
    ///     conflict test, per-path braking pass, and a complete-maneuver
    ///     score. There is no global corridor-wide obstacle speed.
    ///   - DirectActuator executes the selected sampled path/speed every
    ///     script tick (~20 Hz) with a local speed-dependent lookahead.
    ///     GtaDriver remains only as a baseline/diagnostic.
    ///
    /// Tick cadence (script runs at ~20 Hz / 50 ms):
    ///   every tick : ego kinematics + capability + Direct control +
    ///                path-error + debug viz
    ///   10 Hz      : perception, route, tactics, joint maneuver, telemetry
    ///   5 Hz       : corridor width probes (native-heavy)
    internal sealed class RaceBrain
    {
        private Ped driver;
        private Vehicle vehicle;
        private Vector3 finish = Vector3.Zero;
        private float cruiseSetting;
        private int style;
        private DriverProfile profile;
        private RaceTelemetry telemetry;
        private int t0;

        private readonly RaceRoute route = new RaceRoute();
        private readonly RoadCorridor corridor = new RoadCorridor();
        private readonly Perception perception = new Perception();
        private readonly VehicleCapability capability = new VehicleCapability();
        private readonly TrajectoryPlanner traj = new TrajectoryPlanner();
        private readonly SpeedPlanner speedPlan = new SpeedPlanner();
        private readonly RaceTactics tactics = new RaceTactics();
        private readonly RaceDebugViz viz = new RaceDebugViz();
        private IVehicleActuator actuator;

        private int refreshMs = 2000;
        private int stuckMs = 4000;

        // Ego kinematics history for accel / impact classification.
        private float lastSpeed;
        private Vector3 lastPos = Vector3.Zero;
        private Vector3 lastVel = Vector3.Zero;
        private float lastHeading;
        private int lastKinT;
        private float lastHealth = -1f;
        private bool hasKin;
        private SampleKind lastImpact = SampleKind.Normal;
        private int lastImpactEventMs = -100000;
        private float lastAccelLong;
        private float lastLatAccel;
        private float lastYawRate;
        private float lastSlipDeg;

        // Scheduling.
        private int lastPercMs;
        private int lastCorrMs;
        private int lastPlanMs;
        private int lastTeleMs;
        private int lastGpsRetryMs;

        // Plan snapshot for telemetry / HUD.
        public string TacticalName => tactics.Mode.ToString();
        public float TargetSpeed { get; private set; }
        public string SpeedLimit { get; private set; } = "Cruise";
        public float ActualSpeed { get; private set; }
        public float FinishGap { get; private set; }
        public float Progress01 => route.Progress01;
        public bool RouteLost => route.IsLost;
        public float LookaheadM { get; private set; } = 80f;
        public TrajectoryCandidate Chosen => traj.HasChosen ? traj.Chosen : new TrajectoryCandidate();
        public bool Running { get; private set; }
        public string RouteSource => route.Source;
        public string ActuatorName => actuator != null ? actuator.ActuatorName : "?";

        private TacticalMode lastLoggedTactic = (TacticalMode)(-1);
        private bool lastLoggedLost;
        private string lastLoggedLossReason = "";
        private int lastBrakeEventMs = -100000;
        private float lastPathErrLat;
        private float lastPathErrHead;
        private int lastPathErrLogMs;

        // Planner-change detection (telemetry: PLAN events + planId).
        private int maneuverPlanId;
        private int lastPlanLogId = -1;
        private float lastChosenLat = 999f;
        private float lastChosenSpeed = -1f;
        private string lastChosenLimit = "";
        private bool pendingForceReissue;

        // Self-ped diagnosis: handles to correlate Ped@s=0 blockers.
        // Observed clearance -1.6/-1.7m == 0-(1.15+0.45): zero center distance
        // minus car+ped half-widths, i.e. the AI driver at ego center.
        private int oppDriverHandle;
        private int egoVehicleHandle;
        private int lastSelfPedLogMs = -100000;

        public void Start(Ped driver, Vehicle vehicle, Vector3 finish, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs)
        {
            Start(driver, vehicle, finish, cruise, style, profile, telemetry, refreshMs, stuckMs, true, false);
        }

        public void Start(Ped driver, Vehicle vehicle, Vector3 finish, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs, bool useDirect, bool debugViz)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.finish = finish;
            this.cruiseSetting = cruise;
            this.style = style;
            this.profile = profile ?? DriverProfile.FromName("balanced");
            this.telemetry = telemetry;
            this.refreshMs = refreshMs;
            this.stuckMs = stuckMs;
            t0 = Game.GameTime;

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            route.Build(origin, finish);
            capability.Seed(vehicle);
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            corridor.Update(route, origin, LookaheadM, t0);
            route.Update(origin, SafeHeading(vehicle), 0f, t0, corridor.HalfWidth);

            actuator = useDirect ? (IVehicleActuator)new DirectActuator() : (IVehicleActuator)new GtaDriverActuator();
            actuator.Attach(driver, vehicle, cruise, style, refreshMs, stuckMs);
            viz.Enabled = debugViz;
            tactics.SinceMs = t0;

            hasKin = false;
            lastHealth = -1f;
            maneuverPlanId = 0;
            lastPlanLogId = -1;
            lastChosenLat = 999f;
            pendingForceReissue = true;
            Running = true;

            try
            {
                oppDriverHandle = 0;
                egoVehicleHandle = 0;
                try { if (driver != null && driver.Exists()) oppDriverHandle = driver.Handle; } catch { }
                try { if (vehicle != null && vehicle.Exists()) egoVehicleHandle = vehicle.Handle; } catch { }
            }
            catch { }

            try
            {
                telemetry?.Event(0, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};prof={this.profile.Name};risk={this.profile.RiskTolerance:F2}");
                telemetry?.Event(0, "OPP_DRIVER", $"oppDriverHandle={oppDriverHandle};egoVehHandle={egoVehicleHandle};selfPedClearExpect=-1.6m(0-(1.15+0.45))");
                telemetry?.Event(0, "ACTUATOR", $"{actuator.ActuatorName};joint maneuver (path+speed) -> {(useDirect ? "Direct 20Hz path/speed execution" : "GtaDriver baseline servo (diagnostic only)")}");
            }
            catch { }
        }

        public bool Valid()
        {
            try
            {
                return Running && actuator != null && actuator.Valid();
            }
            catch { return false; }
        }

        public void Stop()
        {
            Running = false;
            try { actuator?.Stop(); } catch { }
        }

        public void OnTick()
        {
            if (!Running) return;
            int now = Game.GameTime;
            int t = now - t0;

            Vector3 egoPos;
            Vector3 egoVel;
            Vector3 egoFwd;
            float egoSpeed;
            float egoHeading;
            try
            {
                if (vehicle == null || !vehicle.Exists()) return;
                egoPos = vehicle.Position;
                egoVel = vehicle.Velocity;
                egoSpeed = vehicle.Speed;
                egoFwd = RaceMath.FlatNormalize(new Vector3(vehicle.ForwardVector.X, vehicle.ForwardVector.Y, 0f));
                egoHeading = vehicle.Heading;
            }
            catch { return; }
            ActualSpeed = egoSpeed;
            FinishGap = RaceMath.FlatDistance(egoPos, finish);

            UpdateKinematics(now, egoPos, egoVel, egoSpeed, egoHeading);

            if (!hasKin)
            {
                hasKin = true;
                lastSpeed = egoSpeed;
                lastPos = egoPos;
                lastVel = egoVel;
                lastHeading = egoHeading;
                lastKinT = now;
                try { lastHealth = vehicle.HealthFloat; } catch { lastHealth = -1f; }
                return;
            }

            // --- 10 Hz: perception (persistent, route-aware).
            if (now - lastPercMs >= profile.ReactionIntervalMs)
            {
                lastPercMs = now;
                float reactionS = profile.ReactionIntervalMs / 1000f;
                Vehicle playerVeh = SafePlayerVehicle();
                Ped playerPed = null;
                try { playerPed = Game.Player.Character; } catch { }
                try
                {
                    // Invariant: pass the AI driver so Perception excludes it
                    // + all ego occupants (self-ped @s=0 guard).
                    perception.Update(vehicle, driver, playerVeh, playerPed, egoSpeed,
                        capability.UsableBrake(profile.GripFactor), reactionS, now, profile.ReactionIntervalMs,
                        route, corridor);
                }
                catch { }
            }

            bool doPlan = now - lastPlanMs >= profile.ReactionIntervalMs;
            bool routeUpgradedThisTick = false;
            string routeUpgradeLog = "";
            if (doPlan)
            {
                lastPlanMs = now;
                try { route.Update(egoPos, egoHeading, egoSpeed, now, corridor.HalfWidth); }
                catch { }

                try
                {
                    if (!route.Source.StartsWith("Gps") && now - t0 < 12000 && now - lastGpsRetryMs > 1000)
                    {
                        lastGpsRetryMs = now;
                        string ulog;
                        if (route.TryUpgradeToGps(egoPos, egoHeading, egoSpeed, now, corridor.HalfWidth, out ulog))
                        {
                            routeUpgradedThisTick = true;
                            routeUpgradeLog = ulog ?? "";
                            try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
                            // Log old/new heading/projection/lateral so a
                            // FallbackWalk->GPS switch is auditable as
                            // setup/route evidence, never silently absorbed.
                            // Invalidate the old maneuver: the pre-upgrade path
                            // was built from incomparable arclength/heading.
                            try { telemetry?.Event(t, "GPS_ROUTE", $"upgraded;{routeUpgradeLog}"); } catch { }
                            try
                            {
                                traj.LastCandidates.Clear();
                                traj.HasChosen = false;
                            }
                            catch { }
                        }
                    }
                }
                catch { }
            }

            if (now - lastCorrMs >= 200)
            {
                lastCorrMs = now;
                try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
            }

            if (doPlan)
            {
                Vector3 rivalPos = egoPos;
                Vector3 rivalVel = new Vector3();
                float rivalDist = 9999f;
                bool rivalAhead = false;
                try
                {
                    var pc = Game.Player.Character;
                    Vehicle pv = null;
                    try { pv = pc.IsInVehicle() ? pc.CurrentVehicle : null; } catch { }
                    if (pv != null && pv.Exists() && pv != vehicle)
                    {
                        rivalPos = pv.Position;
                        try { rivalVel = pv.Velocity; } catch { }
                    }
                    else if (pc != null && pc.Exists())
                    {
                        rivalPos = pc.Position;
                        try { rivalVel = pc.Velocity; } catch { }
                    }
                    rivalDist = RaceMath.FlatDistance(egoPos, rivalPos);
                    var toR = new Vector3(rivalPos.X - egoPos.X, rivalPos.Y - egoPos.Y, 0f);
                    rivalAhead = RaceMath.FlatDot(toR, egoFwd) > 0f;
                }
                catch { }

                bool justImpacted = (now - lastImpactEventMs) < 1200;
                LookaheadM = profile.LookaheadForSpeed(egoSpeed);

                float aLatPreview = capability.UsableLat(profile.GripFactor) / (profile.CornerCaution * profile.CornerCaution);
                float k80 = route.CurvatureAhead(80f);
                float curvePreview = k80 > 1e-5f ? (float)Math.Sqrt(aLatPreview / k80) : cruiseSetting;

                try
                {
                    tactics.Update(route, corridor, perception, egoPos, egoFwd, egoSpeed,
                        rivalPos, rivalVel, rivalDist, rivalAhead, curvePreview, cruiseSetting,
                        now, profile, justImpacted);
                }
                catch { }

                TrajectoryCandidate chosen = new TrajectoryCandidate();
                float target = cruiseSetting;
                string limiting = "Cruise";
                bool recoveredTarget = false;
                try
                {
                    if (route.IsLost || tactics.Mode == TacticalMode.Recovery || tactics.Mode == TacticalMode.Crashed)
                    {
                        Vector3 rec = route.RecoveryTarget();
                        float recS = 40f;
                        chosen = new TrajectoryCandidate
                        {
                            LateralM = 0f,
                            LookaheadM = 40f,
                            AimPoint = rec,
                            Score = 0f,
                            ClearanceM = 999f,
                            CurveCost = 0f,
                            TacticalBias = 0f,
                            RejectReason = route.LossReason,
                            Path = new System.Collections.Generic.List<Vector3> { egoPos, rec },
                            StationS = new System.Collections.Generic.List<float> { 0f, recS },
                            SpeedProfile = new System.Collections.Generic.List<float> { Math.Min(cruiseSetting, 11f), Math.Min(cruiseSetting, 11f) },
                            ArrivalT = new System.Collections.Generic.List<float> { 0f, 4f },
                            MinMarginM = 99f,
                            MaxKappa = 0f,
                            TargetSpeed = Math.Min(cruiseSetting, 11f),
                            SpeedLimiting = "Recovery",
                            ConstrainHandle = -1,
                            ConstrainKind = "",
                            ConstrainS = -1f,
                            MinPredClearance = 999f,
                            MeanSpeed = Math.Min(cruiseSetting, 11f),
                            MinSpeed = Math.Min(cruiseSetting, 11f),
                            RequiredDecel = 0f,
                            CandidateIndex = -1,
                        };
                        traj.LastCandidates.Clear();
                        traj.LastCandidates.Add(chosen);
                        traj.Chosen = chosen;
                        traj.HasChosen = true;
                        traj.BrakingPointS = -1f;
                        traj.PlanId++;
                        target = chosen.TargetSpeed;
                        limiting = "Recovery";
                        recoveredTarget = true;
                    }
                    else
                    {
                        // JOINT plan: path + speed together. No global
                        // corridor-wide obstacle speed afterwards.
                        chosen = traj.PlanJoint(route, corridor, perception, tactics, profile,
                            capability, egoPos, egoFwd, egoSpeed, LookaheadM, cruiseSetting);
                        target = chosen.TargetSpeed;
                        limiting = string.IsNullOrEmpty(chosen.SpeedLimiting) ? "Cruise" : chosen.SpeedLimiting;
                    }
                }
                catch { }

                TargetSpeed = target;
                SpeedLimit = limiting ?? "Cruise";
                maneuverPlanId = traj.PlanId;

                // Mirror the CHOSEN maneuver into speedPlan for viz compat.
                try
                {
                    speedPlan.TargetSpeed = target;
                    speedPlan.Limiting = SpeedLimit;
                    speedPlan.ProfileS.Clear();
                    speedPlan.ProfileAllowed.Clear();
                    speedPlan.ProfileTarget.Clear();
                    if (traj.HasChosen && traj.Chosen.SpeedProfile != null)
                    {
                        for (int i = 0; i < traj.Chosen.SpeedProfile.Count; i++)
                        {
                            float s = (traj.Chosen.StationS != null && i < traj.Chosen.StationS.Count)
                                ? traj.Chosen.StationS[i] : i * 10f;
                            speedPlan.ProfileS.Add(s);
                            speedPlan.ProfileAllowed.Add(traj.Chosen.SpeedProfile[i]);
                            speedPlan.ProfileTarget.Add(traj.Chosen.SpeedProfile[i]);
                        }
                    }
                    speedPlan.BrakingPointS = traj.BrakingPointS;
                    speedPlan.CurveLimit = target;
                    speedPlan.ObstacleLimit = target;
                    float dv = egoSpeed - target;
                    speedPlan.RequiredDecel = dv <= 0f ? 0f : (dv * dv) / Math.Max(2f * Math.Max(LookaheadM * 0.6f, 15f), 1f);
                    speedPlan.BrakingNeed = RaceMath.Clamp(speedPlan.RequiredDecel / Math.Max(capability.UsableBrake(profile.GripFactor), 1f), 0f, 1f);
                }
                catch { }

                // --- Planner-change detection (flicker readout).
                try
                {
                    bool changed = false;
                    string what = "";
                    if (lastPlanLogId < 0)
                    {
                        changed = true;
                        what = "init";
                    }
                    else
                    {
                        if (traj.HasChosen && Math.Abs(traj.Chosen.LateralM - lastChosenLat) > 2f)
                        {
                            changed = true;
                            what += $"lat{lastChosenLat:F1}->{traj.Chosen.LateralM:F1} ";
                        }
                        if (Math.Abs(target - lastChosenSpeed) > 3f)
                        {
                            changed = true;
                            what += $"v{lastChosenSpeed:F0}->{target:F0} ";
                        }
                        if ((limiting ?? "") != (lastChosenLimit ?? ""))
                        {
                            changed = true;
                            what += $"{lastChosenLimit}->{limiting} ";
                        }
                    }
                    if (changed)
                    {
                        string det = "";
                        try
                        {
                            var ch = traj.HasChosen ? traj.Chosen : chosen;
                            string prof = "";
                            try
                            {
                                if (ch.SpeedProfile != null)
                                {
                                    var parts = new System.Collections.Generic.List<string>(ch.SpeedProfile.Count);
                                    foreach (var v in ch.SpeedProfile)
                                        parts.Add(v.ToString("F0"));
                                    prof = string.Join("/", parts.ToArray());
                                }
                            }
                            catch { prof = ""; }
                            det = $"id={maneuverPlanId};chIdx={ch.CandidateIndex};lat={ch.LateralM:F1};v={target:F1};lim={limiting};"
                                + $"clear={ch.MinPredClearance:F1};constr={ch.ConstrainKind}#{ch.ConstrainHandle}@{(ch.ConstrainS >= 0 ? ch.ConstrainS.ToString("F0") : "-")};"
                                + $"oppDrv={oppDriverHandle};egoVeh={egoVehicleHandle};"
                                + $"meanV={ch.MeanSpeed:F1};minV={ch.MinSpeed:F1};prof={prof};score={ch.Score:F2};why={what.Trim()}";
                        }
                        catch { det = what; }
                        try { telemetry?.Event(t, "PLAN", det); } catch { }
                        // Explicit self-ped verification: a Ped@s=0 constraint
                        // whose handle equals the AI driver is the false
                        // self-block (clearance -1.6m). After the Perception
                        // + SpeedPlanner fixes this must never fire; if it
                        // does, it is source/invariant failure, not tuning.
                        try
                        {
                            var ch = traj.HasChosen ? traj.Chosen : chosen;
                            bool isPed = !string.IsNullOrEmpty(ch.ConstrainKind)
                                && ch.ConstrainKind.ToLowerInvariant().Contains("ped");
                            if (isPed && ch.ConstrainS >= 0f && ch.ConstrainS < 3f
                                && ch.ConstrainHandle != -1 && ch.ConstrainHandle == oppDriverHandle)
                            {
                                try { telemetry?.Event(t, "SELF_PED", $"constrPed@{ch.ConstrainS:F0}==oppDriver#{oppDriverHandle};clear={ch.MinPredClearance:F1};SELF-BLOCK-SOURCE-FAIL"); } catch { }
                            }
                        }
                        catch { }
                    }
                    lastPlanLogId = maneuverPlanId;
                    if (traj.HasChosen) lastChosenLat = traj.Chosen.LateralM;
                    lastChosenSpeed = target;
                    lastChosenLimit = limiting;
                }
                catch { }

                // Self-ped invariant check EVERY plan tick (not only on change):
                // a persistent Ped@s=0==oppDriver block would otherwise hide
                // behind "no change" once zeroed. Classification: planner vs
                // setup/route vs controller comes from PLAN vs prog/headErr
                // vs PATH_ERR — this event pins the source to self.
                try
                {
                    if (traj.HasChosen)
                    {
                        var ch = traj.Chosen;
                        bool isPed = !string.IsNullOrEmpty(ch.ConstrainKind)
                            && ch.ConstrainKind.ToLowerInvariant().Contains("ped");
                        if (isPed && ch.ConstrainS >= 0f && ch.ConstrainS < 3f
                            && ch.ConstrainHandle != -1 && ch.ConstrainHandle == oppDriverHandle
                            && now - lastSelfPedLogMs > 4000)
                        {
                            lastSelfPedLogMs = now;
                            // Reuse path-err throttle to avoid event spam; the
                            // PLAN-embedded SELF_PED above fires on change.
                            try { telemetry?.Event(t, "SELF_PED", $"persist constrPed@{ch.ConstrainS:F0}==oppDriver#{oppDriverHandle};clear={ch.MinPredClearance:F1}"); } catch { }
                        }
                    }
                }
                catch { }

                // --- Hand the FULL maneuver through the seam (not an aim).
                // A GPS upgrade invalidates the old maneuver (incomparable
                // arclength/heading): force reissue so Direct drops the stale
                // path the same tick. Never fall back to GTA on >90 deg
                // heading errors — those are route/setup evidence first.
                bool forceReissue = tactics.ChangedThisTick || recoveredTarget || routeUpgradedThisTick;
                if (lastLoggedTactic != tactics.Mode) forceReissue = true;
                if (route.IsLost != lastLoggedLost) forceReissue = true;
                pendingForceReissue = forceReissue;
                try
                {
                    Vector3 aim = traj.HasChosen ? traj.Chosen.AimPoint : route.LookaheadPoint(LookaheadM);
                    aim = ClampAimToCorridor(aim, route);
                    if (actuator is DirectActuator)
                    {
                        var m = new ManeuverCommand
                        {
                            Path = traj.HasChosen && traj.Chosen.Path != null
                                ? new System.Collections.Generic.List<Vector3>(traj.Chosen.Path)
                                : new System.Collections.Generic.List<Vector3> { egoPos, aim },
                            StationS = traj.HasChosen && traj.Chosen.StationS != null
                                ? new System.Collections.Generic.List<float>(traj.Chosen.StationS)
                                : new System.Collections.Generic.List<float> { 0f, LookaheadM },
                            SpeedProfile = traj.HasChosen && traj.Chosen.SpeedProfile != null
                                ? new System.Collections.Generic.List<float>(traj.Chosen.SpeedProfile)
                                : new System.Collections.Generic.List<float> { target, target },
                            AimPoint = aim,
                            TargetSpeed = Math.Max(0f, target),
                            Style = style,
                            Reason = tactics.Mode.ToString(),
                            PlanId = maneuverPlanId,
                        };
                        actuator.SetManeuver(m);
                    }
                    else
                    {
                        actuator.SetPlan(aim, Math.Max(0f, target), style, tactics.Mode.ToString());
                        if (traj.HasChosen && traj.Chosen.Path != null)
                        {
                            try
                            {
                                actuator.SetManeuver(new ManeuverCommand
                                {
                                    Path = new System.Collections.Generic.List<Vector3>(traj.Chosen.Path),
                                    StationS = traj.Chosen.StationS != null
                                        ? new System.Collections.Generic.List<float>(traj.Chosen.StationS) : null,
                                    SpeedProfile = traj.Chosen.SpeedProfile != null
                                        ? new System.Collections.Generic.List<float>(traj.Chosen.SpeedProfile) : null,
                                    AimPoint = aim,
                                    TargetSpeed = Math.Max(0f, target),
                                    Style = style,
                                    Reason = tactics.Mode.ToString(),
                                    PlanId = maneuverPlanId,
                                });
                            }
                            catch { }
                        }
                    }
                }
                catch { }

                EmitTransitionEvents(t, now);
            }

            // --- EVERY-tick control (Direct: 20 Hz path/speed execution;
            // GtaDriver: rate-limited servo, safe to call every tick).
            try
            {
                bool force = pendingForceReissue;
                bool reissued = actuator.OnTick(force, tactics.Mode.ToString());
                pendingForceReissue = false;
                if (reissued && actuator is GtaDriverActuator gta && gta.LastTickReissued)
                {
                    try
                    {
                        var ch = traj.HasChosen ? traj.Chosen : new TrajectoryCandidate();
                        telemetry?.Event(t, "CTRL", $"aim={actuator.CurrentAim.X:F0},{actuator.CurrentAim.Y:F0};v={TargetSpeed:F0};mode={tactics.Mode};why={actuator.LastReason};chIdx={ch.CandidateIndex};constr={ch.ConstrainKind}#{ch.ConstrainHandle}");
                    }
                    catch { }
                }
            }
            catch { }

            // --- Path error every tick (Direct maintains it in OnTick).
            try
            {
                actuator?.UpdatePathError(egoPos, egoHeading, egoSpeed, route,
                    traj.HasChosen ? traj.Chosen : new TrajectoryCandidate(),
                    traj.HasChosen, TargetSpeed);
                var pe = actuator.LastError;
                if (pe.Valid)
                {
                    lastPathErrLat = pe.LateralErrM;
                    lastPathErrHead = pe.HeadingErrDeg;
                    if ((Math.Abs(pe.DistToPathM) > 9f || Math.Abs(pe.SpeedErrMps) > 9f) && now - lastPathErrLogMs > 4000)
                    {
                        lastPathErrLogMs = now;
                        try { telemetry?.Event(t, "PATH_ERR", $"distPath={pe.DistToPathM:F0};headErr={pe.HeadingErrDeg:F0};vErr={pe.SpeedErrMps:F0};act={actuator.ActuatorName};locV={pe.LocalTargetMps:F0};steer={pe.SteerDeg:F0}"); } catch { }
                    }
                }
            }
            catch { }

            try
            {
                if (viz.Enabled)
                    viz.Draw(route, corridor, traj, perception, speedPlan, egoPos, egoSpeed, LookaheadM, TargetSpeed);
            }
            catch { }

            if (now - lastTeleMs >= 100)
            {
                lastTeleMs = now;
                WriteSample(t, now, egoPos, egoSpeed);
            }

            lastSpeed = egoSpeed;
            lastPos = egoPos;
            lastVel = egoVel;
            lastHeading = egoHeading;
            lastKinT = now;
        }

        private void UpdateKinematics(int now, Vector3 egoPos, Vector3 egoVel, float egoSpeed, float egoHeading)
        {
            if (!hasKin) return;
            float dtS = (now - lastKinT) / 1000f;
            if (dtS <= 0f || dtS > 0.6f)
            {
                lastImpact = SampleKind.Normal;
                return;
            }
            float accel = (egoSpeed - lastSpeed) / dtS;
            lastAccelLong = accel;
            float dhDeg = RaceMath.HeadingDiffDeg(egoHeading, lastHeading);
            float yawRate = dhDeg * (float)Math.PI / 180f / dtS;
            lastYawRate = yawRate;
            lastLatAccel = egoSpeed * yawRate;
            float slip = 0f;
            try
            {
                float vFlat = RaceMath.FlatLength(new Vector3(egoVel.X, egoVel.Y, 0f));
                if (vFlat > 2f && egoSpeed > 2f)
                {
                    var velDir = RaceMath.FlatNormalize(new Vector3(egoVel.X, egoVel.Y, 0f));
                    Vector3 fwd;
                    try { fwd = RaceMath.FlatNormalize(new Vector3(vehicle.ForwardVector.X, vehicle.ForwardVector.Y, 0f)); }
                    catch { fwd = RaceMath.VectorFromHeading(egoHeading); }
                    slip = RaceMath.SignedAngleDeg(fwd, velDir);
                }
            }
            catch { }
            lastSlipDeg = slip;

            float displacement = RaceMath.FlatDistance(egoPos, lastPos);
            float expected = (Math.Abs(egoSpeed) + Math.Abs(lastSpeed)) * 0.5f * dtS;
            float health = -1f;
            float healthDrop = 0f;
            try
            {
                health = vehicle.HealthFloat;
                if (lastHealth > 0f) healthDrop = lastHealth - health;
            }
            catch { }
            bool collided = false;
            ImpactClassifier.TryReadCollision(vehicle, out collided);

            SampleKind kind = ImpactClassifier.Classify(accel, dtS, displacement, expected, healthDrop, collided, egoSpeed);
            lastImpact = kind;
            if ((kind == SampleKind.Impact || kind == SampleKind.Teleport) && now - lastImpactEventMs > 1500)
            {
                lastImpactEventMs = now;
                try
                {
                    if (kind == SampleKind.Impact)
                        telemetry?.Event(now - t0, "IMPACT", $"dec={accel:F0};spd={egoSpeed:F0};dmg={healthDrop:F0};offCorr={corridor.OffCorridor(route.Lateral):F0};slip={slip:F0};yaw={yawRate:F2}");
                    else
                        telemetry?.Event(now - t0, "TELEPORT", $"moved={displacement:F0};exp={expected:F0};spd={egoSpeed:F0}");
                }
                catch { }
            }
            if (kind == SampleKind.Braking && now - lastBrakeEventMs > 3000)
            {
                lastBrakeEventMs = now;
                try { telemetry?.Event(now - t0, "HARD_BRAKE", $"spd={egoSpeed:F0};dec={accel:F0}"); } catch { }
            }

            if (kind == SampleKind.Normal || kind == SampleKind.Braking)
            {
                try { capability.Observe(accel, lastLatAccel, egoSpeed, dtS, yawRate, slip); } catch { }
            }
            if (health >= 0f) lastHealth = health;
        }

        private void EmitTransitionEvents(int t, int now)
        {
            try
            {
                if (lastLoggedTactic != tactics.Mode)
                {
                    if ((int)lastLoggedTactic != -1)
                        telemetry?.Event(t, "TACTIC", $"{lastLoggedTactic}->{tactics.Mode};why={tactics.Reason};lat={tactics.DesiredLateral:F2}");
                    lastLoggedTactic = tactics.Mode;
                }
                if (route.IsLost && !lastLoggedLost)
                    telemetry?.Event(t, "ROUTE_LOST", $"{route.LossReason};prog={route.AlongS:F0};dist={route.DistToRoute:F0}");
                else if (!route.IsLost && lastLoggedLost)
                    telemetry?.Event(t, "ROUTE_FOUND", $"prog={route.AlongS:F0}");
                lastLoggedLost = route.IsLost;
                lastLoggedLossReason = route.LossReason ?? "";
            }
            catch { }
        }

        private void WriteSample(int t, int now, Vector3 egoPos, float egoSpeed)
        {
            if (telemetry == null) return;
            try
            {
                float offCorr = corridor.OffCorridor(route.Lateral);
                float curv = route.CurvatureAhead(80f);
                float aimLat = traj.HasChosen ? traj.Chosen.LateralM : 0f;
                float chScore = traj.HasChosen ? traj.Chosen.Score : 0f;
                float rejLat = 0f;
                float rejScore = 0f;
                if (traj.LastCandidates.Count > 1)
                {
                    rejLat = traj.LastCandidates[1].LateralM;
                    rejScore = traj.LastCandidates[1].Score;
                }
                float nearD = 999f;
                float nearTtc = 999f;
                float nearClose = 0f;
                if (perception.TryGetClosestThreat(out var th))
                {
                    nearD = th.Dist;
                    nearTtc = th.Ttc;
                    nearClose = th.ClosingSpeed;
                }
                else if (perception.AheadCount > 0)
                {
                    nearD = perception.NearestAheadDist;
                    nearTtc = perception.NearestAheadTtc;
                    nearClose = perception.NearestAheadClosing;
                }
                float minHalf = corridor.MinHalfWidthAhead(LookaheadM);
                var pe = actuator != null ? actuator.LastError : new PathFollowingError();
                float minMargin = traj.HasChosen ? traj.Chosen.MinMarginM : 99f;
                float maxKappa = traj.HasChosen ? traj.Chosen.MaxKappa : 0f;
                string chosenReject = traj.HasChosen ? (traj.Chosen.RejectReason ?? "") : "";
                var ch = traj.HasChosen ? traj.Chosen : new TrajectoryCandidate();
                telemetry.Sample(t, style, tactics.Mode.ToString(),
                    route.AlongS, route.Progress01, LookaheadM,
                    route.Lateral, corridor.HalfWidth, offCorr, route.HeadingErrorDeg, curv,
                    aimLat, chScore, rejLat, rejScore,
                    TargetSpeed, egoSpeed, SpeedLimit ?? "Cruise", speedPlan.BrakingNeed,
                    capability.ABrakeMax, capability.ALatMax, perception.Count,
                    nearD, nearTtc, nearClose,
                    actuator.CurrentCruise, actuator.CurrentStyle,
                    route.IsLost ? 1 : 0, lastImpact.ToString(),
                    actuator.ReissueCount, FinishGap,
                    route.Source ?? "?", minHalf,
                    pe.Valid ? pe.LateralErrM : 0f, pe.Valid ? pe.HeadingErrDeg : 0f,
                    pe.Valid ? pe.SpeedErrMps : 0f, pe.Valid ? pe.DistToPathM : 999f,
                    speedPlan.BrakingPointS, capability.Confidence,
                    actuator.ActuatorName ?? "?", chosenReject,
                    minMargin, maxKappa,
                    // Joint-maneuver extensions (appended, legacy cols untouched).
                    ch.CandidateIndex, ch.MeanSpeed, ch.MinSpeed,
                    ch.ConstrainHandle, ch.ConstrainKind ?? "", ch.ConstrainS,
                    ch.MinPredClearance, maneuverPlanId,
                    pe.Valid ? pe.SteerDeg : 0f, pe.Valid ? pe.Throttle01 : 0f,
                    pe.Valid ? pe.Brake01 : 0f, pe.Valid ? pe.LocalTargetMps : TargetSpeed);
            }
            catch { }
        }

        private Vector3 ClampAimToCorridor(Vector3 aim, RaceRoute rt)
        {
            try
            {
                float half = corridor.HalfWidthAt(LookaheadM);
                Vector3 center = rt.LookaheadPoint(LookaheadM);
                float h = rt.HeadingAhead(LookaheadM);
                var dir = RaceMath.VectorFromHeading(h);
                var to = new Vector3(aim.X - center.X, aim.Y - center.Y, 0f);
                float lat = RaceMath.FlatCross(dir, to);
                float lon = RaceMath.FlatDot(to, dir);
                if (Math.Abs(lat) > half + 2f)
                {
                    float cl = RaceMath.Clamp(lat, -(half + 1f), half + 1f);
                    var left = new Vector3(-dir.Y, dir.X, 0f);
                    return new Vector3(center.X + dir.X * lon + left.X * cl,
                        center.Y + dir.Y * lon + left.Y * cl, aim.Z);
                }
            }
            catch { }
            return aim;
        }

        private static float SafeHeading(Vehicle v)
        {
            try { return v.Heading; } catch { return 0f; }
        }

        private Vehicle SafePlayerVehicle()
        {
            try
            {
                var pc = Game.Player.Character;
                if (pc != null && pc.Exists() && pc.IsInVehicle())
                    return pc.CurrentVehicle;
            }
            catch { }
            return null;
        }
    }
}
