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
        private bool lastLoggedInvalid;
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

        // Localization / recovery logging.
        private float lastLocLogS = -9999f;
        private int lastLocLogMs = -100000;
        private int lastMergeLogMs = -100000;
        private float lastMergeLogS = -9999f;
        private Vector3 _lastEgoFwd = new Vector3(0f, 1f, 0f);
        private float _lastEgoHeading;

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

            // --- Full deterministic reset: every race begins clean. RaceBrain
            // is reused across races; without explicit resets the second race
            // inherits Crashed/Recovery, PlanId~344, stale tracks, hysteresis.
            try { route.Reset(); } catch { }
            try { corridor.Reset(); } catch { }
            try { perception.Reset(); } catch { }
            try { traj.Reset(); } catch { }
            try { speedPlan.Reset(); } catch { }
            try { tactics.Reset(t0); } catch { }

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            float originHeading = SafeHeading(vehicle);
            route.Build(origin, finish);
            capability.Seed(vehicle);
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            corridor.Update(route, origin, LookaheadM, t0);
            route.Update(origin, originHeading, 0f, t0, corridor.HalfWidth);

            actuator = useDirect ? (IVehicleActuator)new DirectActuator() : (IVehicleActuator)new GtaDriverActuator();
            actuator.Attach(driver, vehicle, cruise, style, refreshMs, stuckMs);
            viz.Enabled = debugViz;

            // Kinematics / scheduling / logging: race-local, always reset.
            hasKin = false;
            lastSpeed = 0f;
            lastPos = origin;
            lastVel = new Vector3();
            lastHeading = originHeading;
            lastKinT = t0;
            lastHealth = -1f;
            lastImpact = SampleKind.Normal;
            lastImpactEventMs = -100000;
            lastAccelLong = 0f;
            lastLatAccel = 0f;
            lastYawRate = 0f;
            lastSlipDeg = 0f;
            lastPercMs = 0;
            lastCorrMs = 0;
            lastPlanMs = 0;
            lastTeleMs = 0;
            lastGpsRetryMs = 0;
            TargetSpeed = 0f;
            SpeedLimit = "Cruise";
            ActualSpeed = 0f;
            FinishGap = RaceMath.FlatDistance(origin, finish);
            lastLoggedTactic = (TacticalMode)(-1);
            lastLoggedLost = false;
            lastLoggedLossReason = "";
            lastLoggedInvalid = false;
            lastBrakeEventMs = -100000;
            lastPathErrLat = 0f;
            lastPathErrHead = 0f;
            lastPathErrLogMs = 0;
            maneuverPlanId = 0;
            lastPlanLogId = -1;
            lastChosenLat = 999f;
            lastChosenSpeed = -1f;
            lastChosenLimit = "";
            pendingForceReissue = true;
            lastLocLogS = -9999f;
            lastLocLogMs = -100000;
            lastMergeLogMs = -100000;
            lastMergeLogS = -9999f;
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
                // Pose-foundation invariant at start: every candidate must
                // begin approximately along the rival heading on a straight
                // road. Log the full pose so straight-road tests report
                // heading / route heading / error / progress from tick 0.
                try
                {
                    Vector3 fwd0 = RaceMath.VectorFromHeading(originHeading);
                    float rh0 = route.RouteHeadingDeg;
                    string vStart = "";
                    try { vStart = route.ValidateStart(origin, originHeading, out string vr) ? $"valid;{vr}" : $"INVALID;{vr}"; }
                    catch { vStart = "validate-exc"; }
                    telemetry?.Event(0, "START_POSE", $"egoHead={originHeading:F0};routeHead={rh0:F0};headErr={route.HeadingErrorDeg:F0};lat={route.Lateral:F1};dist={route.DistToRoute:F1};s={route.AlongS:F0};exp={route.ExpectedS:F0};loc={route.LocDetail};startValid={vStart}");
                    // Next ~50-100 m of route for geometry audit.
                    try
                    {
                        string rp = "";
                        for (float d = 0f; d <= 100f; d += 10f)
                        {
                            Vector3 p = route.PointAtS(route.AlongS + d);
                            rp += $"{d:F0}:({p.X:F0},{p.Y:F0}) ";
                        }
                        telemetry?.Event(0, "ROUTE_AHEAD", rp.Trim());
                    }
                    catch { }
                    _lastEgoFwd = fwd0;
                    _lastEgoHeading = originHeading;
                }
                catch { }
            }
            catch { }
        }

        public bool IsStartPoseValid(out string reason)
        {
            reason = "unknown";
            try
            {
                if (vehicle == null || !vehicle.Exists()) { reason = "no-vehicle"; return false; }
                Vector3 p = vehicle.Position;
                float h = SafeHeading(vehicle);
                return route.ValidateStart(p, h, out reason);
            }
            catch (Exception ex) { try { reason = "exc:" + ex.Message; } catch { } return false; }
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
            try { _lastEgoFwd = egoFwd; _lastEgoHeading = egoHeading; } catch { }

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
                try { _lastEgoFwd = egoFwd; _lastEgoHeading = egoHeading; } catch { }

                // Localization audit: log significant jumps/ambiguity with
                // full pose so a sideways route is visible before any crash.
                try
                {
                    bool jumpBig = Math.Abs(route.LocJumpM) > 6f;
                    bool logLoc = jumpBig || route.LocAmbiguous || lastLocLogS < -9000f;
                    if (logLoc && now - lastLocLogMs > 400)
                    {
                        lastLocLogMs = now;
                        lastLocLogS = route.AlongS;
                        telemetry?.Event(t, "LOC", $"egoHead={egoHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={route.HeadingErrorDeg:F0};seg={route.NearestIndex};prevS={route.PrevAlongS:F0};newS={route.AlongS:F0};expS={route.ExpectedS:F0};jump={route.LocJumpM:F1};dist={route.DistToRoute:F1};lat={route.Lateral:F1};{route.LocDetail}");
                        if (route.LocAmbiguous)
                        {
                            try
                            {
                                string rp = "";
                                for (float d = 0f; d <= 60f; d += 10f)
                                {
                                    Vector3 p = route.PointAtS(route.AlongS + d);
                                    rp += $"{d:F0}:({p.X:F0},{p.Y:F0}) ";
                                }
                                telemetry?.Event(t, "ROUTE_AHEAD", rp.Trim());
                            }
                            catch { }
                        }
                    }
                    else if (now - lastLocLogMs > 5000)
                    {
                        // Heartbeat so progress stability over the first 3 s
                        // is auditable even without jumps.
                        lastLocLogMs = now;
                        lastLocLogS = route.AlongS;
                    }
                }
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
                    // Hard route/plan validity invariant: never issue a normal
                    // 15-20 m/s maneuver when the local route tangent is
                    // grossly incompatible with vehicle heading. That state
                    // means wrong-branch localization or a required special
                    // merge — crawl via pose-aware recovery instead.
                    bool headingInvalid = false;
                    try { headingInvalid = route.Built && Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg; }
                    catch { headingInvalid = false; }
                    bool needRecovery = route.IsLost || headingInvalid
                        || tactics.Mode == TacticalMode.Recovery || tactics.Mode == TacticalMode.Crashed;
                    if (needRecovery)
                    {
                        string mergeDetail = "none";
                        Vector3 mergePt = route.RecoveryTarget();
                        float mergeS = route.AlongS + 40f;
                        bool haveMerge = false;
                        try { haveMerge = route.TryGetRecoveryMerge(egoPos, egoHeading, out mergePt, out mergeS, out mergeDetail); }
                        catch { haveMerge = false; }
                        try
                        {
                            if (haveMerge && (now - lastMergeLogMs > 1000 || Math.Abs(mergeS - lastMergeLogS) > 5f))
                            {
                                lastMergeLogMs = now;
                                lastMergeLogS = mergeS;
                                telemetry?.Event(t, "RECOVERY_MERGE", $"have=1;{mergeDetail};headErr={route.HeadingErrorDeg:F0};dist={route.DistToRoute:F1};s={route.AlongS:F0};loss={route.LossReason}");
                            }
                            else if (!haveMerge && now - lastMergeLogMs > 2000)
                            {
                                lastMergeLogMs = now;
                                telemetry?.Event(t, "RECOVERY_MERGE", $"have=0;{mergeDetail};headErr={route.HeadingErrorDeg:F0};dist={route.DistToRoute:F1};s={route.AlongS:F0};loss={route.LossReason}");
                            }
                        }
                        catch { }
                        TrajectoryCandidate rec = new TrajectoryCandidate();
                        bool recOk = false;
                        string recWhy = "";
                        try { recOk = TryBuildRecoveryConnector(egoPos, egoFwd, egoHeading, egoSpeed, mergePt, mergeS, haveMerge, mergeDetail, headingInvalid, out rec, out recWhy); }
                        catch { recOk = false; }
                        if (recOk)
                        {
                            chosen = rec;
                            traj.LastCandidates.Clear();
                            traj.LastCandidates.Add(chosen);
                            traj.Chosen = chosen;
                            traj.HasChosen = true;
                            traj.BrakingPointS = -1f;
                            traj.PlanId++;
                            target = chosen.TargetSpeed;
                            limiting = chosen.SpeedLimiting;
                            recoveredTarget = true;
                        }
                        else
                        {
                            // Infeasible merge (U-turn-like) or no merge:
                            // hold position safely, never command cruise
                            // into a sideways route.
                            try { telemetry?.Event(t, "ROUTE_INVALID", $"hold;why={recWhy};headErr={route.HeadingErrorDeg:F0};seg={route.NearestIndex};s={route.AlongS:F0};dist={route.DistToRoute:F1};loss={route.LossReason};{route.LocDetail}"); } catch { }
                            var holdPath = new System.Collections.Generic.List<Vector3>
                            {
                                egoPos,
                                new Vector3(egoPos.X + egoFwd.X * 12f, egoPos.Y + egoFwd.Y * 12f, egoPos.Z)
                            };
                            float holdV = 0f;
                            chosen = new TrajectoryCandidate
                            {
                                LateralM = 0f,
                                LookaheadM = 12f,
                                AimPoint = holdPath[1],
                                Score = -99f,
                                ClearanceM = 999f,
                                CurveCost = 0f,
                                TacticalBias = 0f,
                                RejectReason = string.IsNullOrEmpty(recWhy) ? "recovery-hold" : recWhy,
                                Path = holdPath,
                                StationS = new System.Collections.Generic.List<float> { 0f, 12f },
                                SpeedProfile = new System.Collections.Generic.List<float> { holdV, holdV },
                                ArrivalT = new System.Collections.Generic.List<float> { 0f, 4f },
                                MinMarginM = 99f,
                                MaxKappa = 0f,
                                TargetSpeed = holdV,
                                SpeedLimiting = headingInvalid ? "PoseHold" : "RecoveryHold",
                                ConstrainHandle = -1,
                                ConstrainKind = "",
                                ConstrainS = -1f,
                                MinPredClearance = 999f,
                                MeanSpeed = holdV,
                                MinSpeed = holdV,
                                RequiredDecel = 0f,
                                CandidateIndex = -2,
                                FirstTangentErrDeg = 0f,
                                RouteHeadErrDeg = route.HeadingErrorDeg,
                            };
                            traj.LastCandidates.Clear();
                            traj.LastCandidates.Add(chosen);
                            traj.Chosen = chosen;
                            traj.HasChosen = true;
                            traj.BrakingPointS = -1f;
                            traj.PlanId++;
                            target = holdV;
                            limiting = chosen.SpeedLimiting;
                            recoveredTarget = true;
                        }
                    }
                    else
                    {
                        // JOINT plan: path + speed together. No global
                        // corridor-wide obstacle speed afterwards.
                        chosen = traj.PlanJoint(route, corridor, perception, tactics, profile,
                            capability, egoPos, egoFwd, egoSpeed, LookaheadM, cruiseSetting);
                        target = chosen.TargetSpeed;
                        limiting = string.IsNullOrEmpty(chosen.SpeedLimiting) ? "Cruise" : chosen.SpeedLimiting;
                        // Double-guard: planner must never return cruise when
                        // the route just went heading-invalid between Update
                        // and Plan. Force a crawl hold instead.
                        if (Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg && target > 6f)
                        {
                            try { telemetry?.Event(t, "ROUTE_INVALID", $"clamp-cruise;headErr={route.HeadingErrorDeg:F0};wasV={target:F1};seg={route.NearestIndex};{route.LocDetail}"); } catch { }
                            target = 4f;
                            limiting = "PoseHold";
                            var cg = chosen;
                            cg.TargetSpeed = target;
                            cg.SpeedLimiting = limiting;
                            chosen = cg;
                            traj.Chosen = cg;
                        }
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
                                + $"meanV={ch.MeanSpeed:F1};minV={ch.MinSpeed:F1};prof={prof};score={ch.Score:F2};why={what.Trim()};"
                                + $"egoHead={egoHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={route.HeadingErrorDeg:F0};"
                                + $"firstTangErr={ch.FirstTangentErrDeg:F1};maxKappa={ch.MaxKappa:F4};seg={route.NearestIndex};s={route.AlongS:F0};expS={route.ExpectedS:F0};{route.LocDetail}";
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
                bool curInvalid = false;
                try { curInvalid = route.Built && Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg; } catch { }
                if (curInvalid != lastLoggedInvalid) forceReissue = true;
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
                    viz.Draw(route, corridor, traj, perception, speedPlan, egoPos, egoFwd, egoSpeed, LookaheadM, TargetSpeed);
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
                bool curInvalid = false;
                try { curInvalid = route.Built && Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg; } catch { }
                if (curInvalid && !lastLoggedInvalid)
                    telemetry?.Event(t, "ROUTE_INVALID", $"headErr={route.HeadingErrorDeg:F0};seg={route.NearestIndex};s={route.AlongS:F0};dist={route.DistToRoute:F1};{route.LocDetail}");
                else if (!curInvalid && lastLoggedInvalid)
                    telemetry?.Event(t, "ROUTE_VALID", $"headErr={route.HeadingErrorDeg:F0};s={route.AlongS:F0}");
                lastLoggedInvalid = curInvalid;
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
                float egoHeadLog = 0f;
                float routeHeadLog = 0f;
                float firstTangLog = 0f;
                try { egoHeadLog = _lastEgoHeading; } catch { }
                try { routeHeadLog = route.RouteHeadingDeg; } catch { }
                try { firstTangLog = ch.FirstTangentErrDeg; } catch { }
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
                    pe.Valid ? pe.Brake01 : 0f, pe.Valid ? pe.LocalTargetMps : TargetSpeed,
                    egoHeadLog, routeHeadLog, firstTangLog,
                    route.ExpectedS, route.LocJumpM);
            }
            catch { }
        }

        /// Pose-aware low-speed recovery connector to a heading-compatible
        /// merge station. Uses the SAME Hermite continuity as normal
        /// candidates (d(0)=current lateral, d'(0)=-tan(headErr), d(S)=0,
        /// d'(S)=0) so recovery never commands an instantaneous heading
        /// change. Rejects U-turn-like merges (huge initial curvature).
        private bool TryBuildRecoveryConnector(Vector3 egoPos, Vector3 egoFwd, float egoHeading,
            float egoSpeed, Vector3 mergePt, float mergeS, bool haveMerge, string mergeDetail,
            bool headingInvalid, out TrajectoryCandidate rec, out string why)
        {
            rec = new TrajectoryCandidate();
            why = "";
            try
            {
                if (!haveMerge)
                {
                    why = $"no-merge;{mergeDetail}";
                    return false;
                }
                float lookahead = mergeS - route.AlongS;
                if (lookahead < 15f) lookahead = 15f;
                if (lookahead > 80f) lookahead = 80f;
                float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
                float headErr = route.HeadingErrorDeg;
                System.Collections.Generic.List<float> pathLats;
                System.Collections.Generic.List<float> pathS;
                var path = TrajectoryPlanner.BuildPoseAwarePath(route, egoPos, startLat, headErr, 0f,
                    lookahead, 5f, out pathLats, out pathS);
                if (path == null || path.Count < 3)
                {
                    why = "short-path";
                    return false;
                }
                var kappa = new System.Collections.Generic.List<float>(path.Count);
                for (int i = 0; i < path.Count; i++) kappa.Add(0f);
                float maxKappa = 0f;
                float minMargin = float.MaxValue;
                for (int k = 0; k < path.Count; k++)
                {
                    float s = pathS[k];
                    float half = corridor.HalfWidthAt(s);
                    float margin = half - Math.Abs(pathLats[k]);
                    if (margin < minMargin) minMargin = margin;
                    if (k >= 1)
                    {
                        var d0 = new Vector3(path[k].X - path[k - 1].X, path[k].Y - path[k - 1].Y, 0f);
                        Vector3 d1 = d0;
                        if (k + 1 < path.Count)
                            d1 = new Vector3(path[k + 1].X - path[k].X, path[k + 1].Y - path[k].Y, 0f);
                        float l0 = RaceMath.FlatLength(d0);
                        float l1 = RaceMath.FlatLength(d1);
                        if (l0 > 0.5f && l1 > 0.5f)
                        {
                            float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
                            float kk = dh / Math.Max((l0 + l1) * 0.5f, 1f);
                            kappa[k] = kk;
                            if (kk > maxKappa) maxKappa = kk;
                        }
                    }
                }
                float firstTang = TrajectoryPlanner.FirstTangentErrorDeg(path, egoFwd);
                // U-turn / impossible merge: huge curvature even at crawl.
                // At 5 m/s, kappa 0.15 needs 3.75 m/s^2 — feasible but sharp;
                // beyond 0.25 (4 m radius) it is a spin, not a merge.
                if (maxKappa > 0.25f)
                {
                    why = $"infeasible-kappa maxKappa={maxKappa:F3} firstTang={firstTang:F0} {mergeDetail}";
                    return false;
                }
                if (minMargin < -3f)
                {
                    why = $"offroad margin={minMargin:F1}";
                    return false;
                }
                float aLatRaw = capability.UsableLat(profile.GripFactor);
                float cc = profile.CornerCaution;
                if (cc < 0.5f) cc = 0.5f;
                if (cc > 2f) cc = 2f;
                float aLatEff = aLatRaw / (cc * cc);
                float aBrake = capability.UsableBrake(profile.GripFactor);
                float topSpeed = 60f;
                try { topSpeed = capability.TopSpeedEst; } catch { }
                float recCruise = Math.Min(cruiseSetting, headingInvalid ? 5f : 8f);
                float[] vAllow;
                float[] vTgt;
                float[] arrivalT;
                int cHandle;
                string cKind;
                float cS;
                float minPredClear;
                try
                {
                    SpeedPlanner.ProfileForPath(path, pathS, kappa, pathLats, route.AlongS,
                        perception, corridor, egoSpeed, recCruise, aLatEff, aBrake, topSpeed,
                        profile, recCruise,
                        out vAllow, out vTgt, out arrivalT, out cHandle, out cKind, out cS, out minPredClear);
                }
                catch
                {
                    why = "speed-prof-exc";
                    return false;
                }
                float meanV = 0f;
                float minV = float.MaxValue;
                for (int i = 0; i < vTgt.Length; i++) { meanV += vTgt[i]; if (vTgt[i] < minV) minV = vTgt[i]; }
                if (vTgt.Length > 0) meanV /= vTgt.Length; else { meanV = recCruise; minV = recCruise; }
                float targetNow = vTgt.Length > 0 ? vTgt[0] : recCruise;
                rec = new TrajectoryCandidate
                {
                    LateralM = 0f,
                    LookaheadM = lookahead,
                    AimPoint = path[path.Count - 1],
                    Score = 0f,
                    ClearanceM = minPredClear,
                    CurveCost = 0f,
                    TacticalBias = 0f,
                    RejectReason = "",
                    Path = path,
                    MinMarginM = minMargin == float.MaxValue ? 99f : minMargin,
                    MaxKappa = maxKappa,
                    StationS = new System.Collections.Generic.List<float>(pathS),
                    SpeedProfile = new System.Collections.Generic.List<float>(vTgt),
                    ArrivalT = new System.Collections.Generic.List<float>(arrivalT),
                    TargetSpeed = targetNow,
                    SpeedLimiting = headingInvalid ? "PoseMerge" : "RecoveryMerge",
                    ConstrainHandle = cHandle,
                    ConstrainKind = cKind ?? "",
                    ConstrainS = cS,
                    MinPredClearance = minPredClear,
                    MeanSpeed = meanV,
                    MinSpeed = minV == float.MaxValue ? recCruise : minV,
                    RequiredDecel = 0f,
                    CandidateIndex = -1,
                    FirstTangentErrDeg = firstTang,
                    RouteHeadErrDeg = headErr,
                };
                why = $"ok mergeS={mergeS:F0} maxKappa={maxKappa:F3} firstTang={firstTang:F1} v={targetNow:F1} {mergeDetail}";
                return true;
            }
            catch (Exception ex)
            {
                try { why = "exc:" + ex.Message; } catch { why = "exc"; }
                return false;
            }
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
