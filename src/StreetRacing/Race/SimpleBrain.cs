using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using StreetRacing.Control;
using StreetRacing.Debug;

namespace StreetRacing.Race
{
    /// Minimal competent driver (collapsed Phases 2-5 foundation).
    ///
    /// Pipeline (deliberately dumb):
    ///   GPS route -> stable route localization -> ONE pose-feasible center
    ///   trajectory (PoseConnector, correct +tan sign) -> fixed/moderate
    ///   curvature-capped speed -> Direct.
    ///
    /// DISABLED here by construction (not tuned around):
    ///   seven lateral candidates, opponent tactics, overtaking racecraft,
    ///   collision-avoidance speed planner, Crashed state, dynamic recovery
    ///   FSM, civilian traffic behavior (unless EnablePassing=1 for the
    ///   single-blocker Phase-5 scenario).
    ///
    /// Intent (Phase 4): KEEP_LINE default, FOLLOW/PASS_LEFT/PASS_RIGHT for
    /// one civilian blocker when enabled, RECOVER for the explicit recovery
    /// primitive. Intent persists with dwell hysteresis; the trajectory is
    /// generated INSIDE the intent, never by rescoring the whole road width.
    ///
    /// Success criterion: 1-2 km of ordinary route at ~15-20 m/s without
    /// leaving the road, stopping for no reason, oscillating, or needing
    /// special modes.
    internal sealed class SimpleBrain
    {
        private Ped driver;
        private Vehicle vehicle;
        private Vector3 finish = Vector3.Zero;
        private float cruiseSetting = 18f;
        private float simpleCruiseCap = 18f;
        private int style;
        private DriverProfile profile;
        private RaceTelemetry telemetry;
        private int t0;
        private bool enablePassing;
        private bool useGtaRejoin;

        private readonly RaceRoute route = new RaceRoute();
        private readonly RoadCorridor corridor = new RoadCorridor();
        private readonly VehicleCapability capability = new VehicleCapability();
        // Viz containers only: single center candidate is published here so
        // the existing RaceDebugViz draws it without changes. No scoring.
        private readonly TrajectoryPlanner trajViz = new TrajectoryPlanner();
        private readonly SpeedPlanner speedViz = new SpeedPlanner();
        private readonly RaceDebugViz viz = new RaceDebugViz();
        private readonly ManeuverIntentState intent = new ManeuverIntentState();
        private readonly RecoveryPrimitive recovery = new RecoveryPrimitive();
        private IVehicleActuator actuator;

        private int lastCorrMs;
        private int lastPlanMs;
        private int lastTeleMs;
        private int lastGpsRetryMs;

        private float lastSpeed;
        private Vector3 lastPos = Vector3.Zero;
        private bool hasKin;
        private int lastKinT;
        private float lastHealth = -1f;
        private float lastAccelLong;
        private int lastImpactEventMs = -100000;
        private float lastSlipDeg;
        private float lastYawRate;

        private TrajectoryCandidate current = new TrajectoryCandidate();
        private bool hasCurrent;
        private int maneuverPlanId;
        private string lastIntentLog = "";
        private int lastIntentLogMs = -100000;
        private Vector3 lastEgoFwd = new Vector3(0f, 1f, 0f);
        private float lastEgoHeading;

        // Single-blocker pass state (Phase 5, gated).
        private int blockerHandle;
        private float blockerLat;
        private float blockerDist;
        private float blockerSpeedAlong;

        public bool Running { get; private set; }
        public string TacticalName => intent.Current.ToString();
        public float TargetSpeed { get; private set; }
        public string SpeedLimit { get; private set; } = "Cruise";
        public float ActualSpeed { get; private set; }
        public float FinishGap { get; private set; }
        public float Progress01 => route.Progress01;
        public bool RouteLost => route.IsLost;
        public float LookaheadM { get; private set; } = 70f;
        public string RouteSource => route.Source;
        public string ActuatorName => actuator != null ? actuator.ActuatorName : "?";

        public void Start(Ped driver, Vehicle vehicle, Vector3 finish, float cruise,
            int style, DriverProfile profile, RaceTelemetry telemetry,
            int refreshMs, int stuckMs, bool debugViz,
            float simpleCruiseCap, bool enablePassing, bool useGtaRejoin)
        {
            this.driver = driver;
            this.vehicle = vehicle;
            this.finish = finish;
            this.cruiseSetting = cruise;
            this.simpleCruiseCap = simpleCruiseCap > 1f ? simpleCruiseCap : 18f;
            this.style = style;
            this.profile = profile ?? DriverProfile.FromName("balanced");
            this.telemetry = telemetry;
            this.enablePassing = enablePassing;
            this.useGtaRejoin = useGtaRejoin;
            t0 = Game.GameTime;

            try { route.Reset(); } catch { }
            try { corridor.Reset(); } catch { }
            try { trajViz.Reset(); } catch { }
            try { speedViz.Reset(); } catch { }
            try { intent.Reset(t0); } catch { }
            try { recovery.Reset(t0); } catch { }

            Vector3 origin;
            try { origin = vehicle.Position; } catch { origin = Game.Player.Character.Position; }
            float originHeading = SafeHeading(vehicle);
            route.Build(origin, finish);
            capability.Seed(vehicle);
            LookaheadM = this.profile.LookaheadForSpeed(0f);
            try { corridor.Update(route, origin, LookaheadM, t0); } catch { }
            try { route.Update(origin, originHeading, 0f, t0, corridor.HalfWidth); } catch { }

            actuator = new DirectActuator();
            actuator.Attach(driver, vehicle, EffectiveCruise(), style, refreshMs, stuckMs);
            viz.Enabled = debugViz;

            hasKin = false;
            lastSpeed = 0f;
            lastPos = origin;
            lastKinT = t0;
            lastHealth = -1f;
            lastAccelLong = 0f;
            lastImpactEventMs = -100000;
            lastCorrMs = 0;
            lastPlanMs = 0;
            lastTeleMs = 0;
            lastGpsRetryMs = 0;
            TargetSpeed = 0f;
            SpeedLimit = "Cruise";
            ActualSpeed = 0f;
            FinishGap = RaceMath.FlatDistance(origin, finish);
            maneuverPlanId = 0;
            hasCurrent = false;
            lastIntentLog = "";
            lastIntentLogMs = -100000;
            blockerHandle = 0;
            Running = true;

            try
            {
                string chk = PoseConnector.SelfTest();
                telemetry?.Event(0, "ROUTE", $"src={route.Source};pts={route.Points.Count};len={route.TotalLength:F0};simpleCruise={EffectiveCruise():F0};poseCheck={chk}");
                telemetry?.Event(0, "ACTUATOR", $"Direct;simple dumb follower (1 center path, no candidates/tactics/traffic);passing={(enablePassing ? "SINGLE-BLOCKER-ON" : "OFF")}");
                string vStart = "";
                try { vStart = route.ValidateStart(origin, originHeading, out string vr) ? $"valid;{vr}" : $"INVALID;{vr}"; }
                catch { vStart = "validate-exc"; }
                telemetry?.Event(0, "START_POSE", $"egoHead={originHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={route.HeadingErrorDeg:F0};lat={route.Lateral:F1};dist={route.DistToRoute:F1};s={route.AlongS:F0};startValid={vStart}");
                lastEgoFwd = RaceMath.VectorFromHeading(originHeading);
                lastEgoHeading = originHeading;
            }
            catch { }
        }

        public bool IsStartPoseValid(out string reason)
        {
            reason = "unknown";
            try
            {
                if (vehicle == null || !vehicle.Exists()) { reason = "no-vehicle"; return false; }
                return route.ValidateStart(vehicle.Position, SafeHeading(vehicle), out reason);
            }
            catch (Exception ex) { try { reason = "exc:" + ex.Message; } catch { } return false; }
        }

        public bool Valid()
        {
            try { return Running && actuator != null && actuator.Valid(); }
            catch { return false; }
        }

        public void Stop()
        {
            Running = false;
            try { actuator?.Stop(); } catch { }
        }

        private float EffectiveCruise()
        {
            float c = Math.Min(cruiseSetting, simpleCruiseCap);
            if (c < 5f) c = 5f;
            if (c > 25f) c = 25f;
            return c;
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
            lastEgoFwd = egoFwd;
            lastEgoHeading = egoHeading;

            UpdateKinematics(now, egoPos, egoVel, egoSpeed, egoHeading);

            if (!hasKin)
            {
                hasKin = true;
                lastSpeed = egoSpeed;
                lastPos = egoPos;
                lastKinT = now;
                try { lastHealth = vehicle.HealthFloat; } catch { lastHealth = -1f; }
                return;
            }

            bool doPlan = now - lastPlanMs >= profile.ReactionIntervalMs;
            if (doPlan)
            {
                lastPlanMs = now;
                try { route.Update(egoPos, egoHeading, egoSpeed, now, corridor.HalfWidth); } catch { }
                try
                {
                    if (!route.Source.StartsWith("Gps") && now - t0 < 12000 && now - lastGpsRetryMs > 1000)
                    {
                        lastGpsRetryMs = now;
                        string ulog;
                        if (route.TryUpgradeToGps(egoPos, egoHeading, egoSpeed, now, corridor.HalfWidth, out ulog))
                        {
                            try { corridor.Update(route, egoPos, LookaheadM, now); } catch { }
                            try { telemetry?.Event(t, "GPS_ROUTE", $"upgraded;{ulog}"); } catch { }
                            try { hasCurrent = false; } catch { }
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
                LookaheadM = profile.LookaheadForSpeed(egoSpeed);
                // --- Corroborated contact evidence for recovery entry.
                bool hasCollided = false;
                float healthDrop = 0f;
                try { hasCollided = vehicle.HasCollided; } catch { }
                try
                {
                    float h = vehicle.HealthFloat;
                    if (lastHealth > 0f) healthDrop = lastHealth - h;
                }
                catch { }

                bool wantRecovery = false;
                string recWhy = "";
                try
                {
                    if (recovery.ShouldEnter(route, egoSpeed, route.AlongS, hasCollided, healthDrop, lastAccelLong, now))
                    {
                        wantRecovery = true;
                        if (route.IsLost) recWhy = "route:" + route.LossReason;
                        else if (Math.Abs(route.HeadingErrorDeg) > RaceRoute.PlanInvalidHeadErrDeg) recWhy = $"headErr {route.HeadingErrorDeg:F0}";
                        else if (hasCollided || healthDrop >= 4f) recWhy = "contact-corroborated";
                        else recWhy = "no-progress";
                    }
                }
                catch { }

                ManeuverCommand m;
                if (wantRecovery && !recovery.Active)
                {
                    try { recovery.Enter(recWhy, now, route.AlongS); } catch { }
                    try { intent.Force(ManeuverIntent.RECOVER, recWhy, now); } catch { }
                    try { telemetry?.Event(t, "RECOVER_ENTER", $"{recWhy};s={route.AlongS:F0};headErr={route.HeadingErrorDeg:F0}"); } catch { }
                }

                if (recovery.Active)
                {
                    try { intent.Force(ManeuverIntent.RECOVER, recovery.Reason, now); } catch { }
                    float recCruise = Math.Min(EffectiveCruise(), 8f);
                    m = recovery.Tick(route, corridor, egoPos, egoFwd, egoHeading, egoSpeed, route.AlongS, now, recCruise);
                    m.Style = style;
                    m.PlanId = ++maneuverPlanId;
                    // Exit when route is healthy and we are moving again.
                    bool healthy = !route.IsLost && Math.Abs(route.HeadingErrorDeg) <= RaceRoute.PlanInvalidHeadErrDeg;
                    if (healthy && egoSpeed > 3f && (now - recovery.SinceMs) > 1500)
                    {
                        // Require a heading-compatible future merge before exit
                        // so we rejoin, not just resume into a sideways route.
                        try
                        {
                            Vector3 mp;
                            float ms;
                            string md;
                            if (route.TryGetRecoveryMerge(egoPos, egoHeading, out mp, out ms, out md))
                            {
                                recovery.Exit(now);
                                intent.Force(ManeuverIntent.KEEP_LINE, "recovered", now);
                                try { telemetry?.Event(t, "RECOVER_EXIT", $"s={route.AlongS:F0};{md}"); } catch { }
                                m = BuildKeepLine(egoPos, egoSpeed);
                                m.PlanId = ++maneuverPlanId;
                            }
                        }
                        catch { }
                    }
                    TargetSpeed = Math.Max(0f, m.TargetSpeed);
                    SpeedLimit = m.Reason ?? "Recovery";
                    PublishViz(m, egoSpeed, t, egoHeading);
                    try { actuator.SetManeuver(m); } catch { }
                    LogIntent(t, now);
                }
                else
                {
                    // --- Normal: ONE dumb center trajectory inside KEEP_LINE,
                    // or single-blocker FOLLOW/PASS when enabled (Phase 5).
                    UpdateBlockerIntent(egoPos, egoSpeed, now, t);
                    m = intent.Current == ManeuverIntent.FOLLOW
                        ? BuildFollow(egoPos, egoSpeed)
                        : intent.Current == ManeuverIntent.PASS_LEFT || intent.Current == ManeuverIntent.PASS_RIGHT
                            ? BuildPass(egoPos, egoSpeed)
                            : BuildKeepLine(egoPos, egoSpeed);
                    m.PlanId = ++maneuverPlanId;
                    TargetSpeed = Math.Max(0f, m.TargetSpeed);
                    SpeedLimit = m.Reason ?? "Cruise";
                    PublishViz(m, egoSpeed, t, egoHeading);
                    try { actuator.SetManeuver(m); } catch { }
                    LogIntent(t, now);
                }
            }

            try { actuator.OnTick(false, intent.Current.ToString()); } catch { }
            try
            {
                actuator?.UpdatePathError(egoPos, egoHeading, egoSpeed, route,
                    hasCurrent ? current : new TrajectoryCandidate(), hasCurrent, TargetSpeed);
            }
            catch { }
            try
            {
                if (viz.Enabled)
                    viz.Draw(route, corridor, trajViz, null, speedViz, egoPos, lastEgoFwd, egoSpeed, LookaheadM, TargetSpeed);
            }
            catch { }

            if (now - lastTeleMs >= 100)
            {
                lastTeleMs = now;
                WriteSample(t, egoPos, egoSpeed);
            }

            lastSpeed = egoSpeed;
            lastPos = egoPos;
            lastKinT = now;
        }

        // --- Single center path: start from actual pose, continuous forward,
        // endLat=0 (center), fixed/moderate speed capped by curvature only.
        private ManeuverCommand BuildKeepLine(Vector3 egoPos, float egoSpeed)
        {
            float cruise = EffectiveCruise();
            float look = LookaheadM;
            float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
            float headErr = route.HeadingErrorDeg;
            List<float> lats;
            List<float> ss;
            var path = PoseConnector.BuildPath(route, egoPos, startLat, headErr, 0f, look, 5f, out lats, out ss);
            float firstTang = PoseConnector.FirstTangentErrorDeg(path, lastEgoHeading);
            // Deterministic guard: connector must leave toward the nose.
            // If it points away, the sign/conventions regressed — log loudly
            // and fall back to a straight nose-hold instead of driving it.
            try
            {
                if (!PoseConnector.VerifyToward(lastEgoHeading, route.RouteHeadingDeg, firstTang))
                {
                    try { telemetry?.Event(Game.GameTime - t0, "POSE_CONNECTOR_FAIL", $"egoHead={lastEgoHeading:F0};routeHead={route.RouteHeadingDeg:F0};headErr={headErr:F0};firstTang={firstTang:F0}"); } catch { }
                    var hold = new List<Vector3> { egoPos, new Vector3(egoPos.X + lastEgoFwd.X * 12f, egoPos.Y + lastEgoFwd.Y * 12f, egoPos.Z) };
                    return new ManeuverCommand
                    {
                        Path = hold,
                        StationS = new List<float> { 0f, 12f },
                        SpeedProfile = new List<float> { Math.Min(cruise, 5f), Math.Min(cruise, 5f) },
                        AimPoint = hold[1],
                        TargetSpeed = Math.Min(cruise, 5f),
                        Style = style,
                        Reason = "PoseConnectorFail",
                        Reverse = false,
                    };
                }
            }
            catch { }

            // Curvature-only speed: v=sqrt(aLat/k), backwards braking pass.
            float aLat = capability.UsableLat(profile.GripFactor);
            float aBrake = capability.UsableBrake(profile.GripFactor);
            float top = 60f;
            try { top = capability.TopSpeedEst; } catch { }
            int n = path.Count;
            var vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureOfPathAt(path, i);
                float vc = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                if (vc > cruise) vc = cruise;
                if (top > 5f && vc > top) vc = top;
                vAllow[i] = vc;
            }
            var vTgt = new float[n];
            if (n > 0)
            {
                vTgt[n - 1] = vAllow[n - 1];
                for (int i = n - 2; i >= 0; i--)
                {
                    float ds = Math.Max(1f, ss[i + 1] - ss[i]);
                    float vr = (float)Math.Sqrt(vTgt[i + 1] * vTgt[i + 1] + 2f * aBrake * ds);
                    vTgt[i] = Math.Min(vAllow[i], vr);
                }
                // Forward accel feasibility.
                float aAcc = Math.Max(2f, aBrake * 0.55f);
                float v0 = vTgt[0] > egoSpeed ? Math.Min(vTgt[0], egoSpeed + 1f) : vTgt[0];
                vTgt[0] = v0;
                for (int i = 1; i < n; i++)
                {
                    float ds = Math.Max(1f, ss[i] - ss[i - 1]);
                    float vr = (float)Math.Sqrt(vTgt[i - 1] * vTgt[i - 1] + 2f * aAcc * ds);
                    if (vTgt[i] > vr) vTgt[i] = vr;
                }
            }
            var prof = new List<float>(vTgt);
            float targetNow = prof.Count > 0 ? prof[0] : cruise;
            float maxKappa = 0f;
            for (int i = 0; i < n; i++) { float k = CurvatureOfPathAt(path, i); if (k > maxKappa) maxKappa = k; }
            current = new TrajectoryCandidate
            {
                LateralM = 0f,
                LookaheadM = look,
                AimPoint = path.Count > 0 ? path[path.Count - 1] : egoPos,
                Score = 0f,
                ClearanceM = 999f,
                CurveCost = 0f,
                TacticalBias = 0f,
                RejectReason = "",
                Path = path,
                MinMarginM = MinMargin(lats, ss),
                MaxKappa = maxKappa,
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                ArrivalT = new List<float>(ss.Count),
                TargetSpeed = targetNow,
                SpeedLimiting = targetNow < cruise - 0.5f ? "Curvature" : "Cruise",
                ConstrainHandle = -1,
                ConstrainKind = "",
                ConstrainS = -1f,
                MinPredClearance = 999f,
                MeanSpeed = Mean(prof),
                MinSpeed = Min(prof, cruise),
                RequiredDecel = 0f,
                CandidateIndex = 0,
                FirstTangentErrDeg = firstTang,
                RouteHeadErrDeg = headErr,
            };
            hasCurrent = true;
            return new ManeuverCommand
            {
                Path = new List<Vector3>(path),
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                AimPoint = current.AimPoint,
                TargetSpeed = targetNow,
                Style = style,
                Reason = current.SpeedLimiting,
                Reverse = false,
            };
        }

        // --- Phase 5 (gated): one slower/stopped civilian ahead.
        // FOLLOW if passing is unsafe; else one committed PASS trajectory.
        private void UpdateBlockerIntent(Vector3 egoPos, float egoSpeed, int now, int t)
        {
            if (!enablePassing)
            {
                if (intent.Current != ManeuverIntent.KEEP_LINE && intent.Current != ManeuverIntent.RECOVER)
                    intent.Force(ManeuverIntent.KEEP_LINE, "passing-off", now);
                blockerHandle = 0;
                return;
            }
            if (intent.Current == ManeuverIntent.RECOVER) return;
            try
            {
                // Lightweight single-blocker scan (no full Perception).
                int bh = 0;
                float bLat = 0f;
                float bDist = 999f;
                float bSpd = 0f;
                float best = float.MaxValue;
                List<Vehicle> near = null;
                try { near = new List<Vehicle>(World.GetNearbyVehicles(egoPos, 65f)); } catch { near = null; }
                if (near != null)
                {
                    foreach (var v in near)
                    {
                        try
                        {
                            if (v == null || !v.Exists() || v == vehicle) continue;
                            Vector3 p = v.Position;
                            var pr = route.ProjectOntoRoute(p);
                            float rd = pr.S - route.AlongS;
                            if (rd < 4f || rd > 55f) continue;
                            float half = corridor.HalfWidthAt(Math.Max(0f, rd));
                            if (Math.Abs(pr.Lateral) > half + 2f) continue;
                            Vector3 vv = new Vector3();
                            try { vv = v.Velocity; } catch { }
                            float spdAlong = RaceMath.FlatDot(new Vector3(vv.X, vv.Y, 0f), pr.Dir);
                            if (rd < best) { best = rd; bh = SafeHandle(v); bLat = pr.Lateral; bDist = rd; bSpd = spdAlong; }
                        }
                        catch { }
                    }
                }
                blockerHandle = bh;
                blockerLat = bLat;
                blockerDist = bDist;
                blockerSpeedAlong = bSpd;

                float cruise = EffectiveCruise();
                bool hasBlocker = bh != 0 && bDist < 55f;
                // Not slower/stopped relative to us: keep line.
                if (!hasBlocker || bSpd > cruise - 2f)
                {
                    // Pass complete: return to KEEP_LINE (forced: completed).
                    if (intent.Current == ManeuverIntent.PASS_LEFT || intent.Current == ManeuverIntent.PASS_RIGHT
                        || intent.Current == ManeuverIntent.FOLLOW)
                    {
                        bool cleared = !hasBlocker || bDist > 18f;
                        if (cleared) intent.Force(ManeuverIntent.KEEP_LINE, "pass-complete", now);
                        else intent.Request(ManeuverIntent.KEEP_LINE, "blocker-fast", now, false);
                    }
                    return;
                }
                // Slower/stopped blocker: can we pass? Need road space.
                float halfAhead = corridor.MinHalfWidthAhead(Math.Min(bDist + 20f, 80f));
                bool wideEnough = halfAhead >= 5.5f;
                if (!wideEnough)
                {
                    intent.Request(ManeuverIntent.FOLLOW, $"narrow half={halfAhead:F1}", now, false);
                    if (intent.Current == ManeuverIntent.PASS_LEFT || intent.Current == ManeuverIntent.PASS_RIGHT)
                        intent.Force(ManeuverIntent.FOLLOW, "unsafe-narrow", now);
                    return;
                }
                // Deliberately choose a side (opposite the blocker, prefer left).
                ManeuverIntent wantPass = bLat >= 0f ? ManeuverIntent.PASS_RIGHT : ManeuverIntent.PASS_LEFT;
                if (intent.Current == ManeuverIntent.KEEP_LINE || intent.Current == ManeuverIntent.FOLLOW)
                {
                    // Commit to the pass (forced: tactical conditions changed).
                    intent.Force(wantPass, $"blocker d={bDist:F0} lat={bLat:F1} v={bSpd:F1}", now);
                    try { telemetry?.Event(t, "PASS_COMMIT", $"{wantPass};blocker#{bh} d={bDist:F0} lat={bLat:F1} v={bSpd:F1} half={halfAhead:F1}"); } catch { }
                }
                else if ((intent.Current == ManeuverIntent.PASS_LEFT || intent.Current == ManeuverIntent.PASS_RIGHT)
                    && intent.Current != wantPass && (now - intent.SinceMs) > 4000)
                {
                    // Re-evaluate side only after a long hold (hysteresis).
                    intent.Force(wantPass, "side-better", now);
                }
            }
            catch { }
        }

        private ManeuverCommand BuildFollow(Vector3 egoPos, float egoSpeed)
        {
            var keep = BuildKeepLine(egoPos, egoSpeed);
            // Cap speed to the blocker with a gap (no indefinite stop logic:
            // if the blocker is stopped and the road is wide, UpdateBlockerIntent
            // already committed to PASS, so FOLLOW here means genuinely unsafe).
            float cap = Math.Max(blockerSpeedAlong + 1.5f, blockerSpeedAlong < 0.5f ? 0f : 4f);
            float cruise = EffectiveCruise();
            if (cap > cruise) cap = cruise;
            // Apply a stop gap for a stopped blocker we must wait for.
            if (blockerSpeedAlong < 0.5f && blockerDist < 12f) cap = 0f;
            var prof = keep.SpeedProfile;
            for (int i = 0; i < prof.Count; i++) if (prof[i] > cap) prof[i] = cap;
            keep.SpeedProfile = prof;
            keep.TargetSpeed = prof.Count > 0 ? prof[0] : cap;
            keep.Reason = "Follow";
            if (hasCurrent)
            {
                var c = current;
                c.TargetSpeed = keep.TargetSpeed;
                c.SpeedLimiting = "Follow";
                c.SpeedProfile = new List<float>(prof);
                current = c;
            }
            return keep;
        }

        private ManeuverCommand BuildPass(Vector3 egoPos, float egoSpeed)
        {
            float cruise = EffectiveCruise();
            float look = LookaheadM;
            float half = corridor.HalfWidthAt(Math.Min(blockerDist + 15f, look));
            if (half < 2.5f) half = 2.5f;
            // One feasible pass trajectory: offset to the committed side,
            // clearing the blocker laterally by car half + margin.
            float side = intent.Current == ManeuverIntent.PASS_LEFT ? 1f : -1f;
            float endLat = side * Math.Max(half * 0.55f, Math.Abs(blockerLat) + 2.6f);
            endLat = RaceMath.Clamp(endLat, -(half + 1f), half + 1f);
            float startLat = RaceMath.Clamp(route.Lateral, -18f, 18f);
            float headErr = route.HeadingErrorDeg;
            List<float> lats;
            List<float> ss;
            var path = PoseConnector.BuildPath(route, egoPos, startLat, headErr, endLat, look, 5f, out lats, out ss);
            float firstTang = PoseConnector.FirstTangentErrorDeg(path, lastEgoHeading);
            float aLat = capability.UsableLat(profile.GripFactor);
            float aBrake = capability.UsableBrake(profile.GripFactor);
            int n = path.Count;
            var vAllow = new float[n];
            for (int i = 0; i < n; i++)
            {
                float k = CurvatureOfPathAt(path, i);
                float vc = k < 1e-5f ? cruise : (float)Math.Sqrt(aLat / k);
                if (vc > cruise) vc = cruise;
                vAllow[i] = vc;
            }
            var vTgt = new float[n];
            if (n > 0)
            {
                vTgt[n - 1] = vAllow[n - 1];
                for (int i = n - 2; i >= 0; i--)
                {
                    float ds = Math.Max(1f, ss[i + 1] - ss[i]);
                    float vr = (float)Math.Sqrt(vTgt[i + 1] * vTgt[i + 1] + 2f * aBrake * ds);
                    vTgt[i] = Math.Min(vAllow[i], vr);
                }
            }
            var prof = new List<float>(vTgt);
            float targetNow = prof.Count > 0 ? prof[0] : cruise;
            float maxKappa = 0f;
            for (int i = 0; i < n; i++) { float k = CurvatureOfPathAt(path, i); if (k > maxKappa) maxKappa = k; }
            current = new TrajectoryCandidate
            {
                LateralM = endLat,
                LookaheadM = look,
                AimPoint = path.Count > 0 ? path[path.Count - 1] : egoPos,
                Score = 0f,
                ClearanceM = 999f,
                CurveCost = 0f,
                TacticalBias = 0f,
                RejectReason = "",
                Path = path,
                MinMarginM = MinMargin(lats, ss),
                MaxKappa = maxKappa,
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                ArrivalT = new List<float>(ss.Count),
                TargetSpeed = targetNow,
                SpeedLimiting = intent.Current.ToString(),
                ConstrainHandle = blockerHandle,
                ConstrainKind = "TrafficVehicle",
                ConstrainS = blockerDist,
                MinPredClearance = 999f,
                MeanSpeed = Mean(prof),
                MinSpeed = Min(prof, cruise),
                RequiredDecel = 0f,
                CandidateIndex = side > 0 ? 1 : -1,
                FirstTangentErrDeg = firstTang,
                RouteHeadErrDeg = headErr,
            };
            hasCurrent = true;
            return new ManeuverCommand
            {
                Path = new List<Vector3>(path),
                StationS = new List<float>(ss),
                SpeedProfile = new List<float>(prof),
                AimPoint = current.AimPoint,
                TargetSpeed = targetNow,
                Style = style,
                Reason = intent.Current.ToString(),
                Reverse = false,
            };
        }

        private void PublishViz(ManeuverCommand m, float egoSpeed, int t, float egoHeading)
        {
            try
            {
                trajViz.LastCandidates.Clear();
                if (hasCurrent) trajViz.LastCandidates.Add(current);
                trajViz.Chosen = current;
                trajViz.HasChosen = hasCurrent;
                trajViz.PlanId = maneuverPlanId;
                trajViz.BrakingPointS = -1f;
                speedViz.TargetSpeed = TargetSpeed;
                speedViz.Limiting = SpeedLimit;
                speedViz.ProfileS.Clear();
                speedViz.ProfileAllowed.Clear();
                speedViz.ProfileTarget.Clear();
                if (m.SpeedProfile != null)
                {
                    for (int i = 0; i < m.SpeedProfile.Count; i++)
                    {
                        float s = (m.StationS != null && i < m.StationS.Count) ? m.StationS[i] : i * 5f;
                        speedViz.ProfileS.Add(s);
                        speedViz.ProfileAllowed.Add(m.SpeedProfile[i]);
                        speedViz.ProfileTarget.Add(m.SpeedProfile[i]);
                    }
                }
                speedViz.BrakingPointS = -1f;
            }
            catch { }
        }

        private void LogIntent(int t, int now)
        {
            try
            {
                string key = intent.Current + "|" + TargetSpeed.ToString("F0") + "|" + SpeedLimit;
                if (key != lastIntentLog || now - lastIntentLogMs > 4000)
                {
                    lastIntentLog = key;
                    lastIntentLogMs = now;
                    var c = hasCurrent ? current : new TrajectoryCandidate();
                    telemetry?.Event(t, "INTENT",
                        $"intent={intent.Current};why={intent.Reason};held={intent.HeldS(now):F1}s;"
                        + $"lat={c.LateralM:F1};v={TargetSpeed:F1};lim={SpeedLimit};"
                        + $"egoHead={lastEgoHeading:F0};routeHead={route.RouteHeadingDeg:F0};"
                        + $"headErr={route.HeadingErrorDeg:F0};firstTang={c.FirstTangentErrDeg:F1};"
                        + $"maxKappa={c.MaxKappa:F4};s={route.AlongS:F0};{route.LocDetail}");
                }
            }
            catch { }
        }

        private void UpdateKinematics(int now, Vector3 egoPos, Vector3 egoVel, float egoSpeed, float egoHeading)
        {
            if (!hasKin) return;
            float dtS = (now - lastKinT) / 1000f;
            if (dtS <= 0f || dtS > 0.6f) return;
            float accel = (egoSpeed - lastSpeed) / dtS;
            lastAccelLong = accel;
            float dhDeg = RaceMath.HeadingDiffDeg(egoHeading, lastEgoHeading);
            float yawRate = 0f;
            try { yawRate = dhDeg * (float)Math.PI / 180f / dtS; } catch { }
            lastYawRate = yawRate;
            float latA = egoSpeed * yawRate;
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
            try { collided = vehicle.HasCollided; } catch { }
            var kind = ImpactClassifier.Classify(accel, dtS, displacement, expected, healthDrop, collided, egoSpeed);
            if ((kind == SampleKind.Impact || kind == SampleKind.Teleport) && now - lastImpactEventMs > 1500)
            {
                lastImpactEventMs = now;
                try
                {
                    if (kind == SampleKind.Impact)
                        telemetry?.Event(now - t0, "IMPACT", $"dec={accel:F0};spd={egoSpeed:F0};dmg={healthDrop:F0};slip={slip:F0};yaw={yawRate:F2}");
                    else
                        telemetry?.Event(now - t0, "TELEPORT", $"moved={displacement:F0};exp={expected:F0};spd={egoSpeed:F0}");
                }
                catch { }
            }
            if (kind == SampleKind.Normal || kind == SampleKind.Braking)
            {
                try { capability.Observe(accel, latA, egoSpeed, dtS, yawRate, slip); } catch { }
            }
            if (health >= 0f) lastHealth = health;
        }

        private void WriteSample(int t, Vector3 egoPos, float egoSpeed)
        {
            if (telemetry == null) return;
            try
            {
                float offCorr = corridor.OffCorridor(route.Lateral);
                float curv = route.CurvatureAhead(80f);
                var c = hasCurrent ? current : new TrajectoryCandidate();
                var pe = actuator != null ? actuator.LastError : new PathFollowingError();
                telemetry.Sample(t, style, intent.Current.ToString(),
                    route.AlongS, route.Progress01, LookaheadM,
                    route.Lateral, corridor.HalfWidth, offCorr, route.HeadingErrorDeg, curv,
                    c.LateralM, c.Score, 0f, 0f,
                    TargetSpeed, egoSpeed, SpeedLimit ?? "Cruise", 0f,
                    capability.ABrakeMax, capability.ALatMax, blockerHandle != 0 ? 1 : 0,
                    blockerHandle != 0 ? blockerDist : 999f, 999f, 0f,
                    actuator.CurrentCruise, actuator.CurrentStyle,
                    route.IsLost ? 1 : 0, "Normal",
                    actuator.ReissueCount, FinishGap,
                    route.Source ?? "?", corridor.MinHalfWidthAhead(LookaheadM),
                    pe.Valid ? pe.LateralErrM : 0f, pe.Valid ? pe.HeadingErrDeg : 0f,
                    pe.Valid ? pe.SpeedErrMps : 0f, pe.Valid ? pe.DistToPathM : 999f,
                    -1f, capability.Confidence,
                    actuator.ActuatorName ?? "?", c.RejectReason ?? "",
                    c.MinMarginM, c.MaxKappa,
                    c.CandidateIndex, c.MeanSpeed, c.MinSpeed,
                    c.ConstrainHandle, c.ConstrainKind ?? "", c.ConstrainS,
                    c.MinPredClearance, maneuverPlanId,
                    pe.Valid ? pe.SteerDeg : 0f, pe.Valid ? pe.Throttle01 : 0f,
                    pe.Valid ? pe.Brake01 : 0f, pe.Valid ? pe.LocalTargetMps : TargetSpeed,
                    lastEgoHeading, route.RouteHeadingDeg, c.FirstTangentErrDeg,
                    route.ExpectedS, route.LocJumpM);
            }
            catch { }
        }

        private static float CurvatureOfPathAt(List<Vector3> path, int k)
        {
            try
            {
                if (path == null || path.Count < 3) return 0f;
                int i0 = Math.Max(0, k - 1);
                int i1 = k;
                int i2 = Math.Min(path.Count - 1, k + 1);
                if (i0 == i1 || i1 == i2) return 0f;
                var d0 = new Vector3(path[i1].X - path[i0].X, path[i1].Y - path[i0].Y, 0f);
                var d1 = new Vector3(path[i2].X - path[i1].X, path[i2].Y - path[i1].Y, 0f);
                float l0 = RaceMath.FlatLength(d0);
                float l1 = RaceMath.FlatLength(d1);
                if (l0 < 0.5f || l1 < 0.5f) return 0f;
                float dh = Math.Abs(RaceMath.SignedAngleDeg(d0, d1)) * (float)Math.PI / 180f;
                return dh / Math.Max((l0 + l1) * 0.5f, 1f);
            }
            catch { return 0f; }
        }

        private float MinMargin(List<float> lats, List<float> ss)
        {
            try
            {
                float m = 99f;
                for (int i = 0; i < lats.Count; i++)
                {
                    float half = corridor.HalfWidthAt(ss[i]);
                    float margin = half - Math.Abs(lats[i]);
                    if (margin < m) m = margin;
                }
                return m;
            }
            catch { return 99f; }
        }

        private static float Mean(List<float> v)
        {
            if (v == null || v.Count == 0) return 0f;
            float s = 0f;
            foreach (var x in v) s += x;
            return s / v.Count;
        }

        private static float Min(List<float> v, float fb)
        {
            if (v == null || v.Count == 0) return fb;
            float m = float.MaxValue;
            foreach (var x in v) if (x < m) m = x;
            return m;
        }

        private static int SafeHandle(Entity e)
        {
            try { return e.Handle; } catch { return 0; }
        }

        private static float SafeHeading(Vehicle v)
        {
            try { return v.Heading; } catch { return 0f; }
        }
    }
}
