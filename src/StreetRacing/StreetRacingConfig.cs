using System;
using System.Windows.Forms;
using GTA;

namespace StreetRacing
{
    internal sealed class StreetRacingConfig
    {
        public int MinDistance = 1200;
        public int MaxDistance = 2800;
        public int FinishRadius = 30;
        public int MaxChallengeRange = 35;
        public float AiCruiseSpeed = 47f;
        public int RefreshIntervalMs = 2000;
        public int StuckTimeoutMs = 4000;
        public string DrivingStyleName = "Reckless";
        public int DrivingStyleRaw = 0;
        public bool TelemetryEnabled = true;
        public int RaceTimeoutMs = 600000;
        public int CooldownMs = 8000;
        public int HonkDebounceMs = 1500;
        public Keys CancelKey = Keys.G;
        // New architecture: driver skill/personality as planner parameters.
        // Profile presets: Cautious | Balanced | Aggressive.
        public string DriverProfileName = "Balanced";
        // Risk override: -1 = use preset, else 0..1 (0 cautious, 1 psycho).
        public float RiskTolerance = -1f;
        // Planner overrides: 0 = use preset.
        public float LookaheadTimeS = 0f;
        public float SafetyMarginM = 0f;
        public float GripFactor = 0f;
        // Spatial debug overlay (route / corridor / candidates / predictions).
        public bool DebugViz = false;
        public string DebugVizRaw = "";
        public string ConfigLoadError = "";
        // Actuator: Direct (intended: executes the joint path/speed maneuver
        // itself every tick) or GtaDriver (baseline/diagnostic only: hands
        // point/speed to GTA pathfinding, does not guarantee the maneuver).
        // Default is Direct: the isolation test for planner deadlocks and the
        // intended controller going forward.
        public string ActuatorName = "Direct";
        // Collapsed driver selection (Phases 1-5):
        //   Simple     = ONE dumb center-path route follower (DEFAULT).
        //                No candidates/tactics/traffic/Crashed. Proves the
        //                1-2 km @15-20 m/s milestone before any racecraft.
        //   DirectDiag = Phase-1 hardware probe (no perception/route/planner).
        //                Straight-road throttle/coast/brake/steer validation.
        //   Legacy     = old full stack (7 candidates + tactics + recovery).
        //                Preserved for comparison only; not the milestone path.
        public string DriverMode = "Simple";
        // Simple follower cruise cap (m/s). Effective cruise is
        // min(AiCruiseSpeed, SimpleCruise), clamped 5..25. 15-20 proving band.
        public float SimpleCruise = 18f;
        // Phase-5 single-blocker pass (one slower/stopped civilian ahead:
        // FOLLOW if unsafe else committed PASS_LEFT/RIGHT, then KEEP_LINE).
        // Default OFF for the milestone-1 dumb-driver test (pure route
        // following). Player Attack/Defend/Commit/SideBySide stays retired
        // until follow/control/recover/pass all pass.
        public bool EnablePassing = false;
        // Emergency low-speed GTA DriveTo rejoin ONLY (recovery primitive
        // fallback). Never the normal driver. Default off.
        public bool UseGtaRejoin = false;
        // Diag probe target speed (m/s) for DirectDiag mode.
        public float DiagCruise = 18f;

        public static StreetRacingConfig Load()
        {
            var c = new StreetRacingConfig();
            try
            {
                var s = ScriptSettings.Load(@"scripts\StreetRacing.ini");
                c.MinDistance = s.GetValue("Race", "MinDistance", c.MinDistance);
                c.MaxDistance = s.GetValue("Race", "MaxDistance", c.MaxDistance);
                c.FinishRadius = s.GetValue("Race", "FinishRadius", c.FinishRadius);
                c.MaxChallengeRange = s.GetValue("Race", "MaxChallengeRange", c.MaxChallengeRange);
                c.AiCruiseSpeed = (float)s.GetValue("Race", "AiCruiseSpeed", (double)c.AiCruiseSpeed);
                c.RefreshIntervalMs = s.GetValue("Race", "RefreshIntervalMs", c.RefreshIntervalMs);
                c.StuckTimeoutMs = s.GetValue("Race", "StuckTimeoutMs", c.StuckTimeoutMs);
                c.DrivingStyleName = s.GetValue("Race", "DrivingStyle", c.DrivingStyleName);
                c.DrivingStyleRaw = s.GetValue("Race", "DrivingStyleRaw", c.DrivingStyleRaw);
                c.TelemetryEnabled = ReadBool(s, "Race", "TelemetryEnabled", c.TelemetryEnabled, null);
                c.RaceTimeoutMs = s.GetValue("Race", "RaceTimeoutMs", c.RaceTimeoutMs);
                c.CooldownMs = s.GetValue("Race", "CooldownMs", c.CooldownMs);
                c.HonkDebounceMs = s.GetValue("Race", "HonkDebounceMs", c.HonkDebounceMs);
                c.DriverProfileName = s.GetValue("Race", "DriverProfile", c.DriverProfileName);
                c.RiskTolerance = (float)s.GetValue("Race", "RiskTolerance", (double)c.RiskTolerance);
                c.LookaheadTimeS = (float)s.GetValue("Race", "LookaheadTimeS", (double)c.LookaheadTimeS);
                c.SafetyMarginM = (float)s.GetValue("Race", "SafetyMarginM", (double)c.SafetyMarginM);
                c.GripFactor = (float)s.GetValue("Race", "GripFactor", (double)c.GripFactor);
                c.DebugViz = ReadBool(s, "Race", "DebugViz", c.DebugViz, raw => c.DebugVizRaw = raw);
                c.ActuatorName = s.GetValue("Race", "Actuator", c.ActuatorName);
                c.DriverMode = s.GetValue("Race", "DriverMode", c.DriverMode);
                c.SimpleCruise = (float)s.GetValue("Race", "SimpleCruise", (double)c.SimpleCruise);
                c.EnablePassing = ReadBool(s, "Race", "EnablePassing", c.EnablePassing, null);
                c.UseGtaRejoin = ReadBool(s, "Race", "UseGtaRejoin", c.UseGtaRejoin, null);
                c.DiagCruise = (float)s.GetValue("Race", "DiagCruise", (double)c.DiagCruise);
                var keyName = s.GetValue("Race", "CancelKey", "G");
                if (Enum.TryParse(keyName, true, out Keys k))
                {
                    c.CancelKey = k;
                }
                if (c.MaxDistance < c.MinDistance)
                {
                    c.MaxDistance = c.MinDistance;
                }
            }
            catch (Exception ex)
            {
                // Missing/corrupt ini -> run on defaults, but expose why.
                c.ConfigLoadError = ex.GetType().Name + ":" + ex.Message;
            }
            return c;
        }

        private static bool ReadBool(
            ScriptSettings settings,
            string section,
            string key,
            bool fallback,
            Action<string> rawSink)
        {
            try
            {
                string raw = settings.GetValue(section, key, fallback ? "true" : "false");
                rawSink?.Invoke(raw ?? "");
                string n = (raw ?? "").Trim().ToLowerInvariant();
                if (n == "1" || n == "true" || n == "yes" || n == "on") return true;
                if (n == "0" || n == "false" || n == "no" || n == "off") return false;

                bool parsed;
                if (bool.TryParse(n, out parsed)) return parsed;
            }
            catch { }
            return fallback;
        }

        public bool UseDirectActuator()
        {
            try
            {
                string n = (ActuatorName ?? "").Trim().ToLowerInvariant();
                return n == "direct" || n == "directactuator";
            }
            catch { return false; }
        }

        public bool UseSimpleDriver()
        {
            try
            {
                string n = (DriverMode ?? "").Trim().ToLowerInvariant();
                return n == "simple" || n == "minimal" || n == "dumb" || n == "";
            }
            catch { return true; }
        }

        public bool UseDiagDriver()
        {
            try
            {
                string n = (DriverMode ?? "").Trim().ToLowerInvariant();
                return n == "directdiag" || n == "diag" || n == "hardware" || n == "probe";
            }
            catch { return false; }
        }

        /// Resolves the driving style int from preset name or raw override.
        /// The style is the low-level actuator mode only; race intelligence
        /// lives in the planner (trajectory + speed), not in these flags.
        /// Flag math (from SHVDN VehicleDrivingFlags):
        ///   Calm        786475     = stop for vehicles/peds, ignore lights (polite baseline)
        ///   Rushed      1074528293 = SHVDN DrivingStyle.Rushed (still stops for vehicles!)
        ///   Reckless    1074528292 = Rushed minus StopForVehicles -> swerves instead of queuing
        ///   Psycho      1074528804 = Reckless plus AllowGoingWrongWay (oncoming-lane passes)
        ///   Disciplined 1074266152 = lane changes only: no shortcuts, no all-vehicle
        ///                            swerve, keeps road direction. Diagnostic baseline.
        public int ResolveDrivingStyle()
        {
            if (DrivingStyleRaw != 0)
            {
                return DrivingStyleRaw;
            }
            switch ((DrivingStyleName ?? "").Trim().ToLowerInvariant())
            {
                case "calm": return 786475;
                case "rushed": return 1074528293;
                case "psycho": return 1074528804;
                case "disciplined": return 1074266152;
                default: return 1074528292;
            }
        }

        public DriverProfile ResolveDriverProfile()
        {
            var p = DriverProfile.FromName(DriverProfileName, RiskTolerance);
            if (LookaheadTimeS > 0.5f && LookaheadTimeS < 8f)
                p.LookaheadTimeS = LookaheadTimeS;
            if (SafetyMarginM > 0f && SafetyMarginM < 30f)
                p.SafetyMarginM = SafetyMarginM;
            if (GripFactor > 0.4f && GripFactor <= 1.05f)
                p.GripFactor = GripFactor;
            return p;
        }
    }
}
