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
                c.TelemetryEnabled = s.GetValue("Race", "TelemetryEnabled", c.TelemetryEnabled);
                c.RaceTimeoutMs = s.GetValue("Race", "RaceTimeoutMs", c.RaceTimeoutMs);
                c.CooldownMs = s.GetValue("Race", "CooldownMs", c.CooldownMs);
                c.HonkDebounceMs = s.GetValue("Race", "HonkDebounceMs", c.HonkDebounceMs);
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
            catch
            {
                // Missing/corrupt ini -> run on defaults.
            }
            return c;
        }

        /// Resolves the driving style int from preset name or raw override.
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
    }
}
