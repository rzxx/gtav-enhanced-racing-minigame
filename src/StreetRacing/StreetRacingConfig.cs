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
        public int StuckTimeoutMs = 4000;
        public int RetaskIntervalMs = 6000;
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
                c.StuckTimeoutMs = s.GetValue("Race", "StuckTimeoutMs", c.StuckTimeoutMs);
                c.RetaskIntervalMs = s.GetValue("Race", "RetaskIntervalMs", c.RetaskIntervalMs);
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
    }
}
