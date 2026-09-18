using System;
using System.Globalization;
using System.IO;

namespace StreetRacing
{
    /// LEGACY per-race CSV telemetry (10 Hz samples + events) for the v1 AI.
    /// Schema logged the invalid `offroad_m` street-node distance — do not use
    /// for new work. Replaced by Telemetry.RaceTelemetry (route / corridor /
    /// trajectory / TTC / tactic / controller / impact classification).
    [System.Obsolete("Use Telemetry.RaceTelemetry.")]
    internal sealed class Telemetry
    {
        private readonly StreamWriter samples;
        private readonly StreamWriter events;
        private int sampleCount;

        public Telemetry(int raceId, int style)
        {
            samples = new StreamWriter($"scripts\\StreetRacing_race_{raceId}.csv", false);
            events = new StreamWriter($"scripts\\StreetRacing_race_{raceId}_events.csv", false);
            samples.WriteLine("t_ms,style,speed_rival,speed_you,dist_rival,dist_you,offroad_m,align_deg,traffic40,frontal_m,cap");
            events.WriteLine("t_ms,type,detail");
            Event(0, "START", "style=" + style);
        }

        public void Sample(int t, int style, float rSpeed, float ySpeed, float rDist, float yDist,
            float offroad, float align, int traffic, float frontal, float cap)
        {
            samples.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2:F1},{3:F1},{4:F0},{5:F0},{6:F1},{7:F0},{8},{9:F0},{10:F1}",
                t, style, rSpeed, ySpeed, rDist, yDist, offroad, align, traffic, frontal, cap));
            if (++sampleCount % 50 == 0)
            {
                samples.Flush();
            }
        }

        public void Event(int t, string type, string detail)
        {
            events.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0},{1},{2}", t, type, detail));
        }

        public void Close()
        {
            try
            {
                samples.Flush();
                events.Flush();
                samples.Close();
                events.Close();
            }
            catch
            {
            }
        }
    }
}
