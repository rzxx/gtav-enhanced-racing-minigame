using System;
using System.Globalization;
using System.IO;

namespace StreetRacing
{
    /// Enriched race telemetry: every important decision is explainable from
    /// the logs. Samples at 10 Hz + discrete events.
    ///
    /// samples columns:
    ///   t_ms,style,tactical,prog_m,prog_pct,look_m,lat_m,halfW_m,offCorr_m,
    ///   headErr_deg,curv,aimLat,chScore,rejLat,rejScore,v_tgt,v_act,v_lim,
    ///   brakeNeed,aBrake,aLat,nActors,nearD,nearTTC,nearClose,cmdCruise,
    ///   cmdStyle,routeLost,impact,reissue,finishGap
    /// events: START / TACTIC / ROUTE_LOST / ROUTE_FOUND / RECOVERY /
    ///   IMPACT / TELEPORT / HARD_BRAKE / CTRL
    internal sealed class RaceTelemetry
    {
        private readonly StreamWriter samples;
        private readonly StreamWriter events;
        private int sampleCount;
        private bool closed;

        public RaceTelemetry(int raceId, int style, string profile)
        {
            samples = new StreamWriter($"scripts\\StreetRacing_race_{raceId}.csv", false);
            events = new StreamWriter($"scripts\\StreetRacing_race_{raceId}_events.csv", false);
            samples.WriteLine("t_ms,style,tactical,prog_m,prog_pct,look_m,lat_m,halfW_m,offCorr_m,headErr_deg,curv,aimLat,chScore,rejLat,rejScore,v_tgt,v_act,v_lim,brakeNeed,aBrake,aLat,nActors,nearD,nearTTC,nearClose,cmdCruise,cmdStyle,routeLost,impact,reissue,finishGap");
            events.WriteLine("t_ms,type,detail");
            Event(0, "START", "style=" + style + ";profile=" + (profile ?? "?"));
        }

        public void Sample(int t, int style, string tactical,
            float progM, float progPct, float lookM,
            float latM, float halfW, float offCorr, float headErr, float curv,
            float aimLat, float chScore, float rejLat, float rejScore,
            float vTgt, float vAct, string vLim, float brakeNeed,
            float aBrake, float aLat, int nActors,
            float nearD, float nearTtc, float nearClose,
            float cmdCruise, int cmdStyle, int routeLost, string impact,
            int reissue, float finishGap)
        {
            if (closed) return;
            samples.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},{3:F0},{4:F3},{5:F0},{6:F1},{7:F1},{8:F1},{9:F0},{10:F4},{11:F1},{12:F2},{13:F1},{14:F2},{15:F1},{16:F1},{17},{18:F2},{19:F1},{20:F1},{21},{22:F0},{23:F1},{24:F1},{25:F1},{26},{27},{28},{29},{30:F0}",
                t, style, tactical, progM, progPct, lookM, latM, halfW, offCorr, headErr, curv,
                aimLat, chScore, rejLat, rejScore, vTgt, vAct, vLim, brakeNeed, aBrake, aLat,
                nActors, nearD, nearTtc, nearClose, cmdCruise, cmdStyle, routeLost, impact, reissue, finishGap));
            if (++sampleCount % 50 == 0)
            {
                try { samples.Flush(); events.Flush(); } catch { }
            }
        }

        public void Event(int t, string type, string detail)
        {
            if (closed) return;
            try
            {
                string safe = (detail ?? "").Replace(',', ';').Replace('\n', ' ');
                events.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0},{1},{2}", t, type, safe));
            }
            catch { }
        }

        public void Close()
        {
            if (closed) return;
            closed = true;
            try { samples.Flush(); events.Flush(); samples.Close(); events.Close(); }
            catch { }
        }
    }
}
