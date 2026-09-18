using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// Immutable GPS acquisition result.
    ///
    /// Lifecycle contract (this pass):
    ///   GTA GPS natives -> RouteSnapshot (ONE sampling)
    ///   -> validation (same snapshot, no natives)
    ///   -> SimpleBrain (same snapshot, no natives)
    ///
    /// A snapshot is the single authoritative route acquisition for a race.
    /// It must be copied by value (points + cumulative + metadata) so the
    /// accepted geometry survives validation -> start without re-querying
    /// GTA. FallbackWalk / StraightFallback snapshots are never valid for
    /// Simple mode (GPS-only invariant).
    internal sealed class RouteSnapshot
    {
        public List<Vector3> Points = new List<Vector3>();
        public List<float> CumulativeS = new List<float>();
        public float TotalLength;
        public string Source = "None";
        public int GpsSamples;
        public Vector3 Finish = Vector3.Zero;
        public Vector3 Origin = Vector3.Zero;

        public bool IsGps
        {
            get
            {
                try { return Points != null && Points.Count >= 2 && Source != null && Source.StartsWith("Gps"); }
                catch { return false; }
            }
        }

        public bool Valid => IsGps;

        public RouteSnapshot Clone()
        {
            var c = new RouteSnapshot
            {
                TotalLength = TotalLength,
                Source = Source,
                GpsSamples = GpsSamples,
                Finish = Finish,
                Origin = Origin,
            };
            if (Points != null)
            {
                foreach (var p in Points) c.Points.Add(p);
            }
            if (CumulativeS != null)
            {
                foreach (var s in CumulativeS) c.CumulativeS.Add(s);
            }
            return c;
        }
    }
}
