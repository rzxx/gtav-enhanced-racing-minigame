using System;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Road context around the rival, sampled at ~2 Hz for telemetry and
    /// (later) for context-dependent recklessness.
    /// Uses only street-snap positions, so no heading-unit pitfalls.
    internal struct RoadSense
    {
        public float OffRoad;   // m from rival to nearest street point
        public float AlignDeg;  // 0 = along road, 180 = against; -1 = unknown
        public int Traffic;     // vehicles within 40 m
        public float Frontal;   // nearest vehicle ahead in a narrow cone, m (999 = clear)

        public static RoadSense Sample(Vehicle rival)
        {
            var s = new RoadSense { AlignDeg = -1f, Frontal = 999f };
            try
            {
                var p = rival.Position;
                Vector3 a;
                try
                {
                    a = World.GetNextPositionOnStreet(p, true);
                }
                catch
                {
                    return s;
                }
                if (FinishPicker.FlatDistance(a, Vector3.Zero) < 1f)
                {
                    return s; // no street data (tunnels, docks, middle of nowhere)
                }
                s.OffRoad = FinishPicker.FlatDistance(p, a);

                var fwd = rival.ForwardVector;
                var flat = Vector3.Normalize(new Vector3(fwd.X, fwd.Y, 0f));
                var probe = p + flat * 25f;
                Vector3 b;
                try
                {
                    b = World.GetNextPositionOnStreet(probe, true);
                }
                catch
                {
                    b = Vector3.Zero;
                }
                if (FinishPicker.FlatDistance(b, Vector3.Zero) > 0.01f)
                {
                    var d = new Vector3(b.X - a.X, b.Y - a.Y, 0f);
                    if (d.Length() > 5f)
                    {
                        var dn = Vector3.Normalize(d);
                        float dot = Vector3.Dot(dn, flat);
                        dot = dot > 1f ? 1f : (dot < -1f ? -1f : dot);
                        s.AlignDeg = (float)(Math.Acos(dot) * 180.0 / Math.PI);
                    }
                }

                int n = 0;
                float best = 999f;
                foreach (var v in World.GetNearbyVehicles(p, 40f))
                {
                    if (v == null || !v.Exists() || v == rival)
                    {
                        continue;
                    }
                    n++;
                    var to = v.Position - p;
                    float dist = to.Length();
                    var ton = Vector3.Normalize(new Vector3(to.X, to.Y, 0f));
                    if (Vector3.Dot(ton, flat) > 0.94f && dist < best)
                    {
                        best = dist;
                    }
                }
                s.Traffic = n;
                s.Frontal = best;
            }
            catch
            {
            }
            return s;
        }
    }
}
