using System;
using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Picks a random finish point on a road ahead of the player.
    /// No waypoint input: pure auto-pick inside [minD, maxD].
    internal static class FinishPicker
    {
        public static bool TryPick(Vector3 origin, Vector3 direction, int minD, int maxD, out Vector3 finish)
        {
            finish = Vector3.Zero;
            var rng = new Random();
            var flat = new Vector3(direction.X, direction.Y, 0f);
            if (flat.Length() < 0.01f)
            {
                flat = new Vector3(0f, 1f, 0f);
            }
            flat.Normalize();

            for (int i = 0; i < 28; i++)
            {
                // Random heading inside a ~120 deg cone ahead.
                double ang = (rng.NextDouble() - 0.5) * (Math.PI * 2.0 / 3.0);
                var dir = RotateFlat(flat, (float)ang);
                float dist = minD + (float)rng.NextDouble() * (maxD - minD);
                var raw = origin + dir * dist;

                Vector3 snapped;
                try
                {
                    snapped = World.GetNextPositionOnStreet(raw);
                }
                catch
                {
                    snapped = raw;
                }
                if (snapped == Vector3.Zero)
                {
                    continue;
                }
                if (FlatDistance(snapped, origin) < minD * 0.7f)
                {
                    continue;
                }
                finish = snapped;
                return true;
            }
            return false;
        }

        public static float FlatDistance(Vector3 a, Vector3 b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        private static Vector3 RotateFlat(Vector3 v, float ang)
        {
            float c = (float)Math.Cos(ang);
            float s = (float)Math.Sin(ang);
            return new Vector3(v.X * c - v.Y * s, v.X * s + v.Y * c, 0f);
        }
    }
}
