using System;
using GTA.Math;

namespace StreetRacing
{
    /// Flat (2D) math helpers shared by route / corridor / planning.
    /// All lateral/longitudinal reasoning is flat; Z is carried but not used for decisions.
    internal static class RaceMath
    {
        public static float FlatDistance(Vector3 a, Vector3 b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public static float FlatLength(Vector3 v)
        {
            return (float)Math.Sqrt(v.X * v.X + v.Y * v.Y);
        }

        public static Vector3 FlatNormalize(Vector3 v)
        {
            float l = FlatLength(v);
            if (l < 1e-5f) return new Vector3(0f, 1f, 0f);
            return new Vector3(v.X / l, v.Y / l, 0f);
        }

        public static float FlatDot(Vector3 a, Vector3 b) => a.X * b.X + a.Y * b.Y;

        /// Z of flat cross product (a x b). Positive = b is left of a.
        public static float FlatCross(Vector3 a, Vector3 b) => a.X * b.Y - a.Y * b.X;

        public static float Clamp(float v, float lo, float hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        /// Signed angle from a to b in degrees, -180..180. Positive = left turn.
        public static float SignedAngleDeg(Vector3 a, Vector3 b)
        {
            var an = FlatNormalize(a);
            var bn = FlatNormalize(b);
            float dot = Clamp(FlatDot(an, bn), -1f, 1f);
            float ang = (float)(Math.Acos(dot) * 180.0 / Math.PI);
            return FlatCross(an, bn) >= 0f ? ang : -ang;
        }

        public static float UnsignedAngleDeg(Vector3 a, Vector3 b)
        {
            return Math.Abs(SignedAngleDeg(a, b));
        }

        /// GTA heading from a flat direction vector.
        /// GTA/SHVDN convention: 0 = +Y/north, 90 = -X/west,
        /// 180 = -Y/south, 270 = +X/east. Heading increases toward the left.
        public static float HeadingFromVector(Vector3 v)
        {
            // Inverse of VectorFromHeading and equivalent to SHVDN Vector3.ToHeading().
            double rad = Math.Atan2(-v.X, v.Y);
            double deg = rad * 180.0 / Math.PI;
            if (deg < 0.0) deg += 360.0;
            if (deg >= 360.0) deg -= 360.0;
            return (float)deg;
        }

        public static Vector3 VectorFromHeading(float headingDeg)
        {
            double rad = headingDeg * Math.PI / 180.0;
            return new Vector3(-(float)Math.Sin(rad), (float)Math.Cos(rad), 0f);
        }

        /// Smallest signed GTA heading difference (target - current), -180..180.
        /// Positive means target is to the left of current; negative means right.
        public static float HeadingDiffDeg(float target, float current)
        {
            float d = (target - current) % 360f;
            if (d > 180f) d -= 360f;
            if (d < -180f) d += 360f;
            return d;
        }

        public struct Projection
        {
            public Vector3 Closest;
            public float T;      // 0..1 along a->b
            public float Dist;   // flat distance p->closest
            public float Along;  // flat distance a->closest
        }

        public static Projection ProjectOnSegment(Vector3 p, Vector3 a, Vector3 b)
        {
            var ab = new Vector3(b.X - a.X, b.Y - a.Y, 0f);
            float len2 = ab.X * ab.X + ab.Y * ab.Y;
            var r = new Projection { Closest = a, T = 0f, Dist = FlatDistance(p, a), Along = 0f };
            if (len2 < 1e-6f) return r;
            var ap = new Vector3(p.X - a.X, p.Y - a.Y, 0f);
            float t = (ap.X * ab.X + ap.Y * ab.Y) / len2;
            t = Clamp(t, 0f, 1f);
            var c = new Vector3(a.X + ab.X * t, a.Y + ab.Y * t, a.Z + (b.Z - a.Z) * t);
            r.Closest = c;
            r.T = t;
            r.Dist = FlatDistance(p, c);
            r.Along = (float)Math.Sqrt(ab.X * ab.X + ab.Y * ab.Y) * t;
            return r;
        }
    }
}
