using System;
using System.Collections.Generic;
using GTA.Math;

namespace StreetRacing
{
    /// Single source of truth for the pose-aware lateral connector.
    ///
    /// Conventions (GTA):
    ///   heading deg: 0 = +Y (north), 90 = -X (west), 270 = +X (east).
    ///   Heading increases toward the left.
    ///   route lateral: + = left of route direction
    ///     (Lateral = Cross(routeDir, ego - closest)).
    ///   headErr = HeadingDiff(routeHead, egoHead) = route - ego, -180..180.
    ///     Positive headErr = route is left of the nose.
    ///
    /// Correct initial slope: d'(0) = -tan(headErr).
    /// With GTA headings increasing to the LEFT, a positive headErr means the
    /// route is left of the nose. In the route-local frame, the ego tangent
    /// must therefore begin with a negative lateral derivative to reproduce the
    /// current vehicle heading before smoothly converging to the centerline.
    ///
    /// Deterministic check: for known ego/route headings the first generated
    /// tangent must rotate TOWARD the vehicle pose (firstTangErr < |headErr|),
    /// never away from it. See VerifyToward() / SelfTest().
    internal static class PoseConnector
    {
        private const float MaxERad = 1.1f; // ~63 deg clamp so Hermite stays finite
        private const float MaxSlope = 2f;

        public static float LateralSlopeForHeadErrDeg(float headErrDeg)
        {
            try
            {
                if (headErrDeg > 180f) headErrDeg = 180f;
                if (headErrDeg < -180f) headErrDeg = -180f;
                float eRad = headErrDeg * (float)Math.PI / 180f;
                if (eRad > MaxERad) eRad = MaxERad;
                if (eRad < -MaxERad) eRad = -MaxERad;
                float m0 = -(float)Math.Tan(eRad);
                if (m0 > MaxSlope) m0 = MaxSlope;
                if (m0 < -MaxSlope) m0 = -MaxSlope;
                return m0;
            }
            catch { return 0f; }
        }

        /// Hermite d(0)=startLat, d'(0)=-tan(headErr), d(S)=endLat, d'(S)=0.
        public static List<Vector3> BuildPath(RaceRoute route, Vector3 egoPos,
            float startLat, float headErrDeg, float endLat, float lookaheadM,
            float stationDs, out List<float> pathLats, out List<float> pathS)
        {
            pathLats = new List<float>();
            pathS = new List<float>();
            var path = new List<Vector3>();
            try
            {
                float m0 = LateralSlopeForHeadErrDeg(headErrDeg);
                float S = Math.Max(lookaheadM, 10f);
                int n = Math.Max(5, Math.Min(33, (int)Math.Ceiling(lookaheadM / stationDs) + 1));
                for (int k = 0; k < n; k++)
                {
                    float s = k == n - 1 ? lookaheadM : k * stationDs;
                    if (s > lookaheadM) s = lookaheadM;
                    float t = S > 1f ? s / S : 1f;
                    if (t < 0f) t = 0f;
                    if (t > 1f) t = 1f;
                    float t2 = t * t;
                    float t3 = t2 * t;
                    float h00 = 2f * t3 - 3f * t2 + 1f;
                    float h10 = t3 - 2f * t2 + t;
                    float h01 = -2f * t3 + 3f * t2;
                    float lat = h00 * startLat + h10 * S * m0 + h01 * endLat;
                    Vector3 rp = route.PointAtS(route.AlongS + s);
                    float h = route.HeadingAtS(route.AlongS + s);
                    var dir = RaceMath.VectorFromHeading(h);
                    var leftV = new Vector3(-dir.Y, dir.X, 0f);
                    path.Add(new Vector3(rp.X + leftV.X * lat, rp.Y + leftV.Y * lat, rp.Z));
                    pathLats.Add(lat);
                    pathS.Add(s);
                    if (s >= lookaheadM - 0.01f) break;
                }
                if (path.Count > 0) path[0] = new Vector3(egoPos.X, egoPos.Y, path[0].Z);
            }
            catch { }
            return path;
        }

        public static float FirstTangentErrorDeg(IList<Vector3> path, float egoHeadingDeg)
        {
            try
            {
                if (path == null || path.Count < 2) return 0f;
                var d = new Vector3(path[1].X - path[0].X, path[1].Y - path[0].Y, 0f);
                if (RaceMath.FlatLength(d) < 0.5f) return 0f;
                float hPath = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(d));
                return Math.Abs(RaceMath.HeadingDiffDeg(hPath, egoHeadingDeg));
            }
            catch { return 0f; }
        }

        /// True when the connector leaves the nose toward the route, i.e. the
        /// first tangent is closer to ego heading than the route tangent is.
        /// Allows a small tolerance for discretization (5 m stations).
        public static bool VerifyToward(float egoHeadingDeg, float routeHeadingDeg, float firstTangErrDeg)
        {
            float headErr = Math.Abs(RaceMath.HeadingDiffDeg(routeHeadingDeg, egoHeadingDeg));
            // With correct sign the first meters continue along the nose:
            // firstTangErr should be small (< ~12 deg) and strictly less
            // than the route mismatch (unless already aligned).
            if (firstTangErrDeg > 12f && headErr < 45f) return false;
            if (headErr > 5f && firstTangErrDeg > headErr + 5f) return false;
            return true;
        }

        /// Deterministic sign check for GTA's heading convention. For a route
        /// at 124 and ego at 94, headErr is +30 and m0 must be about -0.577
        /// so the connector's first tangent reconstructs the ego heading.
        public static string SelfTest()
        {
            try
            {
                float m0 = LateralSlopeForHeadErrDeg(30f);
                if (m0 > -0.4f || m0 < -0.75f) return $"FAIL m0(30)={m0:F3} expect ~-0.577";
                float m0n = LateralSlopeForHeadErrDeg(-10f);
                if (m0n < 0.1f || m0n > 0.25f) return $"FAIL m0(-10)={m0n:F3} expect ~+0.176";
                // Reconstruct initial heading for ego94/route124 case.
                float routeH = 124f;
                var routeDir = RaceMath.VectorFromHeading(routeH);
                var leftV = new Vector3(-routeDir.Y, routeDir.X, 0f);
                var init = new Vector3(
                    routeDir.X + m0 * leftV.X,
                    routeDir.Y + m0 * leftV.Y, 0f);
                float hInit = RaceMath.HeadingFromVector(RaceMath.FlatNormalize(init));
                float err = Math.Abs(RaceMath.HeadingDiffDeg(hInit, 94f));
                if (err > 12f) return $"FAIL initHead={hInit:F0} err={err:F0} expect ~94";
                return "OK";
            }
            catch (Exception ex) { return "FAIL exc:" + ex.Message; }
        }
    }
}
