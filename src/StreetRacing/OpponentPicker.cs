using GTA;
using GTA.Math;

namespace StreetRacing
{
    /// Picks which NPC you challenged by honking.
    /// Scoring prefers cars ahead of you and near your camera aim,
    /// closest distance wins ties. Fixes "who did I honk at?".
    internal static class OpponentPicker
    {
        public static bool TryPick(float maxRange, out Vehicle vehicle, out Ped driver)
        {
            vehicle = null;
            driver = null;

            var player = Game.Player.Character;
            if (player == null || !player.Exists() || !player.IsInVehicle())
            {
                return false;
            }

            var playerVeh = player.CurrentVehicle;
            if (playerVeh == null || !playerVeh.Exists() || playerVeh.Driver != player)
            {
                return false; // must be the driver to issue a challenge
            }

            var origin = playerVeh.Position;
            var fwd = playerVeh.ForwardVector;
            var camDir = GameplayCamera.Direction;
            Vector3 playerFwdFlat;
            try { playerFwdFlat = RaceMath.FlatNormalize(new Vector3(fwd.X, fwd.Y, 0f)); }
            catch { playerFwdFlat = new Vector3(0f, 1f, 0f); }

            Vehicle best = null;
            float bestScore = float.NegativeInfinity;

            foreach (var v in World.GetNearbyVehicles(origin, maxRange))
            {
                if (v == null || !v.Exists() || v == playerVeh || v.IsDead)
                {
                    continue;
                }
                var d = v.Driver;
                if (d == null || !d.Exists() || d.IsDead || d.IsPlayer)
                {
                    continue;
                }

                var to = v.Position - origin;
                float dist = to.Length();
                if (dist < 2f)
                {
                    continue;
                }
                // Invariant: only same-direction targets. Oncoming/opposite
                // forces >90 deg heading errors and head-on routing — reject.
                // Stopped cars still carry heading, so ForwardVector gates even
                // at zero speed. Threshold cos(60 deg)=0.5 allows lane/curve
                // variance but rejects cross (dot~0) and oncoming (dot<0).
                try
                {
                    var tvFwd = RaceMath.FlatNormalize(new Vector3(v.ForwardVector.X, v.ForwardVector.Y, 0f));
                    float dot = RaceMath.FlatDot(tvFwd, playerFwdFlat);
                    if (dot < 0.5f) continue;
                }
                catch { }
                var toN = Vector3.Normalize(to);

                float ahead = Vector3.Dot(fwd, toN);
                float cam = Vector3.Dot(camDir, toN);
                if (ahead < 0.15f && cam < 0.25f)
                {
                    continue; // behind you and not looked at -> not your target
                }

                float score = ahead * 2f + cam * 2f - dist * 0.05f;
                if (score > bestScore)
                {
                    bestScore = score;
                    best = v;
                }
            }

            if (best == null)
            {
                return false;
            }

            vehicle = best;
            driver = best.Driver;
            return driver != null && driver.Exists();
        }
    }
}
