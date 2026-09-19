using System;
using GTA;
using GTA.Math;

namespace StreetRacing.Debug
{
    /// In-game spatial debug visualization. CSV telemetry remains, but road
    /// geometry, candidate paths and predictions must be SEEN to be believed.
    ///
    /// Draws every tick (markers/lines only persist one frame):
    ///   route centerline (yellow), corridor left/right edges (green/red),
    ///   all candidate paths (gray viable / orange blocked), chosen (cyan),
    ///   actor predictions (kind-colored lines + predicted dots),
    ///   aim point (cyan sphere) + braking point (red cylinder).
    ///
    /// Toggle via StreetRacing.ini [Race] DebugViz=1. Costs ~200 draw calls
    /// when enabled; leave off for clean races / perf.
    internal sealed class RaceDebugViz
    {
        public bool Enabled;
        public string LastError { get; private set; } = "";
        public int FramesDrawn { get; private set; }

        public void Draw(
            RaceRoute route,
            RoadCorridor corridor,
            TrajectoryPlanner traj,
            Perception perception,
            SpeedPlanner speedPlan,
            Vector3 egoPos,
            float egoSpeed,
            float lookaheadM,
            float targetSpeed)
        {
            Draw(route, corridor, traj, perception, speedPlan, egoPos,
                new Vector3(0f, 1f, 0f), egoSpeed, lookaheadM, targetSpeed);
        }

        public void Draw(
            RaceRoute route,
            RoadCorridor corridor,
            TrajectoryPlanner traj,
            Perception perception,
            SpeedPlanner speedPlan,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            float lookaheadM,
            float targetSpeed)
        {
            Draw(route, corridor, traj, perception, speedPlan, null,
                egoPos, egoFwd, egoSpeed, lookaheadM, targetSpeed);
        }

        public void Draw(
            RaceRoute route,
            RoadCorridor corridor,
            TrajectoryPlanner traj,
            Perception perception,
            SpeedPlanner speedPlan,
            DrivingReference.Result roadReference,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            float lookaheadM,
            float targetSpeed)
        {
            Draw(route, corridor, traj, perception, speedPlan, roadReference, null,
                egoPos, egoFwd, egoSpeed, lookaheadM, targetSpeed);
        }

        public void Draw(
            RaceRoute route,
            RoadCorridor corridor,
            TrajectoryPlanner traj,
            Perception perception,
            SpeedPlanner speedPlan,
            DrivingReference.Result roadReference,
            LocalWorldModel localWorld,
            Vector3 egoPos,
            Vector3 egoFwd,
            float egoSpeed,
            float lookaheadM,
            float targetSpeed)
        {
            if (!Enabled) return;
            try
            {
                DrawRoute(route);
                if (localWorld != null)
                    DrawLocalWorld(localWorld);
                else if (roadReference == null)
                    DrawCorridor(corridor);
                DrawRoadReference(roadReference);
                DrawCandidates(traj);
                DrawActors(perception, egoSpeed);
                DrawAimAndBraking(route, traj, speedPlan);
                DrawEgoPose(egoPos, egoFwd, route, traj);
                FramesDrawn++;
                LastError = "";
            }
            catch (Exception ex)
            {
                LastError = ex.GetType().Name + ":" + ex.Message;
            }
        }

        private static void DrawEgoPose(Vector3 egoPos, Vector3 egoFwd, RaceRoute route, TrajectoryPlanner traj)
        {
            try
            {
                Vector3 fwd = RaceMath.FlatNormalize(new Vector3(egoFwd.X, egoFwd.Y, 0f));
                var tip = new Vector3(egoPos.X + fwd.X * 22f, egoPos.Y + fwd.Y * 22f, egoPos.Z);
                World.DrawLine(
                    new Vector3(egoPos.X, egoPos.Y, egoPos.Z + 1.2f),
                    new Vector3(tip.X, tip.Y, tip.Z + 1.2f),
                    System.Drawing.Color.FromArgb(230, 255, 255, 255));
                // Selected route tangent at ego (magenta): when this is
                // sideways from the white nose line on a straight road, the
                // race must not start — localization is wrong.
                if (route != null && route.Built)
                {
                    Vector3 td = route.RouteTangentDir;
                    try { td = RaceMath.FlatNormalize(new Vector3(td.X, td.Y, 0f)); } catch { }
                    if (RaceMath.FlatLength(td) > 0.1f)
                    {
                        var tTip = new Vector3(egoPos.X + td.X * 18f, egoPos.Y + td.Y * 18f, egoPos.Z);
                        World.DrawLine(
                            new Vector3(egoPos.X, egoPos.Y, egoPos.Z + 1.1f),
                            new Vector3(tTip.X, tTip.Y, tTip.Z + 1.1f),
                            System.Drawing.Color.FromArgb(230, 255, 0, 255));
                    }
                }
                // First candidate tangent is already visible as the initial
                // direction of each gray/cyan path: all must begin FORWARD
                // from the rival and gently spread laterally.
            }
            catch { }
        }

        private static void DrawRoute(RaceRoute route)
        {
            try
            {
                if (route == null || !route.Built || route.Points.Count < 2) return;
                var col = System.Drawing.Color.FromArgb(220, 255, 210, 0);
                int lo = Math.Max(0, route.NearestIndex - 2);
                int hi = Math.Min(route.Points.Count - 2, lo + 26);
                for (int i = lo; i <= hi; i++)
                {
                    var a = route.Points[i];
                    var b = route.Points[i + 1];
                    World.DrawLine(
                        new Vector3(a.X, a.Y, a.Z + 1f),
                        new Vector3(b.X, b.Y, b.Z + 1f), col);
                }
                // Finish marker column.
                try
                {
                    World.DrawMarker(MarkerType.Cylinder, route.Finish + new Vector3(0f, 0f, 1f),
                        new Vector3(), new Vector3(), new Vector3(6f, 6f, 8f),
                        System.Drawing.Color.FromArgb(200, 255, 220, 0), false, false, false, "", "", false);
                }
                catch { }
            }
            catch { }
        }

        private static void DrawCorridor(RoadCorridor corridor)
        {
            try
            {
                if (corridor == null || corridor.Slices.Count < 2) return;
                var leftCol = System.Drawing.Color.FromArgb(200, 0, 255, 0);
                var rightCol = System.Drawing.Color.FromArgb(200, 255, 60, 60);
                for (int i = 0; i < corridor.Slices.Count - 1; i++)
                {
                    var a = corridor.Slices[i];
                    var b = corridor.Slices[i + 1];
                    if (!a.Valid || !b.Valid) continue;
                    World.DrawLine(
                        new Vector3(a.LeftEdge.X, a.LeftEdge.Y, a.LeftEdge.Z + 0.6f),
                        new Vector3(b.LeftEdge.X, b.LeftEdge.Y, b.LeftEdge.Z + 0.6f), leftCol);
                    World.DrawLine(
                        new Vector3(a.RightEdge.X, a.RightEdge.Y, a.RightEdge.Z + 0.6f),
                        new Vector3(b.RightEdge.X, b.RightEdge.Y, b.RightEdge.Z + 0.6f), rightCol);
                }
            }
            catch { }
        }

        private static void DrawLocalWorld(LocalWorldModel world)
        {
            try
            {
                if (world == null) return;

                for (int i = 0; i < world.Road.Count; i++)
                {
                    var s = world.Road[i];
                    Vector3 f = RaceMath.VectorFromHeading(s.HeadingDeg);
                    f = RaceMath.FlatNormalize(f);
                    Vector3 l = new Vector3(-f.Y, f.X, 0f);
                    Vector3 a = new Vector3(
                        s.Center.X - f.X * s.HalfLengthM,
                        s.Center.Y - f.Y * s.HalfLengthM,
                        s.Center.Z + 0.45f);
                    Vector3 b = new Vector3(
                        s.Center.X + f.X * s.HalfLengthM,
                        s.Center.Y + f.Y * s.HalfLengthM,
                        s.Center.Z + 0.45f);
                    var col = s.Source == "Reference"
                        ? System.Drawing.Color.FromArgb(125, 0, 210, 255)
                        : System.Drawing.Color.FromArgb(80, 80, 160, 255);
                    World.DrawLine(a, b, col);

                    Vector3 lp = new Vector3(
                        s.Center.X + l.X * s.LeftM,
                        s.Center.Y + l.Y * s.LeftM,
                        s.Center.Z + 0.45f);
                    Vector3 rp = new Vector3(
                        s.Center.X - l.X * s.RightM,
                        s.Center.Y - l.Y * s.RightM,
                        s.Center.Z + 0.45f);
                    World.DrawLine(lp, rp, col);
                }

                for (int i = 0; i < world.DebugCells.Count; i++)
                {
                    var cell = world.DebugCells[i];
                    System.Drawing.Color col;
                    if (cell.Occupied)
                        col = System.Drawing.Color.FromArgb(210, 255, 20, 20);
                    else if (cell.OnRoad && cell.FlowCost > 0.75f)
                        col = System.Drawing.Color.FromArgb(150, 255, 120, 20);
                    else if (cell.OnRoad)
                        col = System.Drawing.Color.FromArgb(115, 40, 220, 100);
                    else
                        col = System.Drawing.Color.FromArgb(80, 150, 150, 150);

                    Vector3 a = new Vector3(cell.Position.X, cell.Position.Y, cell.Position.Z + 0.2f);
                    Vector3 b = new Vector3(cell.Position.X, cell.Position.Y, cell.Position.Z + 0.7f);
                    World.DrawLine(a, b, col);
                }
            }
            catch { }
        }

        private static void DrawRoadReference(DrivingReference.Result road)
        {
            try
            {
                if (road == null || road.Path == null || road.Path.Count < 2) return;
                for (int i = 0; i < road.Path.Count - 1; i++)
                {
                    Vector3 a = road.Path[i];
                    Vector3 b = road.Path[i + 1];
                    var d = RaceMath.FlatNormalize(new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                    var left = new Vector3(-d.Y, d.X, 0f);
                    float l = i < road.LeftRoadM.Count ? road.LeftRoadM[i] : 3.5f;
                    float r = i < road.RightRoadM.Count ? road.RightRoadM[i] : 3.5f;
                    float conf = i < road.RoadConfidence.Count ? road.RoadConfidence[i] : 0f;

                    var lp = new Vector3(a.X + left.X * l, a.Y + left.Y * l, a.Z + 0.9f);
                    var rp = new Vector3(a.X - left.X * r, a.Y - left.Y * r, a.Z + 0.9f);
                    var center = new Vector3(a.X, a.Y, a.Z + 0.95f);

                    var edgeCol = conf >= 0.70f
                        ? System.Drawing.Color.FromArgb(220, 30, 220, 255)
                        : conf >= 0.40f
                            ? System.Drawing.Color.FromArgb(200, 255, 200, 40)
                            : System.Drawing.Color.FromArgb(170, 150, 150, 150);

                    World.DrawLine(center, lp, edgeCol);
                    World.DrawLine(center, rp, edgeCol);

                    if (i < road.RoadCenter.Count)
                    {
                        var nc = road.RoadCenter[i];
                        World.DrawLine(center,
                            new Vector3(nc.X, nc.Y, nc.Z + 1.0f),
                            System.Drawing.Color.FromArgb(120, 100, 160, 255));
                    }
                }
            }
            catch { }
        }

        private static void DrawCandidates(TrajectoryPlanner traj)
        {
            try
            {
                if (traj == null || traj.LastCandidates.Count == 0) return;
                foreach (var c in traj.LastCandidates)
                {
                    if (c.Path == null || c.Path.Count < 2) continue;
                    bool isChosen = traj.HasChosen && ReferenceEquals(c.Path, traj.Chosen.Path);
                    // ReferenceEquals on struct copies won't match; compare Aim+Score instead.
                    if (traj.HasChosen)
                        isChosen = c.AimPoint.X == traj.Chosen.AimPoint.X && c.AimPoint.Y == traj.Chosen.AimPoint.Y && c.Score == traj.Chosen.Score;
                    System.Drawing.Color col;
                    if (isChosen)
                        col = System.Drawing.Color.FromArgb(230, 0, 255, 255);
                    else if (!string.IsNullOrEmpty(c.RejectReason))
                        col = System.Drawing.Color.FromArgb(160, 255, 150, 0);
                    else
                        col = System.Drawing.Color.FromArgb(130, 180, 180, 180);
                    for (int i = 0; i < c.Path.Count - 1; i++)
                    {
                        var a = c.Path[i];
                        var b = c.Path[i + 1];
                        World.DrawLine(
                            new Vector3(a.X, a.Y, a.Z + 0.8f),
                            new Vector3(b.X, b.Y, b.Z + 0.8f), col);
                    }
                }
            }
            catch { }
        }

        private static void DrawActors(Perception perception, float egoSpeed)
        {
            try
            {
                if (perception == null) return;
                int drawn = 0;
                foreach (var a in perception.Actors)
                {
                    if (drawn >= 14) break;
                    bool relevant = a.IsAhead || (a.RouteValid && a.RouteDist > -10f && a.RouteDist < 150f);
                    if (!relevant) continue;
                    if (a.Dist > 130f) continue;
                    System.Drawing.Color col;
                    switch (a.Kind)
                    {
                        case ActorKind.Rival: col = System.Drawing.Color.FromArgb(220, 255, 0, 255); break;
                        case ActorKind.Ped: col = System.Drawing.Color.FromArgb(220, 255, 140, 0); break;
                        case ActorKind.Obstacle: col = System.Drawing.Color.FromArgb(220, 255, 0, 0); break;
                        default: col = System.Drawing.Color.FromArgb(200, 255, 255, 0); break;
                    }
                    float dt = 1.5f;
                    try
                    {
                        // Time for ego to reach the actor's station, clamped.
                        float tReach = a.RouteValid
                            ? a.RouteDist / Math.Max(egoSpeed, 6f)
                            : a.Dist / Math.Max(egoSpeed, 6f);
                        dt = RaceMath.Clamp(tReach, 0.5f, 2.5f);
                    }
                    catch { }
                    Vector3 pred;
                    try { pred = perception.Predict(a, dt); }
                    catch { pred = a.Position; }
                    World.DrawLine(
                        new Vector3(a.Position.X, a.Position.Y, a.Position.Z + 1f),
                        new Vector3(pred.X, pred.Y, pred.Z + 1f), col);
                    try
                    {
                        World.DrawMarker(MarkerType.Sphere, pred + new Vector3(0f, 0f, 0.8f),
                            new Vector3(), new Vector3(), new Vector3(1.2f, 1.2f, 1.2f),
                            col, false, false, false, "", "", false);
                    }
                    catch { }
                    drawn++;
                }
            }
            catch { }
        }

        private static void DrawAimAndBraking(RaceRoute route, TrajectoryPlanner traj, SpeedPlanner speedPlan)
        {
            try
            {
                if (traj != null && traj.HasChosen && traj.Chosen.AimPoint != Vector3.Zero)
                {
                    try
                    {
                        World.DrawMarker(MarkerType.Sphere, traj.Chosen.AimPoint + new Vector3(0f, 0f, 1f),
                            new Vector3(), new Vector3(), new Vector3(1.6f, 1.6f, 1.6f),
                            System.Drawing.Color.FromArgb(230, 0, 255, 255), false, false, false, "", "", false);
                    }
                    catch { }
                }
                if (route != null && route.Built && speedPlan != null && speedPlan.BrakingPointS >= 0f)
                {
                    try
                    {
                        Vector3 bp = route.PointAtS(route.AlongS + speedPlan.BrakingPointS);
                        World.DrawMarker(MarkerType.Cylinder, bp + new Vector3(0f, 0f, 1f),
                            new Vector3(), new Vector3(), new Vector3(3f, 3f, 6f),
                            System.Drawing.Color.FromArgb(200, 255, 0, 0), false, false, false, "", "", false);
                    }
                    catch { }
                }
            }
            catch { }
        }
    }
}
