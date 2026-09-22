using System;
using System.Collections.Generic;
using System.Diagnostics;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Persistent, ego-connected physical free-space observer.
    ///
    /// The map grows OUTWARD from the currently reachable surface instead of
    /// sampling arbitrary road-aligned points. Speed changes the time horizon:
    /// near space is wide/dense, mid-field is narrower, and far-field follows
    /// the momentum/route tube. GTA road classification is annotation only.
    ///
    /// SpatialPlannerV1 still owns trajectory generation, but its final choices
    /// can be physically vetoed by a small, budgeted map/object/foliage sweep.
    internal sealed class PhysicalSurfaceMap
    {
        internal enum SurfaceState
        {
            Unknown,
            NoSurface,
            Traversable,
        }

        internal sealed class Cell
        {
            public CellKey Key;
            public Vector3 SamplePosition;
            public Vector3 Position;
            public SurfaceState State;
            public bool RoadSemantic;
            public bool Reachable;
            public bool StaticObstacle;
            public int ComponentId = -1;
            public float ClearanceM;
            public int LastWantedMs;
            public int LastSampleMs;
            public int LastObstacleMs;

            public bool Traversable =>
                State == SurfaceState.Traversable && !StaticObstacle;
        }

        internal struct DebugEdge
        {
            public Vector3 A;
            public Vector3 B;
            public bool Open;
        }

        internal struct DebugObstacle
        {
            public Vector3 Position;
            public Vector3 Normal;
        }

        internal struct PointQuery
        {
            public bool Known;
            public bool Reachable;
            public bool StaticObstacle;
            public bool RoadSemantic;
            public float SurfaceZ;
            public float ClearanceM;
        }

        internal struct TrajectoryCheck
        {
            public bool Available;
            public bool Blocked;
            public bool BudgetExhausted;
            public float BlockedS;
            public Vector3 HitPosition;
            public int Rays;
            public float Ms;
            public string Detail;
        }

        internal struct CellKey : IEquatable<CellKey>
        {
            public int X;
            public int Y;
            public int Layer;

            public bool Equals(CellKey other)
            {
                return X == other.X && Y == other.Y && Layer == other.Layer;
            }

            public override bool Equals(object obj)
            {
                return obj is CellKey && Equals((CellKey)obj);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int h = X;
                    h = h * 397 ^ Y;
                    h = h * 397 ^ Layer;
                    return h;
                }
            }
        }

        private struct FrontierRequest
        {
            public CellKey Key;
            public Vector3 Position;
            public float HintZ;
            public float Priority;
            public CellKey ParentKey;
            public bool HasParent;
        }

        private struct EdgeKey : IEquatable<EdgeKey>
        {
            public CellKey A;
            public CellKey B;

            public bool Equals(EdgeKey other)
            {
                return A.Equals(other.A) && B.Equals(other.B);
            }

            public override bool Equals(object obj)
            {
                return obj is EdgeKey && Equals((EdgeKey)obj);
            }

            public override int GetHashCode()
            {
                unchecked { return A.GetHashCode() * 397 ^ B.GetHashCode(); }
            }
        }

        private sealed class EdgeInfo
        {
            public bool Known;
            public bool Open;
            public int LastVerifiedMs;
        }

        public readonly List<Cell> DebugCells = new List<Cell>(220);
        public readonly List<DebugEdge> DebugEdges = new List<DebugEdge>(80);
        public readonly List<DebugObstacle> DebugObstacles = new List<DebugObstacle>(24);
        public string Detail { get; private set; } = "";

        private readonly Dictionary<CellKey, Cell> cells =
            new Dictionary<CellKey, Cell>();
        private readonly List<FrontierRequest> frontier =
            new List<FrontierRequest>(420);
        private readonly HashSet<CellKey> frontierKeys =
            new HashSet<CellKey>();
        private readonly Dictionary<EdgeKey, EdgeInfo> edges =
            new Dictionary<EdgeKey, EdgeInfo>();
        private readonly List<Vector3> guidePoints =
            new List<Vector3>(32);
        private readonly Queue<Cell> flood = new Queue<Cell>(420);
        private readonly Queue<Cell> distanceFlood = new Queue<Cell>(420);

        private Vehicle lastEgoVehicle;
        private Vector3 lastEgoPos;
        private float lastEgoHeading;
        private float lastEgoSpeed;
        private int lastNowMs;

        private int lastFrontierRefreshMs = -100000;
        private int lastGroundBatchMs = -100000;
        private int lastGroundSamples;
        private int lastObstacleRayMs = -100000;
        private int lastGraphMs = -100000;
        private int lastEvictMs = -100000;
        private int lastDetailMs = -100000;
        private bool graphDirty = true;
        private int obstacleRayCursor;

        private float horizonM;
        private float nearM;
        private float midM;
        private int activeReachable;
        private int activeComponents;

        private int groundOk;
        private int groundMiss;
        private int groundLayerReject;
        private int frontierExpanded;
        private int obstacleRays;
        private int obstacleHits;
        private int obstacleNotReady;
        private int obstacleFailed;
        private int trajectorySweeps;
        private int trajectorySweepHits;
        private int trajectorySweepBudgetStops;
        private int edgeChecks;
        private int edgeBlocks;
        private int edgeBudgetStops;
        private float lastEdgeMs;
        private float peakEdgeMs;
        private float lastSweepMs;
        private float peakSweepMs;
        private float lastTickMs;
        private float peakTickMs;

        private const float GridM = 3.0f;
        private const float LayerBucketM = 4.0f;
        private const float LayerAcceptanceM = 3.0f;
        private const float MaxNeighborStepM = 1.35f;
        private const float BackKeepM = 10f;

        private const float FutureTimeS = 4.0f;
        private const float MinHorizonM = 28f;
        private const float MaxHorizonM = 100f;
        private const float NearTimeS = 1.0f;
        private const float MidTimeS = 2.2f;
        private const float NearHalfWidthM = 18f;
        private const float MidHalfWidthM = 14f;
        private const float FarHalfWidthM = 9f;

        private const int FrontierRefreshMs = 180;
        private const int GroundBatchIntervalMs = 45;
        private const int GroundMaxSamplesPerBatch = 18;
        private const double GroundBudgetMs = 0.60;
        private const int CellFreshMs = 6500;
        private const int EdgeFreshMs = 12000;
        private const int EdgeChecksPerBatch = 5;
        private const double EdgeBudgetMs = 0.55;
        private const float EdgeProbeHeightM = 0.90f;

        private const int ObstacleRayIntervalMs = 100;
        private const float ObstacleRayHeightM = 1.15f;
        private const float ObstacleRayRangeM = 32f;
        private const int ObstaclePersistenceMs = 3000;

        private const int TrajectorySweepMaxRays = 6;
        private const double TrajectorySweepBudgetMs = 0.55;
        private const float TrajectorySweepHeightM = 0.85f;
        private const float TrajectorySweepDistanceM = 42f;

        private static readonly int[] DirX =
            { 1, -1, 0, 0, 1, 1, -1, -1 };
        private static readonly int[] DirY =
            { 0, 0, 1, -1, 1, -1, 1, -1 };

        private static readonly float[] ObstacleAnglesDeg =
        {
            -70f, -45f, -25f, -10f, 0f, 10f, 25f, 45f, 70f
        };

        private static readonly IntersectFlags StaticIntersectFlags =
            IntersectFlags.Map | IntersectFlags.Objects | IntersectFlags.Foliage;

        public void Reset()
        {
            cells.Clear();
            frontier.Clear();
            frontierKeys.Clear();
            edges.Clear();
            guidePoints.Clear();
            flood.Clear();
            distanceFlood.Clear();
            DebugCells.Clear();
            DebugEdges.Clear();
            DebugObstacles.Clear();

            lastEgoVehicle = null;
            lastEgoPos = Vector3.Zero;
            lastEgoHeading = 0f;
            lastEgoSpeed = 0f;
            lastNowMs = 0;

            lastFrontierRefreshMs = -100000;
            lastGroundBatchMs = -100000;
            lastGroundSamples = 0;
            lastObstacleRayMs = -100000;
            lastGraphMs = -100000;
            lastEvictMs = -100000;
            lastDetailMs = -100000;
            graphDirty = true;
            obstacleRayCursor = 0;

            horizonM = MinHorizonM;
            nearM = 12f;
            midM = 22f;
            activeReachable = 0;
            activeComponents = 0;

            groundOk = 0;
            groundMiss = 0;
            groundLayerReject = 0;
            frontierExpanded = 0;
            obstacleRays = 0;
            obstacleHits = 0;
            obstacleNotReady = 0;
            obstacleFailed = 0;
            trajectorySweeps = 0;
            trajectorySweepHits = 0;
            trajectorySweepBudgetStops = 0;
            edgeChecks = 0;
            edgeBlocks = 0;
            edgeBudgetStops = 0;
            lastEdgeMs = 0f;
            peakEdgeMs = 0f;
            lastSweepMs = 0f;
            peakSweepMs = 0f;
            lastTickMs = 0f;
            peakTickMs = 0f;
            Detail = "";
        }

        public void Tick(
            Vehicle egoVehicle,
            DrivingReference.Result reference,
            RaceRoute route,
            Vector3 egoPos,
            float egoHeading,
            float egoSpeed,
            int nowMs,
            bool finishGoalMode = false,
            Vector3 finishGoal = default(Vector3))
        {
            long perfStart = Stopwatch.GetTimestamp();

            lastEgoVehicle = egoVehicle;
            lastEgoPos = egoPos;
            lastEgoHeading = egoHeading;
            lastEgoSpeed = Math.Max(0f, egoSpeed);
            lastNowMs = nowMs;

            horizonM = RaceMath.Clamp(
                12f + Math.Max(lastEgoSpeed, 4f) * FutureTimeS,
                MinHorizonM, MaxHorizonM);
            nearM = RaceMath.Clamp(
                8f + Math.Max(lastEgoSpeed, 4f) * NearTimeS,
                12f, 30f);
            midM = RaceMath.Clamp(
                10f + Math.Max(lastEgoSpeed, 4f) * MidTimeS,
                nearM + 6f, Math.Min(horizonM, 65f));

            BuildGuide(reference, route, finishGoalMode, finishGoal);

            if (nowMs - lastFrontierRefreshMs >= FrontierRefreshMs)
            {
                lastFrontierRefreshMs = nowMs;
                RebuildFrontier(nowMs);
            }

            if (nowMs - lastGroundBatchMs >= GroundBatchIntervalMs)
            {
                lastGroundBatchMs = nowMs;
                SampleFrontier(nowMs);
            }

            if (nowMs - lastObstacleRayMs >= ObstacleRayIntervalMs)
            {
                lastObstacleRayMs = nowMs;
                ScanOneObstacleRay(nowMs);
            }

            if (graphDirty || nowMs - lastGraphMs >= 260)
            {
                lastGraphMs = nowMs;
                RebuildConnectivity(nowMs);
                BuildDebugSnapshot(nowMs);
                graphDirty = false;
            }

            if (nowMs - lastEvictMs >= 1000)
            {
                lastEvictMs = nowMs;
                EvictIrrelevant(nowMs);
            }

            lastTickMs = (float)((Stopwatch.GetTimestamp() - perfStart)
                * 1000.0 / Stopwatch.Frequency);
            if (lastTickMs > peakTickMs) peakTickMs = lastTickMs;

            if (nowMs - lastDetailMs >= 350)
            {
                lastDetailMs = nowMs;
                UpdateDetail();
            }
        }

        /// Positive physical evidence lookup. Unknown / no-ground cells are
        /// deliberately NOT returned as negative evidence: only a sampled
        /// traversable surface may override the legacy road model.
        public bool TryQueryPoint(Vector3 p, out PointQuery query)
        {
            query = new PointQuery
            {
                Known = false,
                Reachable = false,
                StaticObstacle = false,
                RoadSemantic = false,
                SurfaceZ = p.Z,
                ClearanceM = 0f,
            };

            int x = (int)Math.Round(p.X / GridM);
            int y = (int)Math.Round(p.Y / GridM);
            int layer = (int)Math.Round(p.Z / LayerBucketM);
            Cell best = null;
            float bestDz = float.MaxValue;

            for (int dl = -1; dl <= 1; dl++)
            {
                CellKey key = new CellKey
                {
                    X = x,
                    Y = y,
                    Layer = layer + dl,
                };
                Cell c;
                if (!cells.TryGetValue(key, out c)) continue;
                if (c.State != SurfaceState.Traversable) continue;

                float dz = Math.Abs(c.Position.Z - p.Z);
                if (dz > 2.8f || dz >= bestDz) continue;
                bestDz = dz;
                best = c;
            }

            if (best == null) return false;

            query.Known = true;
            query.Reachable = best.Reachable && !best.StaticObstacle;
            query.StaticObstacle = best.StaticObstacle;
            query.RoadSemantic = best.RoadSemantic;
            query.SurfaceZ = best.Position.Z;
            query.ClearanceM = best.ClearanceM;
            return true;
        }

        /// Conservative physical veto for a final trajectory candidate.
        ///
        /// This is deliberately small: at most eight synchronous LOS probes
        /// across the next ~42 m, with a strict sub-millisecond budget. The
        /// broad free-space map answers "where can I drive"; this answers
        /// "does this exact line hit a wall/tree/guardrail?".
        public TrajectoryCheck CheckTrajectory(
            IList<Vector3> path,
            IList<float> stationS,
            float egoHalfWidth)
        {
            var result = new TrajectoryCheck
            {
                Available = lastEgoVehicle != null,
                Blocked = false,
                BudgetExhausted = false,
                BlockedS = -1f,
                HitPosition = Vector3.Zero,
                Rays = 0,
                Ms = 0f,
                Detail = "",
            };

            if (path == null || path.Count < 3
                || stationS == null || stationS.Count != path.Count
                || lastEgoVehicle == null)
            {
                result.Detail = "unavailable";
                return result;
            }

            long perfStart = Stopwatch.GetTimestamp();
            float halfWidth = RaceMath.Clamp(egoHalfWidth * 0.82f, 0.55f, 1.25f);
            float maxS = Math.Min(
                TrajectorySweepDistanceM,
                Math.Max(24f, lastEgoSpeed * 2.0f + 8f));

            int i = 1;
            // Alternate which side gets the first rail across replans so a
            // narrow obstacle beside the centerline cannot live forever on
            // the consistently unsampled side.
            bool sideLeft = (trajectorySweeps & 1) == 0;
            while (i < path.Count - 1
                && result.Rays < TrajectorySweepMaxRays)
            {
                if (stationS[i] > maxS) break;

                int j = i + 1;
                while (j < path.Count - 1
                    && stationS[j] - stationS[i] < 7.0f
                    && stationS[j] <= maxS)
                    j++;

                if (j >= path.Count) j = path.Count - 1;
                if (j <= i) break;

                if (ElapsedMs(perfStart) >= TrajectorySweepBudgetMs
                    && result.Rays > 0)
                {
                    result.BudgetExhausted = true;
                    trajectorySweepBudgetStops++;
                    break;
                }

                Vector3 a = path[i];
                Vector3 b = path[j];
                Vector3 dir = RaceMath.FlatNormalize(
                    new Vector3(b.X - a.X, b.Y - a.Y, 0f));
                if (RaceMath.FlatLength(dir) < 0.1f)
                {
                    i = j;
                    continue;
                }
                Vector3 left = new Vector3(-dir.Y, dir.X, 0f);

                // Center rail first.
                if (CastStaticRail(
                    a, b, Vector3.Zero,
                    stationS[i], ref result))
                    break;

                if (result.Rays >= TrajectorySweepMaxRays) break;
                if (ElapsedMs(perfStart) >= TrajectorySweepBudgetMs)
                {
                    result.BudgetExhausted = true;
                    trajectorySweepBudgetStops++;
                    break;
                }

                // Alternate left/right side rails between segments. Over
                // consecutive replans both vehicle sides are sampled without
                // tripling native cost every frame.
                float sign = sideLeft ? 1f : -1f;
                sideLeft = !sideLeft;
                Vector3 side = new Vector3(
                    left.X * halfWidth * sign,
                    left.Y * halfWidth * sign,
                    0f);
                if (CastStaticRail(
                    a, b, side,
                    stationS[i], ref result))
                    break;

                i = j;
            }

            result.Ms = (float)ElapsedMs(perfStart);
            lastSweepMs = result.Ms;
            if (lastSweepMs > peakSweepMs) peakSweepMs = lastSweepMs;
            trajectorySweeps++;
            if (result.Blocked) trajectorySweepHits++;

            result.Detail = result.Blocked
                ? $"blocked@{result.BlockedS:F0};rays={result.Rays};ms={result.Ms:F2}"
                : $"clear;rays={result.Rays};budget={(result.BudgetExhausted ? 1 : 0)};ms={result.Ms:F2}";
            return result;
        }

        private bool CastStaticRail(
            Vector3 a,
            Vector3 b,
            Vector3 offset,
            float stationS,
            ref TrajectoryCheck result)
        {
            Vector3 start = new Vector3(
                a.X + offset.X,
                a.Y + offset.Y,
                a.Z + TrajectorySweepHeightM);
            Vector3 end = new Vector3(
                b.X + offset.X,
                b.Y + offset.Y,
                b.Z + TrajectorySweepHeightM);

            ShapeTestHandle handle = default(ShapeTestHandle);
            try
            {
                handle = ShapeTest.StartExpensiveSyncTestLOSProbe(
                    start, end,
                    StaticIntersectFlags,
                    lastEgoVehicle,
                    ShapeTestOptions.Default);
            }
            catch { }

            result.Rays++;
            if (handle.IsRequestFailed)
                return false;

            ShapeTestStatus status = ShapeTestStatus.NonExistent;
            ShapeTestResult shape = default(ShapeTestResult);
            try { status = handle.GetResult(out shape); }
            catch { status = ShapeTestStatus.NonExistent; }

            if (status != ShapeTestStatus.Ready || !shape.DidHit)
                return false;

            result.Blocked = true;
            result.BlockedS = stationS;
            result.HitPosition = shape.HitPosition;
            DebugObstacles.Add(new DebugObstacle
            {
                Position = shape.HitPosition,
                Normal = shape.SurfaceNormal,
            });
            if (DebugObstacles.Count > 24)
                DebugObstacles.RemoveAt(0);
            MarkObstacle(shape.HitPosition, lastNowMs);
            return true;
        }

        private void BuildGuide(
            DrivingReference.Result reference,
            RaceRoute route,
            bool finishGoalMode,
            Vector3 finishGoal)
        {
            guidePoints.Clear();

            if (finishGoalMode)
            {
                Vector3 delta = new Vector3(
                    finishGoal.X - lastEgoPos.X,
                    finishGoal.Y - lastEgoPos.Y,
                    0f);
                float distance = RaceMath.FlatLength(delta);
                Vector3 dir = RaceMath.FlatNormalize(delta);
                if (distance > 0.5f
                    && RaceMath.FlatLength(dir) > 0.1f)
                {
                    float limit = Math.Min(distance, horizonM);
                    float step = Math.Max(6f, limit / 10f);
                    for (float d = 0f; d <= limit; d += step)
                    {
                        guidePoints.Add(new Vector3(
                            lastEgoPos.X + dir.X * d,
                            lastEgoPos.Y + dir.Y * d,
                            lastEgoPos.Z));
                        if (guidePoints.Count >= 24) break;
                    }
                    guidePoints.Add(new Vector3(
                        lastEgoPos.X + dir.X * limit,
                        lastEgoPos.Y + dir.Y * limit,
                        lastEgoPos.Z));
                }
                return;
            }

            if (reference != null
                && reference.Path != null
                && reference.Path.Count >= 2)
            {
                for (int i = 0; i < reference.Path.Count; i += 3)
                {
                    Vector3 p = reference.Path[i];
                    if (RaceMath.FlatDistance(lastEgoPos, p) > horizonM + 18f)
                        continue;
                    guidePoints.Add(p);
                    if (guidePoints.Count >= 28) break;
                }
            }

            if (guidePoints.Count < 4
                && route != null && route.Built)
            {
                float step = Math.Max(7f, horizonM / 10f);
                for (float ds = 0f; ds <= horizonM; ds += step)
                {
                    guidePoints.Add(route.PointAtS(route.AlongS + ds));
                    if (guidePoints.Count >= 24) break;
                }
            }
        }

        private void RebuildFrontier(int nowMs)
        {
            frontier.Clear();
            frontierKeys.Clear();

            Cell egoCell = FindNearestReachableCell(lastEgoPos, 6f);
            if (egoCell == null)
            {
                // Bootstrap ONLY the surface directly under the ego. The old
                // 3x3 bootstrap could jump across a fence/guardrail before any
                // edge had been physically verified.
                AddFrontier(
                    CenterFor(KeyFor(
                        lastEgoPos.X, lastEgoPos.Y, lastEgoPos.Z),
                        lastEgoPos.Z),
                    lastEgoPos.Z,
                    -1000f,
                    default(CellKey),
                    false,
                    nowMs);
            }

            // IMPORTANT: AddNeighborsToFrontier may create new cells. Never
            // mutate the dictionary while enumerating cells.Values.
            var sources = new List<Cell>();
            foreach (Cell c in cells.Values)
            {
                if (!c.Reachable || !c.Traversable) continue;
                if (!InsideFutureEnvelope(c.Position)) continue;
                sources.Add(c);
            }

            for (int i = 0; i < sources.Count; i++)
                AddNeighborsToFrontier(sources[i], nowMs);

            frontier.Sort((a, b) => a.Priority.CompareTo(b.Priority));
        }

        private void AddNeighborsToFrontier(Cell parent, int nowMs)
        {
            for (int d = 0; d < DirX.Length; d++)
            {
                CellKey key = new CellKey
                {
                    X = parent.Key.X + DirX[d],
                    Y = parent.Key.Y + DirY[d],
                    Layer = parent.Key.Layer,
                };
                Vector3 p = new Vector3(
                    key.X * GridM,
                    key.Y * GridM,
                    parent.Position.Z);

                if (!InsideFutureEnvelope(p)) continue;

                Cell existing;
                if (cells.TryGetValue(key, out existing)
                    && existing.State != SurfaceState.Unknown
                    && nowMs - existing.LastSampleMs < CellFreshMs)
                {
                    existing.LastWantedMs = nowMs;

                    // A sampled traversable cell can still be disconnected only
                    // because the boundary between it and this parent has not
                    // been verified yet (or another parent was blocked).
                    if (existing.Traversable
                        && !existing.Reachable
                        && parent.Reachable)
                    {
                        AddFrontier(
                            p, parent.Position.Z,
                            FrontierPriority(p) - 2f,
                            parent.Key, true, nowMs);
                    }
                    continue;
                }

                AddFrontier(
                    p, parent.Position.Z,
                    FrontierPriority(p),
                    parent.Key, true, nowMs);
            }
        }

        private void AddFrontier(
            Vector3 world,
            float hintZ,
            float priority,
            CellKey parentKey,
            bool hasParent,
            int nowMs)
        {
            CellKey key = KeyFor(world.X, world.Y, hintZ);
            if (!frontierKeys.Add(key)) return;

            Cell cell;
            if (!cells.TryGetValue(key, out cell))
            {
                cell = new Cell
                {
                    Key = key,
                    SamplePosition = CenterFor(key, hintZ),
                    Position = CenterFor(key, hintZ),
                    State = SurfaceState.Unknown,
                    LastWantedMs = nowMs,
                    LastSampleMs = -100000,
                    LastObstacleMs = -100000,
                };
                cells.Add(key, cell);
            }
            else
            {
                cell.SamplePosition = CenterFor(key, hintZ);
                cell.LastWantedMs = nowMs;
            }

            frontier.Add(new FrontierRequest
            {
                Key = key,
                Position = cell.SamplePosition,
                HintZ = hintZ,
                Priority = priority,
                ParentKey = parentKey,
                HasParent = hasParent,
            });
        }

        private void SampleFrontier(int nowMs)
        {
            lastGroundSamples = 0;
            if (frontier.Count == 0) return;

            long groundStart = Stopwatch.GetTimestamp();
            long edgeStart = 0;
            int edgeChecksThisBatch = 0;

            while (lastGroundSamples < GroundMaxSamplesPerBatch
                && frontier.Count > 0)
            {
                if (lastGroundSamples > 0
                    && ElapsedMs(groundStart) >= GroundBudgetMs)
                    break;

                frontier.Sort((a, b) => a.Priority.CompareTo(b.Priority));
                FrontierRequest req = frontier[0];
                frontier.RemoveAt(0);
                frontierKeys.Remove(req.Key);

                Cell cell;
                if (!cells.TryGetValue(req.Key, out cell)) continue;

                bool freshKnown = cell.State != SurfaceState.Unknown
                    && nowMs - cell.LastSampleMs < CellFreshMs;

                if (!freshKnown)
                {
                    float groundZ = 0f;
                    bool found = false;
                    try
                    {
                        found = World.GetGroundHeight(
                            new Vector3(
                                req.Position.X,
                                req.Position.Y,
                                req.HintZ + 4.5f),
                            out groundZ);
                    }
                    catch { found = false; }

                    cell.LastSampleMs = nowMs;
                    cell.LastWantedMs = nowMs;
                    lastGroundSamples++;

                    if (!found)
                    {
                        groundMiss++;
                        cell.State = SurfaceState.NoSurface;
                        cell.Reachable = false;
                        cell.ComponentId = -1;
                        graphDirty = true;
                        continue;
                    }

                    float dzLayer = Math.Abs(groundZ - req.HintZ);
                    if (dzLayer > LayerAcceptanceM)
                    {
                        groundLayerReject++;
                        cell.State = SurfaceState.NoSurface;
                        cell.Reachable = false;
                        cell.ComponentId = -1;
                        graphDirty = true;
                        continue;
                    }

                    groundOk++;
                    cell.Position = new Vector3(
                        req.Position.X,
                        req.Position.Y,
                        groundZ);
                    cell.State = SurfaceState.Traversable;

                    bool road = false;
                    try
                    {
                        road = Function.Call<bool>(
                            Hash.IS_POINT_ON_ROAD,
                            cell.Position.X,
                            cell.Position.Y,
                            cell.Position.Z + 0.15f,
                            0);
                    }
                    catch { road = false; }
                    cell.RoadSemantic = road;
                }

                if (!cell.Traversable)
                    continue;

                bool connected = RaceMath.FlatDistance(
                    cell.Position, lastEgoPos) <= GridM * 0.75f;

                if (req.HasParent)
                {
                    Cell parent;
                    if (cells.TryGetValue(req.ParentKey, out parent)
                        && parent.Reachable
                        && parent.Traversable
                        && Math.Abs(parent.Position.Z - cell.Position.Z)
                            <= MaxNeighborStepM)
                    {
                        bool edgeOpen;
                        bool resolved = TryResolveEdge(
                            parent, cell, nowMs,
                            ref edgeStart, ref edgeChecksThisBatch,
                            out edgeOpen);

                        if (!resolved)
                        {
                            // Edge budget is exhausted. Keep the sampled cell,
                            // but do NOT call it reachable until the physical
                            // crossing itself has been checked.
                            AddFrontier(
                                cell.Position,
                                parent.Position.Z,
                                req.Priority + 0.5f,
                                parent.Key,
                                true,
                                nowMs);
                            edgeBudgetStops++;
                            break;
                        }

                        connected = edgeOpen;
                    }
                }

                cell.Reachable = connected;
                graphDirty = true;

                if (connected)
                {
                    frontierExpanded++;
                    AddNeighborsToFrontier(cell, nowMs);
                }
            }
        }

        private bool TryResolveEdge(
            Cell a,
            Cell b,
            int nowMs,
            ref long edgeStart,
            ref int edgeChecksThisBatch,
            out bool open)
        {
            open = false;
            if (a == null || b == null
                || !a.Traversable || !b.Traversable)
                return true;

            if (Math.Abs(a.Position.Z - b.Position.Z) > MaxNeighborStepM)
                return true;

            Vector3 mid = new Vector3(
                (a.Position.X + b.Position.X) * 0.5f,
                (a.Position.Y + b.Position.Y) * 0.5f,
                (a.Position.Z + b.Position.Z) * 0.5f);

            // Far future remains coarse. Near + mid future requires physical
            // boundary verification so fences/guardrails split components.
            if (RaceMath.FlatDistance(mid, lastEgoPos) > VerifiedEdgeRadius())
            {
                open = true;
                return true;
            }

            EdgeKey key = MakeEdgeKey(a.Key, b.Key);
            EdgeInfo info;
            if (edges.TryGetValue(key, out info)
                && info.Known
                && nowMs - info.LastVerifiedMs < EdgeFreshMs)
            {
                open = info.Open;
                return true;
            }

            if (edgeChecksThisBatch >= EdgeChecksPerBatch)
                return false;

            if (edgeStart == 0)
                edgeStart = Stopwatch.GetTimestamp();
            else if (ElapsedMs(edgeStart) >= EdgeBudgetMs)
                return false;

            Vector3 start = new Vector3(
                a.Position.X,
                a.Position.Y,
                a.Position.Z + EdgeProbeHeightM);
            Vector3 end = new Vector3(
                b.Position.X,
                b.Position.Y,
                b.Position.Z + EdgeProbeHeightM);

            long oneStart = Stopwatch.GetTimestamp();
            ShapeTestHandle handle = default(ShapeTestHandle);
            try
            {
                handle = ShapeTest.StartExpensiveSyncTestLOSProbe(
                    start, end,
                    StaticIntersectFlags,
                    lastEgoVehicle,
                    ShapeTestOptions.Default);
            }
            catch { }

            edgeChecksThisBatch++;
            edgeChecks++;
            lastEdgeMs = (float)ElapsedMs(oneStart);
            if (lastEdgeMs > peakEdgeMs) peakEdgeMs = lastEdgeMs;

            if (handle.IsRequestFailed)
                return false;

            ShapeTestStatus status = ShapeTestStatus.NonExistent;
            ShapeTestResult shape = default(ShapeTestResult);
            try { status = handle.GetResult(out shape); }
            catch { status = ShapeTestStatus.NonExistent; }

            if (status != ShapeTestStatus.Ready)
                return false;

            open = !shape.DidHit;
            if (!edges.TryGetValue(key, out info))
            {
                info = new EdgeInfo();
                edges.Add(key, info);
            }
            info.Known = true;
            info.Open = open;
            info.LastVerifiedMs = nowMs;

            if (!open)
            {
                edgeBlocks++;
                DebugEdges.Add(new DebugEdge
                {
                    A = a.Position,
                    B = b.Position,
                    Open = false,
                });
                if (DebugEdges.Count > 80)
                    DebugEdges.RemoveAt(0);

                DebugObstacles.Add(new DebugObstacle
                {
                    Position = shape.HitPosition,
                    Normal = shape.SurfaceNormal,
                });
                if (DebugObstacles.Count > 24)
                    DebugObstacles.RemoveAt(0);
            }

            return true;
        }

        private bool InsideFutureEnvelope(Vector3 p)
        {
            Vector3 rel = new Vector3(
                p.X - lastEgoPos.X,
                p.Y - lastEgoPos.Y,
                0f);
            float dist = RaceMath.FlatLength(rel);
            if (dist <= nearM) return true;
            if (dist > horizonM) return false;

            Vector3 f = RaceMath.FlatNormalize(
                RaceMath.VectorFromHeading(lastEgoHeading));
            Vector3 l = new Vector3(-f.Y, f.X, 0f);
            float along = RaceMath.FlatDot(rel, f);
            float lateral = Math.Abs(RaceMath.FlatDot(rel, l));

            if (along < -BackKeepM) return false;

            float guideDist = GuideDistance(p);
            float halfWidth = dist <= midM
                ? MidHalfWidthM
                : FarHalfWidthM;

            // Route curvature can carry the future tube outside the current
            // ego-heading cone, so guide proximity is an alternate admission.
            return lateral <= halfWidth
                || guideDist <= halfWidth + 5f;
        }

        private float VerifiedEdgeRadius()
        {
            // Verify slightly beyond the mid-field boundary so increasing
            // speed does not turn today's coarse far cells into tomorrow's
            // suddenly disconnected near cells before the edge checker catches
            // up. The lead grows with speed but remains budgeted.
            float lead = RaceMath.Clamp(lastEgoSpeed * 0.80f, 8f, 18f);
            return Math.Min(horizonM, midM + lead);
        }

        private float FrontierPriority(Vector3 p)
        {
            Vector3 rel = new Vector3(
                p.X - lastEgoPos.X,
                p.Y - lastEgoPos.Y,
                0f);
            float dist = RaceMath.FlatLength(rel);
            float guideDist = GuideDistance(p);

            if (dist <= nearM)
                return dist * 0.16f + guideDist * 0.04f;

            if (dist <= midM)
                return 10f + dist * 0.10f + guideDist * 0.18f;

            return 24f + dist * 0.07f + guideDist * 0.30f;
        }

        private float GuideDistance(Vector3 p)
        {
            if (guidePoints.Count == 0) return 0f;
            float best = float.MaxValue;
            for (int i = 0; i < guidePoints.Count; i++)
            {
                float d = RaceMath.FlatDistance(p, guidePoints[i]);
                if (d < best) best = d;
            }
            return best == float.MaxValue ? 0f : best;
        }

        private void ScanOneObstacleRay(int nowMs)
        {
            if (lastEgoVehicle == null) return;
            if (obstacleRayCursor == 0)
                DebugObstacles.Clear();

            int i = obstacleRayCursor++;
            if (obstacleRayCursor >= ObstacleAnglesDeg.Length)
                obstacleRayCursor = 0;

            Vector3 start = new Vector3(
                lastEgoPos.X,
                lastEgoPos.Y,
                lastEgoPos.Z + ObstacleRayHeightM);
            float h = lastEgoHeading + ObstacleAnglesDeg[i];
            Vector3 dir = RaceMath.FlatNormalize(
                RaceMath.VectorFromHeading(h));
            float range = Math.Min(
                ObstacleRayRangeM,
                Math.Max(18f, lastEgoSpeed * 1.7f + 8f));
            Vector3 end = new Vector3(
                start.X + dir.X * range,
                start.Y + dir.Y * range,
                start.Z);

            ShapeTestHandle handle = default(ShapeTestHandle);
            try
            {
                handle = ShapeTest.StartExpensiveSyncTestLOSProbe(
                    start, end,
                    StaticIntersectFlags,
                    lastEgoVehicle,
                    ShapeTestOptions.Default);
            }
            catch { }

            obstacleRays++;
            if (handle.IsRequestFailed)
            {
                obstacleFailed++;
                return;
            }

            ShapeTestStatus status = ShapeTestStatus.NonExistent;
            ShapeTestResult shape = default(ShapeTestResult);
            try { status = handle.GetResult(out shape); }
            catch { status = ShapeTestStatus.NonExistent; }

            if (status != ShapeTestStatus.Ready)
            {
                obstacleNotReady++;
                return;
            }
            if (!shape.DidHit) return;

            obstacleHits++;
            DebugObstacles.Add(new DebugObstacle
            {
                Position = shape.HitPosition,
                Normal = shape.SurfaceNormal,
            });
            MarkObstacle(shape.HitPosition, nowMs);
        }

        private void MarkObstacle(Vector3 hit, int nowMs)
        {
            Cell best = null;
            float bestScore = float.MaxValue;

            foreach (Cell c in cells.Values)
            {
                if (c.State != SurfaceState.Traversable) continue;
                float d = RaceMath.FlatDistance(c.Position, hit);
                if (d > GridM * 1.2f) continue;
                float dz = Math.Abs(c.Position.Z - hit.Z);
                if (dz > 2.6f) continue;
                float score = d + dz * 0.35f;
                if (score >= bestScore) continue;
                bestScore = score;
                best = c;
            }

            if (best == null) return;
            best.StaticObstacle = true;
            best.LastObstacleMs = nowMs;
            best.Reachable = false;
            graphDirty = true;
        }

        private void RebuildConnectivity(int nowMs)
        {
            foreach (Cell c in cells.Values)
            {
                if (c.StaticObstacle
                    && nowMs - c.LastObstacleMs > ObstaclePersistenceMs)
                    c.StaticObstacle = false;

                c.Reachable = false;
                c.ComponentId = -1;
                c.ClearanceM = 0f;
            }

            int component = 0;
            foreach (Cell start in cells.Values)
            {
                if (!start.Traversable || start.ComponentId >= 0) continue;
                FloodComponent(start, component++);
            }

            Cell egoCell = FindNearestTraversableCell(lastEgoPos, 6f);
            int reachableComponent = egoCell != null
                ? egoCell.ComponentId
                : -1;

            activeReachable = 0;
            activeComponents = 0;
            var activeComponentIds = new HashSet<int>();

            if (reachableComponent >= 0)
            {
                foreach (Cell c in cells.Values)
                {
                    c.Reachable = c.ComponentId == reachableComponent;
                    if (c.Reachable && InsideFutureEnvelope(c.Position))
                        activeReachable++;
                    if (InsideFutureEnvelope(c.Position)
                        && c.ComponentId >= 0)
                        activeComponentIds.Add(c.ComponentId);
                }
            }
            else
            {
                foreach (Cell c in cells.Values)
                    if (InsideFutureEnvelope(c.Position)
                        && c.ComponentId >= 0)
                        activeComponentIds.Add(c.ComponentId);
            }

            activeComponents = activeComponentIds.Count;
            ComputeClearance();
        }

        private void FloodComponent(Cell start, int componentId)
        {
            flood.Clear();
            start.ComponentId = componentId;
            flood.Enqueue(start);

            while (flood.Count > 0)
            {
                Cell c = flood.Dequeue();
                for (int d = 0; d < DirX.Length; d++)
                {
                    Cell n;
                    if (!TryFindNeighbor(c, d, out n)) continue;
                    if (n.ComponentId >= 0) continue;
                    if (!ConnectionOpen(c, n)) continue;
                    n.ComponentId = componentId;
                    flood.Enqueue(n);
                }
            }
        }

        private void ComputeClearance()
        {
            distanceFlood.Clear();
            const float inf = 9999f;

            foreach (Cell c in cells.Values)
            {
                if (!c.Reachable)
                {
                    c.ClearanceM = 0f;
                    continue;
                }

                c.ClearanceM = inf;
                bool boundary = false;
                for (int d = 0; d < 4; d++)
                {
                    Cell n;
                    if (!TryFindNeighbor(c, d, out n)
                        || !n.Reachable
                        || !ConnectionOpen(c, n))
                    {
                        boundary = true;
                        break;
                    }
                }

                if (boundary)
                {
                    c.ClearanceM = GridM * 0.5f;
                    distanceFlood.Enqueue(c);
                }
            }

            while (distanceFlood.Count > 0)
            {
                Cell c = distanceFlood.Dequeue();
                for (int d = 0; d < 4; d++)
                {
                    Cell n;
                    if (!TryFindNeighbor(c, d, out n)) continue;
                    if (!n.Reachable || !ConnectionOpen(c, n)) continue;

                    float nd = c.ClearanceM + GridM;
                    if (nd + 0.01f >= n.ClearanceM) continue;
                    n.ClearanceM = nd;
                    distanceFlood.Enqueue(n);
                }
            }

            foreach (Cell c in cells.Values)
                if (c.Reachable && c.ClearanceM >= inf)
                    c.ClearanceM = GridM * 0.5f;
        }

        private bool TryFindNeighbor(
            Cell c,
            int dir,
            out Cell neighbor)
        {
            neighbor = null;
            int nx = c.Key.X + DirX[dir];
            int ny = c.Key.Y + DirY[dir];
            float bestDz = float.MaxValue;

            for (int dl = -1; dl <= 1; dl++)
            {
                CellKey key = new CellKey
                {
                    X = nx,
                    Y = ny,
                    Layer = c.Key.Layer + dl,
                };
                Cell candidate;
                if (!cells.TryGetValue(key, out candidate)) continue;
                if (!candidate.Traversable) continue;

                float dz = Math.Abs(
                    candidate.Position.Z - c.Position.Z);
                if (dz > MaxNeighborStepM || dz >= bestDz)
                    continue;
                bestDz = dz;
                neighbor = candidate;
            }

            return neighbor != null;
        }

        private bool ConnectionOpen(Cell a, Cell b)
        {
            if (a == null || b == null) return false;
            if (!a.Traversable || !b.Traversable) return false;
            if (Math.Abs(a.Position.Z - b.Position.Z) > MaxNeighborStepM)
                return false;

            Vector3 mid = new Vector3(
                (a.Position.X + b.Position.X) * 0.5f,
                (a.Position.Y + b.Position.Y) * 0.5f,
                (a.Position.Z + b.Position.Z) * 0.5f);

            // Far future is intentionally coarse. Near/mid connectivity must
            // have an explicit physical edge result.
            if (RaceMath.FlatDistance(mid, lastEgoPos) > VerifiedEdgeRadius())
                return true;

            EdgeInfo info;
            return edges.TryGetValue(
                    MakeEdgeKey(a.Key, b.Key), out info)
                && info.Known
                && info.Open
                && lastNowMs - info.LastVerifiedMs < EdgeFreshMs;
        }

        private Cell FindNearestReachableCell(
            Vector3 p,
            float maxDist)
        {
            Cell best = null;
            float bestD = maxDist;
            foreach (Cell c in cells.Values)
            {
                if (!c.Reachable || !c.Traversable) continue;
                float dz = Math.Abs(c.Position.Z - p.Z);
                if (dz > 1.8f) continue;
                float d = RaceMath.FlatDistance(c.Position, p);
                if (d >= bestD) continue;
                bestD = d;
                best = c;
            }
            return best;
        }

        private Cell FindNearestTraversableCell(
            Vector3 p,
            float maxDist)
        {
            Cell best = null;
            float bestD = maxDist;
            foreach (Cell c in cells.Values)
            {
                if (!c.Traversable) continue;
                float dz = Math.Abs(c.Position.Z - p.Z);
                if (dz > 1.8f) continue;
                float d = RaceMath.FlatDistance(c.Position, p);
                if (d >= bestD) continue;
                bestD = d;
                best = c;
            }
            return best;
        }

        private void BuildDebugSnapshot(int nowMs)
        {
            DebugCells.Clear();
            DebugEdges.Clear();

            foreach (Cell c in cells.Values)
            {
                if (!InsideFutureEnvelope(c.SamplePosition)) continue;
                if (nowMs - c.LastWantedMs > 2500
                    && !c.Reachable)
                    continue;

                DebugCells.Add(c);
                if (DebugCells.Count >= 190) break;
            }

            // Only expose actual local discontinuities, not every open edge.
            foreach (Cell c in cells.Values)
            {
                if (DebugEdges.Count >= 80) break;
                if (!c.Reachable || !c.Traversable) continue;
                if (RaceMath.FlatDistance(c.Position, lastEgoPos) > nearM + 8f)
                    continue;

                for (int d = 0; d < 4 && DebugEdges.Count < 80; d++)
                {
                    Cell n;
                    if (!TryFindAnySurfaceNeighbor(c, d, out n)) continue;
                    if (ConnectionOpen(c, n)) continue;

                    DebugEdges.Add(new DebugEdge
                    {
                        A = c.Position,
                        B = n.Position,
                        Open = false,
                    });
                }
            }
        }

        private bool TryFindAnySurfaceNeighbor(
            Cell c,
            int dir,
            out Cell neighbor)
        {
            neighbor = null;
            int nx = c.Key.X + DirX[dir];
            int ny = c.Key.Y + DirY[dir];
            float bestDz = float.MaxValue;

            for (int dl = -1; dl <= 1; dl++)
            {
                CellKey key = new CellKey
                {
                    X = nx,
                    Y = ny,
                    Layer = c.Key.Layer + dl,
                };
                Cell candidate;
                if (!cells.TryGetValue(key, out candidate)) continue;
                if (candidate.State != SurfaceState.Traversable) continue;

                float dz = Math.Abs(candidate.Position.Z - c.Position.Z);
                if (dz >= bestDz) continue;
                bestDz = dz;
                neighbor = candidate;
            }
            return neighbor != null;
        }

        private void EvictIrrelevant(int nowMs)
        {
            var remove = new List<CellKey>();
            float keepRadius = horizonM + 18f;

            foreach (KeyValuePair<CellKey, Cell> pair in cells)
            {
                Cell c = pair.Value;
                float d = RaceMath.FlatDistance(
                    c.SamplePosition, lastEgoPos);
                bool behindFar = !InsideFutureEnvelope(c.SamplePosition)
                    && d > nearM + 8f;
                bool stale = nowMs - c.LastWantedMs > 5000;
                bool veryStale = nowMs - c.LastSampleMs > 18000;

                if (d > keepRadius
                    || (behindFar && stale)
                    || (veryStale && d > nearM))
                    remove.Add(pair.Key);
            }

            for (int i = 0; i < remove.Count; i++)
                cells.Remove(remove[i]);

            if (remove.Count > 0)
            {
                var removeEdges = new List<EdgeKey>();
                foreach (EdgeKey key in edges.Keys)
                {
                    if (!cells.ContainsKey(key.A)
                        || !cells.ContainsKey(key.B))
                        removeEdges.Add(key);
                }
                for (int i = 0; i < removeEdges.Count; i++)
                    edges.Remove(removeEdges[i]);

                graphDirty = true;
            }
        }

        private void UpdateDetail()
        {
            int traversable = 0;
            int reachable = 0;
            int reachableOffRoad = 0;
            int staticBlocked = 0;
            int noSurface = 0;
            float maxClear = 0f;

            foreach (Cell c in cells.Values)
            {
                if (!InsideFutureEnvelope(c.SamplePosition)) continue;

                if (c.State == SurfaceState.Traversable)
                {
                    if (c.StaticObstacle) staticBlocked++;
                    else traversable++;
                }
                else if (c.State == SurfaceState.NoSurface)
                {
                    noSurface++;
                }

                if (c.Reachable)
                {
                    reachable++;
                    if (!c.RoadSemantic) reachableOffRoad++;
                    if (c.ClearanceM > maxClear)
                        maxClear = c.ClearanceM;
                }
            }

            Detail =
                $"cells={cells.Count};frontier={frontier.Count};"
                + $"horizon={horizonM:F0};near={nearM:F0};mid={midM:F0};"
                + $"trav={traversable};reach={reachable};reachActive={activeReachable};"
                + $"reachOffRoad={reachableOffRoad};components={activeComponents};"
                + $"staticBlocked={staticBlocked};noSurface={noSurface};"
                + $"gOk={groundOk};gMiss={groundMiss};gLayer={groundLayerReject};"
                + $"gBatch={lastGroundSamples};expanded={frontierExpanded};"
                + $"edgeChecks={edgeChecks};edgeBlocks={edgeBlocks};"
                + $"edgeBudget={edgeBudgetStops};edgeMs={lastEdgeMs:F2};edgePeak={peakEdgeMs:F2};"
                + $"obsRays={obstacleRays};obsHits={obstacleHits};"
                + $"obsWait={obstacleNotReady};obsFail={obstacleFailed};"
                + $"sweeps={trajectorySweeps};sweepHits={trajectorySweepHits};"
                + $"sweepBudget={trajectorySweepBudgetStops};"
                + $"sweepMs={lastSweepMs:F2};sweepPeak={peakSweepMs:F2};"
                + $"maxClear={maxClear:F1};"
                + $"tickMs={lastTickMs:F2};peakMs={peakTickMs:F2}";
        }

        private static EdgeKey MakeEdgeKey(
            CellKey a,
            CellKey b)
        {
            if (CompareKey(a, b) <= 0)
                return new EdgeKey { A = a, B = b };
            return new EdgeKey { A = b, B = a };
        }

        private static int CompareKey(
            CellKey a,
            CellKey b)
        {
            if (a.X != b.X) return a.X.CompareTo(b.X);
            if (a.Y != b.Y) return a.Y.CompareTo(b.Y);
            return a.Layer.CompareTo(b.Layer);
        }

        private static CellKey KeyFor(
            float x,
            float y,
            float hintZ)
        {
            return new CellKey
            {
                X = (int)Math.Round(x / GridM),
                Y = (int)Math.Round(y / GridM),
                Layer = (int)Math.Round(hintZ / LayerBucketM),
            };
        }

        private static Vector3 CenterFor(
            CellKey key,
            float hintZ)
        {
            return new Vector3(
                key.X * GridM,
                key.Y * GridM,
                hintZ);
        }

        private static double ElapsedMs(long start)
        {
            return (Stopwatch.GetTimestamp() - start)
                * 1000.0 / Stopwatch.Frequency;
        }
    }
}
