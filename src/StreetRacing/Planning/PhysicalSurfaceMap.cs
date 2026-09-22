using System;
using System.Collections.Generic;
using System.Diagnostics;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Debug-only physical traversability observer.
    ///
    /// V3 deliberately avoids long-lived asynchronous shape tests. Ground
    /// geometry comes from GTA's direct ground-height query under a strict
    /// local Z-layer gate. Static obstacles are sampled sparsely with a small
    /// synchronous LOS fan. SpatialPlannerV1 does NOT consume this map yet.
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

            public bool Traversable => State == SurfaceState.Traversable && !StaticObstacle;
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

        private struct ProbeRequest
        {
            public CellKey Key;
            public Vector3 Position;
            public float HintZ;
            public float Priority;
        }

        public readonly List<Cell> DebugCells = new List<Cell>(260);
        public readonly List<DebugEdge> DebugEdges = new List<DebugEdge>(80);
        public readonly List<DebugObstacle> DebugObstacles = new List<DebugObstacle>(20);
        public string Detail { get; private set; } = "";

        private readonly Dictionary<CellKey, Cell> cells =
            new Dictionary<CellKey, Cell>();
        private readonly List<ProbeRequest> desired =
            new List<ProbeRequest>(300);
        private readonly HashSet<CellKey> desiredKeys =
            new HashSet<CellKey>();
        private readonly Queue<Cell> flood = new Queue<Cell>(320);
        private readonly Queue<Cell> distanceFlood = new Queue<Cell>(320);

        private int desiredCursor;
        private int lastDesiredRefreshMs = -100000;
        private int lastGroundBatchMs = -100000;
        private int lastObstacleScanMs = -100000;
        private int lastGraphMs = -100000;
        private int lastEvictMs = -100000;
        private bool graphDirty = true;
        private int obstaclePhase;

        private int groundOk;
        private int groundMiss;
        private int groundLayerReject;
        private int obstacleRays;
        private int obstacleHits;
        private int obstacleNotReady;
        private int obstacleFailed;
        private float lastTickMs;
        private float peakTickMs;

        private const float GridM = 3.0f;
        private const float LayerBucketM = 4.0f;
        private const float LayerAcceptanceM = 2.8f;
        private const float MaxNeighborStepM = 0.95f;
        private const float NearFieldRadiusM = 15f;
        private const float SampleAheadM = 65f;
        private const float SampleHalfWidthM = 15f;
        private const int MaxDesiredCells = 260;
        private const int GroundBatchIntervalMs = 75;
        private const int GroundSamplesPerBatch = 3;
        private const int CellFreshMs = 7000;
        private const int ObstacleScanIntervalMs = 350;
        private const float ObstacleRayHeightM = 1.25f;
        private const float ObstacleRayRangeM = 30f;
        private const int ObstaclePersistenceMs = 2500;

        private static readonly int[] DirX = { 1, -1, 0, 0 };
        private static readonly int[] DirY = { 0, 0, 1, -1 };

        private static readonly float[] ObstacleAnglesDeg =
        {
            -90f, -72f, -54f, -36f, -18f,
             0f,   18f,  36f,  54f,  72f, 90f
        };

        private static readonly IntersectFlags ObstacleIntersectFlags =
            IntersectFlags.Map | IntersectFlags.Objects | IntersectFlags.Foliage;

        public void Reset()
        {
            cells.Clear();
            desired.Clear();
            desiredKeys.Clear();
            flood.Clear();
            distanceFlood.Clear();
            DebugCells.Clear();
            DebugEdges.Clear();
            DebugObstacles.Clear();

            desiredCursor = 0;
            lastDesiredRefreshMs = -100000;
            lastGroundBatchMs = -100000;
            lastObstacleScanMs = -100000;
            lastGraphMs = -100000;
            lastEvictMs = -100000;
            graphDirty = true;
            obstaclePhase = 0;

            groundOk = 0;
            groundMiss = 0;
            groundLayerReject = 0;
            obstacleRays = 0;
            obstacleHits = 0;
            obstacleNotReady = 0;
            obstacleFailed = 0;
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
            int nowMs)
        {
            long perfStart = Stopwatch.GetTimestamp();

            if (nowMs - lastDesiredRefreshMs >= 350)
            {
                lastDesiredRefreshMs = nowMs;
                RebuildDesired(reference, route, egoPos, nowMs);
            }

            if (nowMs - lastGroundBatchMs >= GroundBatchIntervalMs)
            {
                lastGroundBatchMs = nowMs;
                SampleGround(nowMs);
            }

            if (nowMs - lastObstacleScanMs >= ObstacleScanIntervalMs)
            {
                lastObstacleScanMs = nowMs;
                ScanObstacleFan(egoVehicle, egoPos, egoHeading, nowMs);
            }

            if (graphDirty || nowMs - lastGraphMs >= 300)
            {
                lastGraphMs = nowMs;
                RebuildConnectivity(egoPos, nowMs);
                BuildDebugSnapshot(egoPos, nowMs);
                graphDirty = false;
            }

            if (nowMs - lastEvictMs >= 1500)
            {
                lastEvictMs = nowMs;
                EvictFarAndStale(egoPos, nowMs);
            }

            lastTickMs = (float)((Stopwatch.GetTimestamp() - perfStart)
                * 1000.0 / Stopwatch.Frequency);
            if (lastTickMs > peakTickMs) peakTickMs = lastTickMs;
            UpdateDetail();
        }

        private void RebuildDesired(
            DrivingReference.Result reference,
            RaceRoute route,
            Vector3 egoPos,
            int nowMs)
        {
            desired.Clear();
            desiredKeys.Clear();

            // Small local disk, independent of road semantics.
            int halo = (int)Math.Ceiling(NearFieldRadiusM / GridM);
            for (int dx = -halo; dx <= halo; dx++)
            {
                for (int dy = -halo; dy <= halo; dy++)
                {
                    float mx = dx * GridM;
                    float my = dy * GridM;
                    if (mx * mx + my * my > NearFieldRadiusM * NearFieldRadiusM)
                        continue;

                    Vector3 p = new Vector3(
                        egoPos.X + mx,
                        egoPos.Y + my,
                        egoPos.Z);
                    AddDesired(
                        p, egoPos.Z,
                        RaceMath.FlatDistance(egoPos, p),
                        nowMs);
                }
            }

            // Wide, sparse sampling scaffold around the executable reference.
            // The reference decides where to LOOK, never what is traversable.
            if (reference != null
                && reference.Path != null
                && reference.Path.Count >= 2)
            {
                for (int i = 0; i < reference.Path.Count; i += 3)
                {
                    Vector3 center = reference.Path[i];
                    if (RaceMath.FlatDistance(egoPos, center) > SampleAheadM + 20f)
                        continue;

                    Vector3 dir = ReferenceDirection(reference.Path, i);
                    Vector3 left = new Vector3(-dir.Y, dir.X, 0f);

                    for (float lat = -SampleHalfWidthM;
                        lat <= SampleHalfWidthM + 0.01f;
                        lat += GridM)
                    {
                        Vector3 p = new Vector3(
                            center.X + left.X * lat,
                            center.Y + left.Y * lat,
                            center.Z);
                        AddDesired(
                            p, center.Z,
                            RaceMath.FlatDistance(egoPos, p),
                            nowMs);
                    }
                }
            }
            else if (route != null && route.Built)
            {
                for (float ds = 0f; ds <= SampleAheadM; ds += 9f)
                {
                    Vector3 center = route.PointAtS(route.AlongS + ds);
                    float h = route.HeadingAtS(route.AlongS + ds);
                    Vector3 f = RaceMath.FlatNormalize(
                        RaceMath.VectorFromHeading(h));
                    Vector3 left = new Vector3(-f.Y, f.X, 0f);

                    for (float lat = -SampleHalfWidthM;
                        lat <= SampleHalfWidthM + 0.01f;
                        lat += GridM)
                    {
                        Vector3 p = new Vector3(
                            center.X + left.X * lat,
                            center.Y + left.Y * lat,
                            center.Z);
                        AddDesired(
                            p, center.Z,
                            RaceMath.FlatDistance(egoPos, p),
                            nowMs);
                    }
                }
            }

            desired.Sort((a, b) => a.Priority.CompareTo(b.Priority));
            if (desired.Count > MaxDesiredCells)
                desired.RemoveRange(MaxDesiredCells, desired.Count - MaxDesiredCells);
            if (desiredCursor >= desired.Count) desiredCursor = 0;
        }

        private void AddDesired(
            Vector3 world,
            float hintZ,
            float priority,
            int nowMs)
        {
            CellKey key = KeyFor(world.X, world.Y, hintZ);
            if (!desiredKeys.Add(key))
            {
                Cell existing;
                if (cells.TryGetValue(key, out existing))
                    existing.LastWantedMs = nowMs;
                return;
            }

            Vector3 center = CenterFor(key, hintZ);
            desired.Add(new ProbeRequest
            {
                Key = key,
                Position = center,
                HintZ = hintZ,
                Priority = priority,
            });

            Cell cell;
            if (!cells.TryGetValue(key, out cell))
            {
                cell = new Cell
                {
                    Key = key,
                    SamplePosition = center,
                    Position = center,
                    State = SurfaceState.Unknown,
                    LastWantedMs = nowMs,
                    LastSampleMs = -100000,
                    LastObstacleMs = -100000,
                };
                cells.Add(key, cell);
            }
            else
            {
                cell.SamplePosition = center;
                cell.LastWantedMs = nowMs;
            }
        }

        private void SampleGround(int nowMs)
        {
            if (desired.Count == 0) return;

            int sampled = 0;
            int scanned = 0;
            while (sampled < GroundSamplesPerBatch
                && scanned < desired.Count)
            {
                if (desiredCursor >= desired.Count) desiredCursor = 0;
                ProbeRequest req = desired[desiredCursor++];
                scanned++;

                Cell cell;
                if (!cells.TryGetValue(req.Key, out cell)) continue;
                if (nowMs - cell.LastSampleMs < CellFreshMs
                    && cell.State != SurfaceState.Unknown)
                    continue;

                float groundZ = 0f;
                bool found = false;
                try
                {
                    found = World.GetGroundHeight(
                        new Vector3(
                            req.Position.X,
                            req.Position.Y,
                            req.HintZ + 4f),
                        out groundZ);
                }
                catch { found = false; }

                cell.LastSampleMs = nowMs;
                sampled++;

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
                    // Most importantly: do not collapse an overpass onto the
                    // terrain/road several metres below it.
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
                graphDirty = true;
            }
        }

        private void ScanObstacleFan(
            Vehicle egoVehicle,
            Vector3 egoPos,
            float egoHeading,
            int nowMs)
        {
            DebugObstacles.Clear();

            Vector3 startBase = new Vector3(
                egoPos.X,
                egoPos.Y,
                egoPos.Z + ObstacleRayHeightM);

            // Alternate even/odd rays so a full fan is covered over two scans,
            // while each scan stays cheap.
            int phase = obstaclePhase++ & 1;
            for (int i = phase; i < ObstacleAnglesDeg.Length; i += 2)
            {
                float h = egoHeading + ObstacleAnglesDeg[i];
                Vector3 dir = RaceMath.FlatNormalize(
                    RaceMath.VectorFromHeading(h));
                Vector3 end = new Vector3(
                    startBase.X + dir.X * ObstacleRayRangeM,
                    startBase.Y + dir.Y * ObstacleRayRangeM,
                    startBase.Z);

                ShapeTestHandle handle = default(ShapeTestHandle);
                try
                {
                    handle = ShapeTest.StartExpensiveSyncTestLOSProbe(
                        startBase,
                        end,
                        ObstacleIntersectFlags,
                        egoVehicle != null && egoVehicle.Exists()
                            ? egoVehicle
                            : null,
                        ShapeTestOptions.Default);
                }
                catch { }

                obstacleRays++;
                if (handle.IsRequestFailed)
                {
                    obstacleFailed++;
                    continue;
                }

                ShapeTestStatus status = ShapeTestStatus.NonExistent;
                ShapeTestResult result = default(ShapeTestResult);
                try { status = handle.GetResult(out result); }
                catch { status = ShapeTestStatus.NonExistent; }

                if (status != ShapeTestStatus.Ready)
                {
                    obstacleNotReady++;
                    continue;
                }
                if (!result.DidHit) continue;

                obstacleHits++;
                DebugObstacles.Add(new DebugObstacle
                {
                    Position = result.HitPosition,
                    Normal = result.SurfaceNormal,
                });

                MarkObstacle(result.HitPosition, nowMs);
            }
        }

        private void MarkObstacle(Vector3 hit, int nowMs)
        {
            Cell best = null;
            float bestScore = float.MaxValue;

            foreach (Cell c in cells.Values)
            {
                if (c.State != SurfaceState.Traversable) continue;
                float d = RaceMath.FlatDistance(c.Position, hit);
                if (d > GridM * 1.1f) continue;
                float dz = Math.Abs(c.Position.Z - hit.Z);
                if (dz > 2.5f) continue;
                float score = d + dz * 0.35f;
                if (score >= bestScore) continue;
                bestScore = score;
                best = c;
            }

            if (best == null) return;
            best.StaticObstacle = true;
            best.LastObstacleMs = nowMs;
            graphDirty = true;
        }

        private void RebuildConnectivity(Vector3 egoPos, int nowMs)
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

            Cell egoCell = null;
            float egoBest = float.MaxValue;
            foreach (Cell c in cells.Values)
            {
                if (!c.Traversable) continue;
                float dz = Math.Abs(c.Position.Z - egoPos.Z);
                if (dz > 1.8f) continue;
                float d = RaceMath.FlatDistance(c.Position, egoPos) + dz * 2f;
                if (d >= egoBest) continue;
                egoBest = d;
                egoCell = c;
            }

            int reachableComponent = egoCell != null && egoBest <= 6f
                ? egoCell.ComponentId
                : -1;

            if (reachableComponent >= 0)
            {
                foreach (Cell c in cells.Values)
                    c.Reachable = c.ComponentId == reachableComponent;
            }

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
                for (int d = 0; d < 4; d++)
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

        private bool TryFindNeighbor(Cell c, int dir, out Cell neighbor)
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

                float dz = Math.Abs(candidate.Position.Z - c.Position.Z);
                if (dz > MaxNeighborStepM || dz >= bestDz) continue;
                bestDz = dz;
                neighbor = candidate;
            }

            return neighbor != null;
        }

        private static bool ConnectionOpen(Cell a, Cell b)
        {
            if (a == null || b == null) return false;
            if (!a.Traversable || !b.Traversable) return false;
            return Math.Abs(a.Position.Z - b.Position.Z) <= MaxNeighborStepM;
        }

        private void BuildDebugSnapshot(Vector3 egoPos, int nowMs)
        {
            DebugCells.Clear();
            DebugEdges.Clear();

            int cellsAdded = 0;
            foreach (Cell c in cells.Values)
            {
                if (RaceMath.FlatDistance(c.SamplePosition, egoPos) > 70f)
                    continue;
                DebugCells.Add(c);
                cellsAdded++;
                if (cellsAdded >= 240) break;
            }

            // Only show BLOCKED local connections. Hundreds of open-edge draw
            // calls added clutter and measurable frame cost; cell reachability
            // already shows the connected free region.
            foreach (Cell c in cells.Values)
            {
                if (DebugEdges.Count >= 80) break;
                if (c.State != SurfaceState.Traversable) continue;
                if (RaceMath.FlatDistance(c.Position, egoPos) > 42f) continue;

                for (int d = 0; d < 4 && DebugEdges.Count < 80; d += 2)
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

        private void EvictFarAndStale(Vector3 egoPos, int nowMs)
        {
            var remove = new List<CellKey>();
            foreach (KeyValuePair<CellKey, Cell> pair in cells)
            {
                Cell c = pair.Value;
                float d = RaceMath.FlatDistance(c.SamplePosition, egoPos);
                bool unwanted = nowMs - c.LastWantedMs > 6000;
                bool stale = nowMs - c.LastSampleMs > 20000;
                if ((unwanted && d > 75f) || (stale && d > 40f))
                    remove.Add(pair.Key);
            }

            for (int i = 0; i < remove.Count; i++)
                cells.Remove(remove[i]);

            if (remove.Count > 0) graphDirty = true;
        }

        private void UpdateDetail()
        {
            int traversable = 0;
            int reachable = 0;
            int reachableOffRoad = 0;
            int staticBlocked = 0;
            int noSurface = 0;
            int components = 0;
            float maxClear = 0f;

            foreach (Cell c in cells.Values)
            {
                if (c.State == SurfaceState.Traversable)
                {
                    if (c.StaticObstacle) staticBlocked++;
                    else traversable++;

                    if (c.ComponentId + 1 > components)
                        components = c.ComponentId + 1;
                }
                else if (c.State == SurfaceState.NoSurface)
                {
                    noSurface++;
                }

                if (c.Reachable)
                {
                    reachable++;
                    if (!c.RoadSemantic) reachableOffRoad++;
                    if (c.ClearanceM > maxClear) maxClear = c.ClearanceM;
                }
            }

            Detail = $"cells={cells.Count};desired={desired.Count};"
                + $"trav={traversable};reach={reachable};"
                + $"reachOffRoad={reachableOffRoad};components={components};"
                + $"staticBlocked={staticBlocked};noSurface={noSurface};"
                + $"gOk={groundOk};gMiss={groundMiss};gLayer={groundLayerReject};"
                + $"obsRays={obstacleRays};obsHits={obstacleHits};"
                + $"obsWait={obstacleNotReady};obsFail={obstacleFailed};"
                + $"maxClear={maxClear:F1};"
                + $"tickMs={lastTickMs:F2};peakMs={peakTickMs:F2}";
        }

        private static CellKey KeyFor(float x, float y, float hintZ)
        {
            return new CellKey
            {
                X = (int)Math.Round(x / GridM),
                Y = (int)Math.Round(y / GridM),
                Layer = (int)Math.Round(hintZ / LayerBucketM),
            };
        }

        private static Vector3 CenterFor(CellKey key, float hintZ)
        {
            return new Vector3(
                key.X * GridM,
                key.Y * GridM,
                hintZ);
        }

        private static Vector3 ReferenceDirection(
            IList<Vector3> path,
            int i)
        {
            Vector3 d;
            if (i <= 0)
                d = new Vector3(
                    path[1].X - path[0].X,
                    path[1].Y - path[0].Y,
                    0f);
            else if (i >= path.Count - 1)
                d = new Vector3(
                    path[i].X - path[i - 1].X,
                    path[i].Y - path[i - 1].Y,
                    0f);
            else
                d = new Vector3(
                    path[i + 1].X - path[i - 1].X,
                    path[i + 1].Y - path[i - 1].Y,
                    0f);

            return RaceMath.FlatNormalize(d);
        }
    }
}
