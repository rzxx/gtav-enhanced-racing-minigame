using System;
using System.Collections.Generic;
using GTA;
using GTA.Math;
using GTA.Native;

namespace StreetRacing
{
    /// Debug-only physical traversability observer.
    ///
    /// This is intentionally NOT consumed by SpatialPlannerV1 yet. The purpose
    /// of the first pass is to establish whether we can reconstruct useful local
    /// drivable space from the physical world before replacing LocalWorldModel.
    ///
    /// Geometry comes from collision shape tests:
    ///   - downward LOS probes discover a local surface, Z and normal
    ///   - capsule probes between neighboring cells discover walls/barriers
    ///   - a graph over confirmed-open edges gives physical connectivity
    ///   - flood fill from the ego surface gives the currently reachable patch
    ///
    /// GTA road classification is stored only as semantic annotation. A cell can
    /// be physically reachable even when IS_POINT_ON_ROAD says false.
    internal sealed class PhysicalSurfaceMap
    {
        internal enum SurfaceState
        {
            Unknown,
            Pending,
            NoSurface,
            TooSteep,
            Traversable,
        }

        internal sealed class Cell
        {
            public CellKey Key;
            public Vector3 SamplePosition;
            public Vector3 Position;
            public Vector3 Normal;
            public SurfaceState State;
            public bool RoadSemantic;
            public bool Reachable;
            public int ComponentId = -1;
            public float ClearanceM;
            public int LastWantedMs;
            public int LastSampleMs;
            public bool GroundPending;

            public bool HasSurface =>
                State == SurfaceState.Traversable || State == SurfaceState.TooSteep;
            public bool Traversable => State == SurfaceState.Traversable;
        }

        internal struct DebugEdge
        {
            public Vector3 A;
            public Vector3 B;
            public bool Open;
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
            public bool Pending;
            public int LastSampleMs;
        }

        private struct ProbeRequest
        {
            public CellKey Key;
            public Vector3 Position;
            public float HintZ;
            public float Priority;
        }

        private struct PendingGround
        {
            public ShapeTestHandle Handle;
            public CellKey Key;
            public int StartedMs;
        }

        private struct PendingEdge
        {
            public ShapeTestHandle Handle;
            public EdgeKey Key;
            public int StartedMs;
        }

        public readonly List<Cell> DebugCells = new List<Cell>(700);
        public readonly List<DebugEdge> DebugEdges = new List<DebugEdge>(900);
        public string Detail { get; private set; } = "";

        private readonly Dictionary<CellKey, Cell> cells =
            new Dictionary<CellKey, Cell>();
        private readonly Dictionary<EdgeKey, EdgeInfo> edges =
            new Dictionary<EdgeKey, EdgeInfo>();
        private readonly List<ProbeRequest> desired =
            new List<ProbeRequest>(900);
        private readonly HashSet<CellKey> desiredKeys =
            new HashSet<CellKey>();
        private readonly List<PendingGround> pendingGround =
            new List<PendingGround>(64);
        private readonly List<PendingEdge> pendingEdge =
            new List<PendingEdge>(64);
        private readonly Queue<Cell> flood = new Queue<Cell>(900);
        private readonly Queue<Cell> distanceFlood = new Queue<Cell>(900);

        private int desiredCursor;
        private int lastDesiredRefreshMs = -100000;
        private int lastGraphMs = -100000;
        private int lastEvictMs = -100000;
        private bool graphDirty = true;

        private const float GridM = 2.5f;
        private const float LayerBucketM = 4.0f;
        private const float GroundProbeUpM = 2.5f;
        private const float GroundProbeDownM = 5.5f;
        private const float TraversableNormalZ = 0.68f;
        private const float MaxStepM = 0.65f;
        private const float EdgeCapsuleHeightM = 0.85f;
        private const float EdgeCapsuleRadiusM = 0.40f;
        private const int GroundStartsPerTick = 10;
        private const int EdgeStartsPerTick = 8;
        private const int MaxPendingGround = 48;
        private const int MaxPendingEdges = 48;
        private const int ProbeTimeoutMs = 900;
        private const int CellFreshMs = 12000;
        private const int EdgeFreshMs = 15000;

        // Ground sampling should discover the actual terrain/road surface,
        // not the tops of props. Obstacles use map + props + foliage; vehicles
        // and peds remain in the dynamic Perception layer.
        private static readonly IntersectFlags GroundIntersectFlags =
            IntersectFlags.Map;
        private static readonly IntersectFlags ObstacleIntersectFlags =
            IntersectFlags.Map | IntersectFlags.Objects | IntersectFlags.Foliage;

        private int groundStartFailed;
        private int groundReady;
        private int groundNonExistent;
        private int edgeStartFailed;
        private int edgeReady;
        private int edgeNonExistent;

        private static readonly int[] DirX = { 1, -1, 0, 0 };
        private static readonly int[] DirY = { 0, 0, 1, -1 };

        public void Reset()
        {
            cells.Clear();
            edges.Clear();
            desired.Clear();
            desiredKeys.Clear();
            pendingGround.Clear();
            pendingEdge.Clear();
            DebugCells.Clear();
            DebugEdges.Clear();
            desiredCursor = 0;
            lastDesiredRefreshMs = -100000;
            lastGraphMs = -100000;
            lastEvictMs = -100000;
            graphDirty = true;
            groundStartFailed = 0;
            groundReady = 0;
            groundNonExistent = 0;
            edgeStartFailed = 0;
            edgeReady = 0;
            edgeNonExistent = 0;
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
            PollGroundResults(nowMs);
            PollEdgeResults(nowMs);

            if (nowMs - lastDesiredRefreshMs >= 250)
            {
                lastDesiredRefreshMs = nowMs;
                RebuildDesired(reference, route, egoPos, egoHeading, nowMs);
            }

            StartGroundProbes(egoVehicle, nowMs);
            StartEdgeProbes(egoVehicle, egoPos, nowMs);

            if (graphDirty || nowMs - lastGraphMs >= 350)
            {
                lastGraphMs = nowMs;
                RebuildConnectivity(egoPos);
                BuildDebugSnapshot(egoPos);
                graphDirty = false;
            }

            if (nowMs - lastEvictMs >= 1500)
            {
                lastEvictMs = nowMs;
                EvictFarAndStale(egoPos, nowMs);
            }

            UpdateDetail();
        }

        private void RebuildDesired(
            DrivingReference.Result reference,
            RaceRoute route,
            Vector3 egoPos,
            float egoHeading,
            int nowMs)
        {
            desired.Clear();
            desiredKeys.Clear();

            // Independent near-field disk. This is deliberately NOT road/node
            // derived: it discovers whatever physical surface actually exists
            // around the vehicle.
            int halo = (int)Math.Ceiling(25f / GridM);
            for (int dx = -halo; dx <= halo; dx++)
            {
                for (int dy = -halo; dy <= halo; dy++)
                {
                    float mx = dx * GridM;
                    float my = dy * GridM;
                    if (mx * mx + my * my > 25f * 25f) continue;
                    AddDesired(
                        new Vector3(egoPos.X + mx, egoPos.Y + my, egoPos.Z),
                        egoPos.Z,
                        RaceMath.FlatDistance(
                            egoPos,
                            new Vector3(egoPos.X + mx, egoPos.Y + my, egoPos.Z)),
                        nowMs);
                }
            }

            // Wide strips around the executable reference tell us about the
            // physical width of the road, junction apron and nearby branches.
            // The reference is ONLY a sampling scaffold; it cannot mark a cell
            // traversable.
            if (reference != null
                && reference.Path != null
                && reference.Path.Count >= 2)
            {
                for (int i = 0; i < reference.Path.Count; i += 2)
                {
                    Vector3 center = reference.Path[i];
                    Vector3 dir = ReferenceDirection(reference.Path, i);
                    Vector3 left = new Vector3(-dir.Y, dir.X, 0f);

                    for (float lat = -20f; lat <= 20.01f; lat += GridM)
                    {
                        Vector3 p = new Vector3(
                            center.X + left.X * lat,
                            center.Y + left.Y * lat,
                            center.Z);
                        float priority = RaceMath.FlatDistance(egoPos, p);
                        AddDesired(p, center.Z, priority, nowMs);
                    }
                }
            }
            else if (route != null && route.Built)
            {
                // Reference can temporarily disappear near terminal route
                // geometry. Keep observing a wide strip along route samples.
                for (float s = 0f; s <= 65f; s += 7.5f)
                {
                    Vector3 center = route.PointAtS(route.AlongS + s);
                    float h = route.HeadingAtS(route.AlongS + s);
                    Vector3 f = RaceMath.FlatNormalize(RaceMath.VectorFromHeading(h));
                    Vector3 left = new Vector3(-f.Y, f.X, 0f);
                    for (float lat = -18f; lat <= 18.01f; lat += GridM)
                    {
                        Vector3 p = new Vector3(
                            center.X + left.X * lat,
                            center.Y + left.Y * lat,
                            center.Z);
                        AddDesired(p, center.Z, RaceMath.FlatDistance(egoPos, p), nowMs);
                    }
                }
            }

            desired.Sort((a, b) => a.Priority.CompareTo(b.Priority));
            if (desiredCursor >= desired.Count) desiredCursor = 0;
        }

        private void AddDesired(Vector3 world, float hintZ, float priority, int nowMs)
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
                    Normal = new Vector3(0f, 0f, 1f),
                    State = SurfaceState.Unknown,
                    LastWantedMs = nowMs,
                    LastSampleMs = -100000,
                };
                cells.Add(key, cell);
            }
            else
            {
                cell.SamplePosition = center;
                cell.LastWantedMs = nowMs;
            }
        }

        private void StartGroundProbes(Vehicle egoVehicle, int nowMs)
        {
            if (desired.Count == 0 || pendingGround.Count >= MaxPendingGround) return;

            int started = 0;
            int scanned = 0;
            while (started < GroundStartsPerTick
                && scanned < desired.Count
                && pendingGround.Count < MaxPendingGround)
            {
                if (desiredCursor >= desired.Count) desiredCursor = 0;
                ProbeRequest req = desired[desiredCursor++];
                scanned++;

                Cell cell;
                if (!cells.TryGetValue(req.Key, out cell)) continue;
                if (cell.GroundPending) continue;
                if (nowMs - cell.LastSampleMs < CellFreshMs
                    && cell.State != SurfaceState.Unknown)
                    continue;

                Vector3 start = new Vector3(
                    req.Position.X, req.Position.Y, req.HintZ + GroundProbeUpM);
                Vector3 end = new Vector3(
                    req.Position.X, req.Position.Y, req.HintZ - GroundProbeDownM);

                ShapeTestHandle handle = default(ShapeTestHandle);
                try
                {
                    handle = ShapeTest.StartTestLOSProbe(
                        start, end,
                        GroundIntersectFlags,
                        egoVehicle != null && egoVehicle.Exists() ? egoVehicle : null,
                        ShapeTestOptions.Default);
                }
                catch { }

                if (handle.IsRequestFailed)
                {
                    groundStartFailed++;
                    cell.LastSampleMs = nowMs;
                    continue;
                }

                cell.GroundPending = true;
                cell.State = SurfaceState.Pending;
                pendingGround.Add(new PendingGround
                {
                    Handle = handle,
                    Key = req.Key,
                    StartedMs = nowMs,
                });
                started++;
            }
        }

        private void PollGroundResults(int nowMs)
        {
            for (int i = pendingGround.Count - 1; i >= 0; i--)
            {
                PendingGround p = pendingGround[i];
                Cell cell;
                if (!cells.TryGetValue(p.Key, out cell))
                {
                    pendingGround.RemoveAt(i);
                    continue;
                }

                if (nowMs - p.StartedMs > ProbeTimeoutMs)
                {
                    cell.GroundPending = false;
                    cell.State = SurfaceState.Unknown;
                    pendingGround.RemoveAt(i);
                    continue;
                }

                ShapeTestStatus status = ShapeTestStatus.NonExistent;
                ShapeTestResult shapeResult = default(ShapeTestResult);
                try { status = p.Handle.GetResult(out shapeResult); }
                catch { status = ShapeTestStatus.NonExistent; }

                if (status == ShapeTestStatus.NotReady) continue;

                cell.GroundPending = false;
                cell.LastSampleMs = nowMs;
                pendingGround.RemoveAt(i);

                if (status != ShapeTestStatus.Ready)
                {
                    groundNonExistent++;
                    cell.State = SurfaceState.Unknown;
                    continue;
                }

                groundReady++;
                if (!shapeResult.DidHit)
                {
                    cell.State = SurfaceState.NoSurface;
                    cell.RoadSemantic = false;
                    cell.Reachable = false;
                    cell.ComponentId = -1;
                    cell.ClearanceM = 0f;
                    graphDirty = true;
                    continue;
                }

                Vector3 hitPos = shapeResult.HitPosition;
                Vector3 normal = shapeResult.SurfaceNormal;
                cell.Position = hitPos;
                cell.Normal = normal;
                cell.State = normal.Z >= TraversableNormalZ
                    ? SurfaceState.Traversable
                    : SurfaceState.TooSteep;

                bool road = false;
                try
                {
                    road = Function.Call<bool>(
                        Hash.IS_POINT_ON_ROAD,
                        hitPos.X, hitPos.Y, hitPos.Z + 0.15f, 0);
                }
                catch { road = false; }
                cell.RoadSemantic = road;
                graphDirty = true;
            }
        }

        private void StartEdgeProbes(Vehicle egoVehicle, Vector3 egoPos, int nowMs)
        {
            if (pendingEdge.Count >= MaxPendingEdges) return;

            int started = 0;
            foreach (Cell a in cells.Values)
            {
                if (started >= EdgeStartsPerTick || pendingEdge.Count >= MaxPendingEdges)
                    break;
                if (!a.Traversable) continue;
                if (RaceMath.FlatDistance(a.Position, egoPos) > 70f) continue;

                for (int d = 0; d < 4; d++)
                {
                    if (started >= EdgeStartsPerTick || pendingEdge.Count >= MaxPendingEdges)
                        break;

                    Cell b;
                    if (!TryFindNeighbor(a, d, out b)) continue;

                    EdgeKey key = MakeEdgeKey(a.Key, b.Key);
                    EdgeInfo edge;
                    if (!edges.TryGetValue(key, out edge))
                    {
                        edge = new EdgeInfo();
                        edges.Add(key, edge);
                    }

                    if (edge.Pending) continue;
                    if (edge.Known && nowMs - edge.LastSampleMs < EdgeFreshMs)
                        continue;

                    float dz = Math.Abs(a.Position.Z - b.Position.Z);
                    if (dz > MaxStepM)
                    {
                        edge.Known = true;
                        edge.Open = false;
                        edge.Pending = false;
                        edge.LastSampleMs = nowMs;
                        graphDirty = true;
                        continue;
                    }

                    Vector3 start = new Vector3(
                        a.Position.X, a.Position.Y,
                        a.Position.Z + EdgeCapsuleHeightM);
                    Vector3 end = new Vector3(
                        b.Position.X, b.Position.Y,
                        b.Position.Z + EdgeCapsuleHeightM);

                    ShapeTestHandle handle = default(ShapeTestHandle);
                    try
                    {
                        handle = ShapeTest.StartTestCapsule(
                            start, end, EdgeCapsuleRadiusM,
                            ObstacleIntersectFlags,
                            egoVehicle != null && egoVehicle.Exists() ? egoVehicle : null,
                            ShapeTestOptions.Default);
                    }
                    catch { }

                    if (handle.IsRequestFailed)
                    {
                        edgeStartFailed++;
                        continue;
                    }

                    edge.Pending = true;
                    pendingEdge.Add(new PendingEdge
                    {
                        Handle = handle,
                        Key = key,
                        StartedMs = nowMs,
                    });
                    started++;
                }
            }
        }

        private void PollEdgeResults(int nowMs)
        {
            for (int i = pendingEdge.Count - 1; i >= 0; i--)
            {
                PendingEdge p = pendingEdge[i];
                EdgeInfo edge;
                if (!edges.TryGetValue(p.Key, out edge))
                {
                    pendingEdge.RemoveAt(i);
                    continue;
                }

                if (nowMs - p.StartedMs > ProbeTimeoutMs)
                {
                    edge.Pending = false;
                    edge.Known = false;
                    pendingEdge.RemoveAt(i);
                    continue;
                }

                ShapeTestStatus status = ShapeTestStatus.NonExistent;
                ShapeTestResult shapeResult = default(ShapeTestResult);
                try { status = p.Handle.GetResult(out shapeResult); }
                catch { status = ShapeTestStatus.NonExistent; }

                if (status == ShapeTestStatus.NotReady) continue;

                edge.Pending = false;
                pendingEdge.RemoveAt(i);

                if (status != ShapeTestStatus.Ready)
                {
                    edgeNonExistent++;
                    edge.Known = false;
                    continue;
                }

                edgeReady++;
                edge.Known = true;
                edge.Open = !shapeResult.DidHit;
                edge.LastSampleMs = nowMs;
                graphDirty = true;
            }
        }

        private void RebuildConnectivity(Vector3 egoPos)
        {
            foreach (Cell c in cells.Values)
            {
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
                    if (!IsOpen(c, n)) continue;
                    n.ComponentId = componentId;
                    flood.Enqueue(n);
                }
            }
        }

        private void ComputeClearance()
        {
            distanceFlood.Clear();
            float inf = 9999f;

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
                        || !IsOpen(c, n))
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
                    if (!n.Reachable || !IsOpen(c, n)) continue;

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

            // Layer bucket is an identity aid, not connectivity. Search nearby
            // buckets and choose the physically closest surface in Z.
            for (int dl = -2; dl <= 2; dl++)
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
                if (dz > MaxStepM || dz >= bestDz) continue;
                bestDz = dz;
                neighbor = candidate;
            }

            return neighbor != null;
        }

        private bool IsOpen(Cell a, Cell b)
        {
            EdgeInfo e;
            return edges.TryGetValue(MakeEdgeKey(a.Key, b.Key), out e)
                && e.Known
                && e.Open;
        }

        private void BuildDebugSnapshot(Vector3 egoPos)
        {
            DebugCells.Clear();
            DebugEdges.Clear();

            foreach (Cell c in cells.Values)
            {
                if (RaceMath.FlatDistance(c.SamplePosition, egoPos) > 82f) continue;
                DebugCells.Add(c);
            }

            foreach (KeyValuePair<EdgeKey, EdgeInfo> pair in edges)
            {
                EdgeInfo e = pair.Value;
                if (!e.Known) continue;

                Cell a;
                Cell b;
                if (!cells.TryGetValue(pair.Key.A, out a)
                    || !cells.TryGetValue(pair.Key.B, out b))
                    continue;
                if (RaceMath.FlatDistance(a.Position, egoPos) > 70f
                    && RaceMath.FlatDistance(b.Position, egoPos) > 70f)
                    continue;

                DebugEdges.Add(new DebugEdge
                {
                    A = a.Position,
                    B = b.Position,
                    Open = e.Open,
                });
            }
        }

        private void EvictFarAndStale(Vector3 egoPos, int nowMs)
        {
            var removeCells = new List<CellKey>();
            foreach (KeyValuePair<CellKey, Cell> pair in cells)
            {
                Cell c = pair.Value;
                float d = RaceMath.FlatDistance(c.SamplePosition, egoPos);
                bool unwanted = nowMs - c.LastWantedMs > 7000;
                bool stale = nowMs - c.LastSampleMs > 30000;
                if ((unwanted && d > 90f) || (stale && d > 45f))
                    removeCells.Add(pair.Key);
            }

            if (removeCells.Count == 0) return;

            for (int i = 0; i < removeCells.Count; i++)
                cells.Remove(removeCells[i]);

            var removeEdges = new List<EdgeKey>();
            foreach (EdgeKey key in edges.Keys)
            {
                if (!cells.ContainsKey(key.A) || !cells.ContainsKey(key.B))
                    removeEdges.Add(key);
            }
            for (int i = 0; i < removeEdges.Count; i++)
                edges.Remove(removeEdges[i]);

            graphDirty = true;
        }

        private void UpdateDetail()
        {
            int traversable = 0;
            int reachable = 0;
            int reachableOffRoad = 0;
            int tooSteep = 0;
            int noSurface = 0;
            int components = 0;
            float maxClear = 0f;

            foreach (Cell c in cells.Values)
            {
                if (c.Traversable)
                {
                    traversable++;
                    if (c.ComponentId + 1 > components)
                        components = c.ComponentId + 1;
                }
                else if (c.State == SurfaceState.TooSteep) tooSteep++;
                else if (c.State == SurfaceState.NoSurface) noSurface++;

                if (c.Reachable)
                {
                    reachable++;
                    if (!c.RoadSemantic) reachableOffRoad++;
                    if (c.ClearanceM > maxClear) maxClear = c.ClearanceM;
                }
            }

            int openEdges = 0;
            int blockedEdges = 0;
            foreach (EdgeInfo e in edges.Values)
            {
                if (!e.Known) continue;
                if (e.Open) openEdges++;
                else blockedEdges++;
            }

            Detail = $"cells={cells.Count};trav={traversable};reach={reachable};"
                + $"reachOffRoad={reachableOffRoad};components={components};"
                + $"openEdges={openEdges};blockedEdges={blockedEdges};"
                + $"noSurface={noSurface};steep={tooSteep};"
                + $"pendingG={pendingGround.Count};pendingE={pendingEdge.Count};"
                + $"gReady={groundReady};gGone={groundNonExistent};gFail={groundStartFailed};"
                + $"eReady={edgeReady};eGone={edgeNonExistent};eFail={edgeStartFailed};"
                + $"maxClear={maxClear:F1}";
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
            return new Vector3(key.X * GridM, key.Y * GridM, hintZ);
        }

        private static EdgeKey MakeEdgeKey(CellKey a, CellKey b)
        {
            if (CompareKey(a, b) <= 0)
                return new EdgeKey { A = a, B = b };
            return new EdgeKey { A = b, B = a };
        }

        private static int CompareKey(CellKey a, CellKey b)
        {
            if (a.X != b.X) return a.X.CompareTo(b.X);
            if (a.Y != b.Y) return a.Y.CompareTo(b.Y);
            return a.Layer.CompareTo(b.Layer);
        }

        private static Vector3 ReferenceDirection(IList<Vector3> path, int i)
        {
            Vector3 d;
            if (i <= 0)
                d = new Vector3(
                    path[1].X - path[0].X,
                    path[1].Y - path[0].Y, 0f);
            else if (i >= path.Count - 1)
                d = new Vector3(
                    path[i].X - path[i - 1].X,
                    path[i].Y - path[i - 1].Y, 0f);
            else
                d = new Vector3(
                    path[i + 1].X - path[i - 1].X,
                    path[i + 1].Y - path[i - 1].Y, 0f);

            return RaceMath.FlatNormalize(d);
        }
    }
}
