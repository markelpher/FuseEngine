using System;
using System.Collections.Generic;
using System.Numerics;
using Fuse.Debug;
using Fuse.Physics;
using Fuse.Scene;
using JoltPhysicsSharp;

namespace Fuse.Enemy;

/// <summary>
/// Builds stable, surface-relative contacts for the spider body and feet.
/// A contact is deliberately more than a raycast hit: it keeps the supporting
/// body/local anchor, a filtered normal and a confidence value.
/// </summary>
public sealed class SpiderSurfaceSolver : IGizmoDrawable, IDisposable
{
    private const float Epsilon = 0.0001f;
    private const float BodyProbeRadius = 0.48f;
    private const float FootProbeHeight = 1.35f;
    private const float FootProbeDistance = 4.5f;
    private const float SameSurfaceObstacleProbeMinimum = 1.75f;
    private const float SameSurfaceObstacleProbeMaximum = 3.50f;
    private const float SameSurfaceDetourMargin = 0.30f;
    private const float SameSurfaceDetourMaximumOffset = 12.0f;
    private const int SameSurfaceSearchShapeBudget = 96;
    private const int SameSurfaceSearchRayBudget = 24;

    private readonly SceneManager _scene;
    private readonly SpiderLocomotionProfile _profile;
    private readonly List<ShapeCastResult> _castHits = new(16);
    private readonly List<CollideShapeResult> _overlapHits = new(16);
    private readonly List<DebugProbe> _debugProbes = new();
    private readonly List<SpiderSurfaceContact> _debugCandidates = new();
    private readonly HashSet<BodyID> _ignoredBodies = new();
    private SpiderSurfaceContact _lastBodyContact;
    private readonly DefaultBroadPhaseLayerFilter _queryBroadPhase = new();
    private readonly DefaultObjectLayerFilter _queryObjectLayer = new();
    private readonly DefaultShapeFilter _queryShapeFilter = new();
    private SurfaceBodyFilter? _queryBodyFilter;
    private BodyID _querySelf = BodyID.Invalid;
    private readonly Dictionary<(int Length, int Radius), Shape> _segmentShapes = new();
    private readonly Queue<(int Length, int Radius)> _shapeOrder = new();
    private int _shapeQueryLimit;
    private int _raycastLimit;
    private bool _disposed;

    public int FrameShapeQueries { get; private set; }
    public int FrameRaycasts { get; private set; }
    public bool QueryBudgetExhausted { get; private set; }
    public bool HasShapeQueryBudget => !_disposed && FrameShapeQueries < _shapeQueryLimit;
    public bool HasRaycastBudget => !_disposed && FrameRaycasts < _raycastLimit;

    public SpiderSurfaceContact LastBodyContact => _lastBodyContact;
    public BodyFilter CreateBodyFilter(BodyID self) =>
        new SurfaceBodyFilter(self, _ignoredBodies, _scene.NonWalkableBodies);
    public SpiderLocomotionProfile Profile => _profile;

    public SpiderSurfaceSolver(SceneManager scene, SpiderLocomotionProfile? profile = null)
    {
        _scene = scene;
        _profile = profile ?? SpiderLocomotionProfile.Default;
        BeginFrame();
        Debug.DebugDrawer.Register(this);
    }

    /// <summary>
    /// Registra corpos auxiliares da própria aranha que não podem ser
    /// interpretados como superfícies de suporte. As hitboxes articuladas de
    /// dano são corpos físicos separados da cápsula de locomoção.
    /// </summary>
    public void SetIgnoredBodies(IEnumerable<BodyID>? bodyIds)
    {
        _ignoredBodies.Clear();

        if (bodyIds == null)
            return;

        foreach (BodyID bodyId in bodyIds)
        {
            if (bodyId.IsValid)
                _ignoredBodies.Add(bodyId);
        }
    }

    /// <summary>Clears transient probe debug once per animation frame.</summary>
    public void BeginFrame()
    {
        _debugProbes.Clear();
        _debugCandidates.Clear();
        FrameShapeQueries = FrameRaycasts = 0;
        QueryBudgetExhausted = false;
        // Reserve most of the update for the gait that can free a blocked body.
        _shapeQueryLimit = System.Math.Max(1, _profile.MaximumShapeQueriesPerUpdate / 3);
        _raycastLimit = System.Math.Max(1, _profile.MaximumRaycastsPerUpdate / 3);
    }

    public void BeginMotor()
    {
        // Pursuit may exhaust its probe allowance; it must not steal the
        // motor's essential support acquisition/post-move confirmation rays.
        _raycastLimit = System.Math.Max(1, _profile.MaximumRaycastsPerUpdate * 2 / 3);
    }

    public void BeginGait()
    {
        _shapeQueryLimit = System.Math.Max(1, _profile.MaximumShapeQueriesPerUpdate);
        _raycastLimit = System.Math.Max(1, _profile.MaximumRaycastsPerUpdate);
    }

    private bool TakeShapeQuery()
    {
        if (_disposed || FrameShapeQueries >= _shapeQueryLimit)
        {
            QueryBudgetExhausted = true;
            return false;
        }
        FrameShapeQueries++;
        return true;
    }

    private bool TakeRaycast()
    {
        if (_disposed || FrameRaycasts >= _raycastLimit)
        {
            QueryBudgetExhausted = true;
            return false;
        }
        FrameRaycasts++;
        return true;
    }

    private SurfaceBodyFilter QueryBodyFilter(BodyID self)
    {
        if (_queryBodyFilter == null || _querySelf != self)
        {
            _queryBodyFilter?.Dispose();
            _querySelf = self;
            _queryBodyFilter = new SurfaceBodyFilter(self, _ignoredBodies, _scene.NonWalkableBodies);
        }
        return _queryBodyFilter;
    }

    public SpiderSurfaceContact Refresh(in SpiderSurfaceContact contact)
    {
        if (_disposed || !contact.IsValid || !contact.BodyId.IsValid ||
            (contact.SupportBody != null && (!contact.SupportBody.IsBuilt || contact.SupportBody.Native != contact.BodyId)))
            return default;
        _scene.Physics.BodyLockInterface.LockRead(contact.BodyId, out BodyLockRead readLock);
        if (!readLock.Succeeded) return default;
        try
        {
            if (readLock.Body is not { } body || body.IsSensor) return default;
            return contact.WithWorldPose(
                body.Position + Vector3.Transform(contact.LocalPoint, body.Rotation),
                NormalizeOrFallback(Vector3.Transform(contact.LocalNormal, body.Rotation), contact.Normal));
        }
        finally { _scene.Physics.BodyLockInterface.UnlockRead(readLock); }
    }

    /// <summary>
    /// Samples only the currently expected support plane. Unlike the legacy
    /// six-axis fallback this query is expressed entirely in the spider's local
    /// surface frame, so +X/-X and +Z/-Z are genuinely equivalent.
    /// </summary>
    public bool TryFindSupportContact(
        Vector3 bodyCenter,
        Vector3 expectedNormal,
        Vector3 preferredForward,
        float clearance,
        BodyID selfBody,
        out SpiderSurfaceContact contact)
    {
        expectedNormal = NormalizeOrZero(expectedNormal);
        if (expectedNormal.LengthSquared() <= Epsilon * Epsilon ||
            !BuildTangentBasis(expectedNormal, preferredForward, out Vector3 forward, out Vector3 right))
        {
            contact = default;
            return false;
        }

        float patchRadius = MathF.Min(BodyProbeRadius, MathF.Max(0.18f, clearance * 0.42f));
        Vector3[] offsets =
        {
            Vector3.Zero,
            forward * patchRadius,
            -forward * patchRadius,
            right * patchRadius,
            -right * patchRadius
        };

        SpiderSurfaceContact best = default;
        float bestScore = float.MaxValue;
        foreach (Vector3 offset in offsets)
        {
            Vector3 origin = bodyCenter + expectedNormal * 0.18f + offset;
            if (!TryProbeContact(
                    origin,
                    -expectedNormal,
                    clearance + 0.85f,
                    selfBody,
                    out SpiderSurfaceContact candidate,
                    out float distance))
            {
                continue;
            }

            float alignment = Vector3.Dot(candidate.Normal, expectedNormal);
            if (alignment < 0.62f)
                continue;

            float score = distance + (1f - alignment) * 2f + offset.LengthSquared() * 0.08f;
            if (score < bestScore)
            {
                best = candidate;
                bestScore = score;
            }
        }

        contact = best;
        if (!contact.IsValid)
            return false;

        _lastBodyContact = contact;
        return true;
    }

    /// <summary>
    /// Searches for an adjacent face in the intended movement direction.
    /// A forward probe handles concave corners; a quarter-circle probe fan
    /// handles convex edges where the next wall is below and behind the body.
    /// </summary>
    public bool TryFindTransitionContact(
        Vector3 bodyCenter,
        Vector3 currentNormal,
        Vector3 movementDirection,
        float clearance,
        float lookAhead,
        BodyID selfBody,
        out SpiderSurfaceContact contact)
    {
        currentNormal = NormalizeOrZero(currentNormal);
        if (currentNormal.LengthSquared() <= Epsilon * Epsilon)
        {
            contact = default;
            return false;
        }

        movementDirection -= currentNormal * Vector3.Dot(movementDirection, currentNormal);
        if (movementDirection.LengthSquared() <= Epsilon * Epsilon)
        {
            contact = default;
            return false;
        }
        movementDirection = Vector3.Normalize(movementDirection);

        // Concave transition: the spider is walking directly into the next face.
        Vector3 forwardOrigin = bodyCenter + currentNormal * 0.10f;
        if (TryProbeContact(
                forwardOrigin,
                movementDirection,
                MathF.Max(clearance + MathF.Max(0.12f, lookAhead), _profile.SurfaceTransitionProbeWorld),
                selfBody,
                out SpiderSurfaceContact forwardContact,
                out _) &&
            IsUsableTransition(forwardContact.Normal, currentNormal) &&
            Vector3.Dot(forwardContact.Normal, movementDirection) < -0.28f)
        {
            contact = forwardContact;
            _lastBodyContact = contact;
            return true;
        }

        // Convex transition: rotate the support ray from -up toward -forward.
        // This detects the outer face before the old support disappears fully.
        Vector3 fanOrigin = bodyCenter + currentNormal * 0.10f +
                            movementDirection * MathF.Max(0.08f, lookAhead * 0.45f);
        SpiderSurfaceContact best = default;
        float bestScore = float.MaxValue;
        for (int i = 0; i < 6; i++)
        {
            float angle = (20f + i * 13f) * (MathF.PI / 180f);
            Vector3 direction = NormalizeOrFallback(
                -currentNormal * MathF.Cos(angle) - movementDirection * MathF.Sin(angle),
                -currentNormal);

            if (!TryProbeContact(
                    fanOrigin,
                    direction,
                    clearance * 2.65f + MathF.Max(0.15f, lookAhead),
                    selfBody,
                    out SpiderSurfaceContact candidate,
                    out float distance))
            {
                continue;
            }

            if (!IsUsableTransition(candidate.Normal, currentNormal))
                continue;

            float forwardAlignment = Vector3.Dot(candidate.Normal, movementDirection);
            if (forwardAlignment < 0.20f)
                continue;

            float score = distance + (1f - forwardAlignment) * clearance;
            if (score < bestScore)
            {
                best = candidate;
                bestScore = score;
            }
        }

        contact = best;
        if (!contact.IsValid)
            return false;

        _lastBodyContact = contact;
        return true;
    }

    /// <summary>
    /// Finds a short lateral waypoint when an obstacle blocks a spider that is
    /// otherwise pursuing a target on the same support surface. This is a
    /// bounded local query, not a navigation graph: it tries a small set of
    /// tangent-side offsets and exits up to 12 m away, checks feet/body space,
    /// and samples support. Only complete routes with a clear exit are accepted.
    /// </summary>
    public bool TryFindSameSurfaceDetour(
        Vector3 bodyCenter,
        Vector3 currentNormal,
        Vector3 desiredDirection,
        Vector3 targetPosition,
        float clearance,
        BodyID selfBody,
        Vector3 preferredSide,
        out Vector3 waypoint,
        out float score,
        out Vector3 selectedSide)
    {
        return TryFindSameSurfaceDetour(bodyCenter, currentNormal, desiredDirection,
            targetPosition, clearance, selfBody, preferredSide, out waypoint,
            out score, out selectedSide, out _);
    }

    // The second waypoint keeps the route outside the obstacle until a checked
    // reconnect is available, rather than cutting back across its near corner.
    internal bool TryFindSameSurfaceDetour(
        Vector3 bodyCenter,
        Vector3 currentNormal,
        Vector3 desiredDirection,
        Vector3 targetPosition,
        float clearance,
        BodyID selfBody,
        Vector3 preferredSide,
        out Vector3 waypoint,
        out float score,
        out Vector3 selectedSide,
        out Vector3 exitWaypoint)
    {
        waypoint = exitWaypoint = selectedSide = Vector3.Zero;
        score = float.MaxValue;

        currentNormal = NormalizeOrZero(currentNormal);
        desiredDirection = NormalizeOrZero(
            ProjectOnPlane(desiredDirection, currentNormal));
        Vector3 forward = Vector3.Zero;
        Vector3 right = Vector3.Zero;
        if (currentNormal.LengthSquared() <= Epsilon * Epsilon ||
            desiredDirection.LengthSquared() <= Epsilon * Epsilon ||
            !IsFinite(bodyCenter) ||
            !IsFinite(targetPosition) ||
            !BuildTangentBasis(
                currentNormal,
                desiredDirection,
                out forward,
                out right))
        {
            return false;
        }

        if (!float.IsFinite(clearance) || clearance <= 0f)
            return false;

        float probeDistance = System.Math.Clamp(clearance * 4f,
            SameSurfaceObstacleProbeMinimum, SameSurfaceObstacleProbeMaximum);
        Vector3 targetDelta = ProjectOnPlane(targetPosition - bodyCenter, currentNormal);
        float targetDistance = targetDelta.Length();
        if (!float.IsFinite(targetDistance) || targetDistance <= Epsilon || IsSameSurfacePathClear(bodyCenter,
                targetPosition, currentNormal, clearance, selfBody, probeDistance))
            return false;

        int shapeLimit = System.Math.Min(_shapeQueryLimit,
            FrameShapeQueries + SameSurfaceSearchShapeBudget);
        int rayLimit = System.Math.Min(_raycastLimit,
            FrameRaycasts + SameSurfaceSearchRayBudget);
        if (FrameShapeQueries + 8 > shapeLimit || FrameRaycasts + 7 > rayLimit)
            return false;
        if (!TryProbeContact(bodyCenter + currentNormal * 0.18f, -currentNormal,
                clearance + 0.30f, selfBody, out SpiderSurfaceContact support, out _) ||
            Vector3.Dot(support.Normal, currentNormal) < 0.92f)
            return false;

        // The target is a body centre, possibly at raw player height. Keep all
        // three route legs on the physical source plane.
        // This is local steering, not a cast across the entire map to a
        // distant player. Reassess the remaining route after the local exit.
        Vector3 reconnect = bodyCenter + targetDelta * MathF.Min(1f, 24f / targetDistance);
        preferredSide = NormalizeOrZero(ProjectOnPlane(preferredSide, currentNormal));
        bool committedSide = preferredSide.LengthSquared() > Epsilon * Epsilon;
        float minimumOffset = SameSurfaceLegHalfWidth(clearance) + SameSurfaceDetourMargin;
        ReadOnlySpan<float> offsets = [minimumOffset, minimumOffset * 1.5f,
            minimumOffset * 2.5f, SameSurfaceDetourMaximumOffset];
        ReadOnlySpan<float> advances = [probeDistance + minimumOffset,
            8f, SameSurfaceDetourMaximumOffset];

        // First complete route on a committed side wins. Never return a merely
        // lateral escape that still points straight back into the obstacle.
        for (int sideIndex = 0; sideIndex < (committedSide ? 1 : 2); sideIndex++)
        {
            Vector3 side = committedSide ? preferredSide : (sideIndex == 0 ? right : -right);
            foreach (float offset in offsets)
            {
                float lateral = MathF.Min(offset, SameSurfaceDetourMaximumOffset);
                Vector3 candidate = bodyCenter + side * lateral;
                if (FrameShapeQueries + 2 > shapeLimit || FrameRaycasts + 6 > rayLimit)
                    return false;
                if (!IsSameSurfacePathClear(bodyCenter, candidate, currentNormal, clearance, selfBody))
                    continue;

                foreach (float advance in advances)
                {
                    if (FrameShapeQueries + 6 > shapeLimit || FrameRaycasts + 6 > rayLimit)
                        return false;
                    Vector3 exit = candidate + forward * MathF.Min(advance, targetDistance);
                    // A clear cross-section back to the pursuit axis proves
                    // this exit has passed the blocking footprint. Merely
                    // finding a clear diagonal can release the route too soon.
                    Vector3 axisExit = bodyCenter + forward * MathF.Min(advance, targetDistance);
                    if (!IsSameSurfacePathClear(candidate, exit, currentNormal, clearance, selfBody) ||
                        !IsSameSurfacePathClear(exit, axisExit, currentNormal, clearance, selfBody) ||
                        !IsSameSurfacePathClear(exit, reconnect, currentNormal, clearance, selfBody))
                        continue;
                    if (!HasSameSurfaceRouteSupport(bodyCenter, candidate, support, currentNormal, clearance, selfBody) ||
                        !HasSameSurfaceRouteSupport(candidate, exit, support, currentNormal, clearance, selfBody) ||
                        !HasSameSurfaceRouteSupport(exit, reconnect, support, currentNormal, clearance, selfBody))
                        continue;

                    waypoint = candidate;
                    exitWaypoint = exit;
                    selectedSide = side;
                    score = lateral + Vector3.Distance(candidate, exit) + Vector3.Distance(exit, reconnect);
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Reconfirms a transition contact from the target face's own normal
    /// space. The original transition hit can lie exactly on an edge, where a
    /// probe from the body alternates between two faces. Moving the probe away
    /// from the face by the body clearance and casting back makes the target
    /// face deterministic. Small tangent samples provide a stable fallback
    /// when the original hit is on a sharp mesh edge.
    /// </summary>
    public bool TryConfirmTransitionContact(
        Vector3 bodyCenter,
        in SpiderSurfaceContact proposedContact,
        Vector3 currentNormal,
        Vector3 movementDirection,
        float clearance,
        BodyID selfBody,
        out SpiderSurfaceContact contact)
    {
        contact = default;
        if (!proposedContact.IsValid)
            return false;

        currentNormal = NormalizeOrZero(currentNormal);
        Vector3 targetNormal = NormalizeOrZero(proposedContact.Normal);
        if (currentNormal.LengthSquared() <= Epsilon * Epsilon ||
            targetNormal.LengthSquared() <= Epsilon * Epsilon)
        {
            return false;
        }

        float surfaceAlignment = Vector3.Dot(currentNormal, targetNormal);
        if (surfaceAlignment >= 0.82f || surfaceAlignment <= -0.42f)
            return false;

        SpiderSurfaceContact refreshed = Refresh(proposedContact);
        if (!refreshed.IsValid)
            return false;

        targetNormal = NormalizeOrZero(refreshed.Normal);
        if (targetNormal.LengthSquared() <= Epsilon * Epsilon)
            return false;

        Vector3 faceForward = NormalizeOrZero(
            ProjectOnPlane(movementDirection, targetNormal));
        if (faceForward.LengthSquared() <= Epsilon * Epsilon)
            faceForward = BuildFallbackTangent(targetNormal);
        Vector3 faceRight = NormalizeOrZero(
            Vector3.Cross(faceForward, targetNormal));
        if (faceForward.LengthSquared() <= Epsilon * Epsilon ||
            faceRight.LengthSquared() <= Epsilon * Epsilon)
        {
            return false;
        }

        clearance = MathF.Max(0.01f, clearance);
        float normalOffset = MathF.Max(0.15f, clearance * 0.30f);
        float tangentSpread = MathF.Max(0.10f, clearance * 0.35f);
        Vector3[] tangentOffsets =
        {
            Vector3.Zero,
            faceForward * tangentSpread,
            -faceForward * tangentSpread,
            faceRight * tangentSpread,
            -faceRight * tangentSpread
        };

        float bestScore = float.MaxValue;
        foreach (Vector3 tangentOffset in tangentOffsets)
        {
            Vector3 samplePoint = refreshed.Point + tangentOffset;
            Vector3 origin = samplePoint +
                             targetNormal * (clearance + normalOffset);
            float castDistance = clearance + normalOffset + 0.25f;
            if (!TryProbeContact(
                    origin,
                    -targetNormal,
                    castDistance,
                    selfBody,
                    out SpiderSurfaceContact candidate,
                    out _))
            {
                continue;
            }

            Vector3 candidateNormal = NormalizeOrZero(candidate.Normal);
            float alignment = Vector3.Dot(candidateNormal, targetNormal);
            if (candidateNormal.LengthSquared() <= Epsilon * Epsilon ||
                alignment < 0.86f)
            {
                continue;
            }

            float pointDeviation = Vector3.DistanceSquared(
                candidate.Point,
                refreshed.Point);
            float score = (1f - alignment) * 8f +
                          tangentOffset.LengthSquared() * 0.35f +
                          pointDeviation * 0.05f;
            if (score >= bestScore)
                continue;

            bestScore = score;
            contact = candidate;
        }

        if (!contact.IsValid)
            return false;

        _lastBodyContact = contact;
        return true;
    }

    /// <summary>
    /// Acquires a nearby surface using a local frame. This is used only for
    /// spawn/recovery; normal walking stays locked to the current support or an
    /// explicitly detected adjacent face.
    /// </summary>
    public bool TryAcquireContact(
        Vector3 bodyCenter,
        Vector3 preferredNormal,
        Vector3 preferredForward,
        float searchDistance,
        BodyID selfBody,
        out SpiderSurfaceContact contact)
    {
        preferredNormal = NormalizeOrZero(preferredNormal);
        if (preferredNormal.LengthSquared() <= Epsilon * Epsilon ||
            !BuildTangentBasis(preferredNormal, preferredForward, out Vector3 forward, out Vector3 right))
        {
            contact = default;
            return false;
        }

        Vector3[] directions =
        {
            -preferredNormal,
            forward,
            -forward,
            right,
            -right,
            preferredNormal,
            NormalizeOrFallback(-preferredNormal + forward, -preferredNormal),
            NormalizeOrFallback(-preferredNormal - forward, -preferredNormal),
            NormalizeOrFallback(-preferredNormal + right, -preferredNormal),
            NormalizeOrFallback(-preferredNormal - right, -preferredNormal)
        };

        SpiderSurfaceContact best = default;
        float bestScore = float.MaxValue;
        foreach (Vector3 direction in directions)
        {
            if (!TryProbeContact(
                    bodyCenter,
                    direction,
                    searchDistance,
                    selfBody,
                    out SpiderSurfaceContact candidate,
                    out float distance))
            {
                continue;
            }

            float continuity = Vector3.Dot(candidate.Normal, preferredNormal);
            float score = distance + (1f - continuity) * 0.35f;
            if (score < bestScore)
            {
                best = candidate;
                bestScore = score;
            }
        }

        contact = best;
        if (!contact.IsValid)
            return false;

        _lastBodyContact = contact;
        return true;
    }

    /// <summary>
    /// Finds a foot target from a local, surface-relative fan. A candidate must
    /// be inside the chain reach band and is scored against the desired stride.
    /// </summary>
    public bool TryFindFootContact(Vector3 hipPosition, Vector3 desiredPosition, Vector3 expectedNormal,
        Vector3 strideDirection, float minReach, float maxReach, BodyID selfBody,
        SpiderSurfaceContact preferredContact, out SpiderSurfaceContact contact,
        float footprintRadius = 0f, Func<SpiderSurfaceContact, bool>? accept = null)
    {
        contact = default;
        expectedNormal = NormalizeOrZero(expectedNormal);
        if (maxReach <= 0f || !BuildTangentBasis(expectedNormal, strideDirection, out Vector3 forward, out Vector3 right))
            return false;
        var hits = new List<ProbeHit>(16);
        float spread = maxReach * 0.09f;
        float height = maxReach * _profile.ProbeHeightFractionOfReach;
        float distance = maxReach * _profile.ProbeDistanceFractionOfReach;
        ReadOnlySpan<Vector3> offsets = [Vector3.Zero, forward * spread, -forward * spread,
            right * spread, -right * spread, (forward + right) * spread, (forward - right) * spread];
        foreach (Vector3 offset in offsets)
            AddProbe(hits, desiredPosition + offset + expectedNormal * height, -expectedNormal,
                distance, selfBody, expectedNormal, 0f);
        AddProbe(hits, hipPosition, NormalizeOrFallback(desiredPosition - hipPosition, -expectedNormal),
            maxReach, selfBody, expectedNormal, 0.1f);
        AddProbe(hits, desiredPosition + expectedNormal * height * 0.3f,
            NormalizeOrFallback(-expectedNormal + forward, -expectedNormal), distance, selfBody, expectedNormal, 0.2f);
        AddProbe(hits, desiredPosition + expectedNormal * height * 0.3f,
            NormalizeOrFallback(-expectedNormal - forward, -expectedNormal), distance, selfBody, expectedNormal, 0.2f);
        foreach (Vector3 side in new[] { forward, -forward, right, -right })
            AddProbe(hits, hipPosition, side, maxReach, selfBody, expectedNormal, 0.4f);

        float Score(ProbeHit hit)
        {
            float score = Vector3.DistanceSquared(hit.Hit.Position, desiredPosition) / (maxReach * maxReach) +
                (1f - Vector3.Dot(hit.Hit.Normal, expectedNormal)) * 0.35f + hit.Priority * 0.1f;
            if (preferredContact.IsValid && hit.Hit.BodyID == preferredContact.BodyId)
                score -= 0.08f * MathF.Max(0f, Vector3.Dot(hit.Hit.Normal, preferredContact.Normal));
            return score;
        }
        hits.Sort((a, b) => Score(a).CompareTo(Score(b)));
        foreach (ProbeHit hit in hits)
        {
            float reach = Vector3.Distance(hipPosition, hit.Hit.Position);
            if (reach < minReach || reach > maxReach ||
                Vector3.Dot(hit.Hit.Normal, expectedNormal) < _profile.MinimumContactNormalAlignment) continue;
            var candidate = CreateContact(hit.Hit, 1f / (1f + hit.Priority));
            if (!candidate.IsValid || footprintRadius > 0f && !HasSupportPatch(candidate, footprintRadius, selfBody) ||
                accept?.Invoke(candidate) == false) continue;
            contact = candidate;
            _debugCandidates.Add(contact);
            return true;
        }
        return false;
    }

    /// <summary>Overlap and swept capsule queries used by the complete IK chain.</summary>
    public bool IsSegmentClear(Vector3 a, Vector3 b, float radius, BodyID selfBody, float skin = 0.004f)
    {
        if (!TakeShapeQuery()) return false;
        Shape shape = SegmentShape(a, b, radius, out Matrix4x4 transform);
        var bodies = QueryBodyFilter(selfBody);
        Vector3 scale = Vector3.One, offset = Vector3.Zero;
        transform = Matrix4x4.Transpose(transform);
        _overlapHits.Clear();
        var settings = new CollideShapeSettings { BackFaceMode = BackFaceMode.CollideWithBackFaces };
        _scene.Physics.NarrowPhaseQuery.CollideShape(shape, in scale, in transform, in settings, in offset,
            CollisionCollectorType.AllHit, _overlapHits, _queryBroadPhase, _queryObjectLayer, bodies, _queryShapeFilter);
        foreach (var hit in _overlapHits)
            if (hit.PenetrationDepth > skin) return false;
        return true;
    }

    public bool IsSegmentMotionClear(Vector3 a, Vector3 b, Vector3 nextA, Vector3 nextB,
        float radius, BodyID selfBody, float skin = 0.004f)
    {
        Vector3 translation = ((nextA - a) + (nextB - b)) * 0.5f;
        // A translated capsule inflated by the non-translational endpoint
        // motion contains the entire linearly swept rotating segment.
        float expansion = MathF.Max((nextA - a - translation).Length(), (nextB - b - translation).Length());
        if (_disposed || FrameShapeQueries >= _shapeQueryLimit)
                {
                    QueryBudgetExhausted = true;
                    return false;
                }
                Shape shape = SegmentShape(a, b, radius + expansion, out Matrix4x4 transform);
        return ShapeTravelFraction(shape, transform, translation, selfBody, skin, hit =>
        {
            // Inflation bounds the rotating capsule, but can overlap a floor
            // that the actual segment never approaches. Reject that false
            // positive only with a separating plane for all four endpoints.
            _scene.Physics.BodyLockInterface.LockRead(hit.BodyID2, out BodyLockRead readLock);
            if (!readLock.Succeeded) return false;
            try
            {
                if (readLock.Body is not { } body) return false;
                Vector3 point = hit.ContactPointOn2;
                Vector3 normal = body.GetWorldSpaceSurfaceNormal(hit.SubShapeID2, point);
                normal = NormalizeOrZero(normal);
                if (Vector3.Dot((a + b) * 0.5f - point, normal) < 0f) normal = -normal;
                float minimum = MathF.Min(MathF.Min(Vector3.Dot(a - point, normal), Vector3.Dot(b - point, normal)),
                    MathF.Min(Vector3.Dot(nextA - point, normal), Vector3.Dot(nextB - point, normal)));
                return normal.LengthSquared() > 0.5f && minimum >= radius - skin;
            }
            finally { _scene.Physics.BodyLockInterface.UnlockRead(readLock); }
        }) >= 1f;
    }

    public float ShapeTravelFraction(Shape shape, Matrix4x4 transform, Vector3 displacement,
        BodyID selfBody, float skin = 0.004f, Func<ShapeCastResult, bool>? separatedFromHit = null)
    {
        if (!TakeShapeQuery()) return 0f;
        var bodies = QueryBodyFilter(selfBody);
        Vector3 baseOffset = Vector3.Zero;
        // JoltPhysicsSharp's shape-query ABI consumes column-major transforms.
        // Passing System.Numerics translation in M41 casts at the origin.
        transform = Matrix4x4.Transpose(transform);
        _castHits.Clear();
        var settings = new ShapeCastSettings
        {
            BackFaceModeTriangles = BackFaceMode.CollideWithBackFaces,
            BackFaceModeConvex = BackFaceMode.CollideWithBackFaces,
            ReturnDeepestPoint = true
        };
        _scene.Physics.NarrowPhaseQuery.CastShape(shape, in transform, in displacement, settings, in baseOffset,
            CollisionCollectorType.AllHit, _castHits, _queryBroadPhase, _queryObjectLayer, bodies, _queryShapeFilter);
        float fraction = 1f;
        foreach (var hit in _castHits)
        {
            if (separatedFromHit?.Invoke(hit) == true) continue;
            // Tangency is allowed only when moving away/parallel. Body identity
            // or proximity to the endpoints never makes a wall benign.
            if (hit.Fraction <= 0f && hit.PenetrationDepth <= skin &&
                Vector3.Dot(displacement, hit.PenetrationAxis) <= 0.000001f) continue;
            fraction = MathF.Min(fraction, MathF.Max(0f, hit.Fraction - skin / MathF.Max(displacement.Length(), 0.001f)));
        }
        return fraction;
    }

    private Shape SegmentShape(Vector3 a, Vector3 b, float radius, out Matrix4x4 transform)
    {
        Vector3 delta = b - a;
        float length = delta.Length();
        Quaternion rotation = Animation.SpiderLocomotionMath.RotationBetween(Vector3.UnitY, delta, Vector3.UnitX);
        transform = Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation((a + b) * 0.5f);
        // Round outward by at most 0.1 mm: cached shapes conservatively contain
        // the requested segment while avoiding native allocation per IK trial.
        var key = (Length: (int)MathF.Ceiling(length * 10000f), Radius: (int)MathF.Ceiling(MathF.Max(radius, 0.001f) * 10000f));
        if (_segmentShapes.TryGetValue(key, out Shape? shape)) return shape;
        if (_segmentShapes.Count >= 128)
        {
            var oldest = _shapeOrder.Dequeue();
            _segmentShapes[oldest].Dispose();
            _segmentShapes.Remove(oldest);
        }
        shape = key.Length == 0 ? new SphereShape(key.Radius / 10000f) :
            new CapsuleShape(key.Length / 20000f, key.Radius / 10000f);
        _segmentShapes.Add(key, shape);
        _shapeOrder.Enqueue(key);
        return shape;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _queryBodyFilter?.Dispose();
        _queryShapeFilter.Dispose();
        _queryBroadPhase.Dispose();
        _queryObjectLayer.Dispose();
        foreach (Shape shape in _segmentShapes.Values) shape.Dispose();
        _segmentShapes.Clear();
        _shapeOrder.Clear();
    }

    public bool IsFootStepPathClear(Vector3 start, Vector3 end, Vector3 startNormal, Vector3 endNormal,
        float liftHeight, float radius, BodyID selfBody, SpiderSurfaceContact startContact,
        SpiderSurfaceContact endContact, out float blockedFraction)
    {
        int samples = System.Math.Clamp((int)MathF.Ceiling((Vector3.Distance(start, end) + liftHeight * 2f) /
            MathF.Max(radius, 0.03f)), 8, 64);
        Vector3 previous = start;
        for (int i = 1; i <= samples; i++)
        {
            Vector3 next = Animation.SpiderLocomotionMath.StepPoint(start, end, startNormal, endNormal, liftHeight, (float)i / samples);
            if (!IsSegmentClear(next, next, radius, selfBody) ||
                !IsSegmentMotionClear(previous, previous, next, next, radius, selfBody))
            {
                blockedFraction = (float)(i - 1) / samples;
                return false;
            }
            previous = next;
        }
        blockedFraction = 1f;
        return true;
    }

    public bool HasSupportPatch(in SpiderSurfaceContact contact, float radius, BodyID selfBody)
    {
        if (!Refresh(contact).IsValid) return false;
        if (!BuildTangentBasis(contact.Normal, Vector3.UnitX, out Vector3 x, out Vector3 z)) return false;
        ReadOnlySpan<Vector3> offsets = [x * radius, -x * radius, z * radius, -z * radius];
        foreach (Vector3 offset in offsets)
        {
            if (!TakeRaycast() || !_scene.Raycast(contact.Point + offset + contact.Normal * (radius + 0.04f), -contact.Normal,
                    radius + 0.08f, out var hit, selfBody, true, _ignoredBodies, walkableOnly: true) ||
                hit.BodyID != contact.BodyId || Vector3.Dot(hit.Normal, contact.Normal) < _profile.ContactPatchNormalAlignment)
                return false;
        }
        return true;
    }

    /// <summary>
    /// Checks a tangent corridor from a body centre: the physical capsule and
    /// a conservative feet/leg band above the expected support plane. Costs at
    /// most two shape queries and no rays; false also means invalid input or
    /// insufficient budget. This does not prove continuous floor support.
    /// </summary>
    public bool IsSameSurfacePathClear(
        Vector3 start,
        Vector3 end,
        Vector3 surfaceNormal,
        float clearance,
        BodyID selfBody,
        float maxDistance = float.MaxValue)
    {
        if (_disposed || !IsFinite(start) || !IsFinite(end) ||
            !IsFinite(surfaceNormal) || !float.IsFinite(clearance) || clearance <= 0f ||
            !float.IsFinite(maxDistance) || maxDistance <= 0f)
            return false;
        surfaceNormal = NormalizeOrZero(surfaceNormal);
        if (surfaceNormal.LengthSquared() <= Epsilon * Epsilon)
            return false;
        Vector3 path = ProjectOnPlane(end - start, surfaceNormal);
        float pathLength = path.Length();
        if (!float.IsFinite(pathLength)) return false;
        if (pathLength <= Epsilon) return true;
        if (FrameShapeQueries + 2 > _shapeQueryLimit)
        {
            QueryBudgetExhausted = true;
            return false;
        }

        Vector3 direction = path / pathLength;
        Vector3 width = NormalizeOrZero(Vector3.Cross(surfaceNormal, direction));
        Vector3 displacement = direction * MathF.Min(pathLength, maxDistance);
        Matrix4x4 orientation = new(
            width.X, width.Y, width.Z, 0f,
            surfaceNormal.X, surfaceNormal.Y, surfaceNormal.Z, 0f,
            direction.X, direction.Y, direction.Z, 0f,
            0f, 0f, 0f, 1f);
        using (var capsule = new CapsuleShape(_profile.BodyCylinderHeight * 0.5f, _profile.BodyRadius))
        {
            if (ShapeTravelFraction(capsule, orientation * Matrix4x4.CreateTranslation(start),
                    displacement, selfBody) < 1f)
                return false;
        }

        // A swept footprint covers the entire width, not three rays with gaps.
        // Use the same extent fore/aft: gait stance rotates while steering, so
        // a zero-length cross section would detect turns into low boxes late.
        // Its bottom stays above support seams, but below the body capsule's
        // lowest point so low boxes cannot hide underneath the front probe.
        float bottom = MathF.Max(_profile.CollisionSkinWorld * 2f, 0.025f);
        float top = clearance + _profile.BodyRadius * 0.5f;
        float halfHeight = (top - bottom) * 0.5f;
        Vector3 bandCenter = start + surfaceNormal * ((top + bottom) * 0.5f - clearance);
        float stanceExtent = SameSurfaceLegHalfWidth(clearance);
        using var band = new BoxShape(new Vector3(stanceExtent, halfHeight, stanceExtent), 0.01f);
        return ShapeTravelFraction(band, orientation * Matrix4x4.CreateTranslation(bandCenter),
            displacement, selfBody) >= 1f;
    }

    private float SameSurfaceLegHalfWidth(float clearance) =>
        // Include turn/waypoint tolerance and the motor's near-face lookahead.
        // The profile has no rig-specific stance extents; this is deliberately
        // conservative and never substitutes for gait pose validation.
        MathF.Max(clearance * 3f, _profile.SurfaceTransitionProbeWorld + SameSurfaceDetourMargin);

    private bool HasSameSurfaceRouteSupport(Vector3 start, Vector3 end,
        in SpiderSurfaceContact source, Vector3 normal, float clearance, BodyID selfBody)
    {
        // Two bounded samples per leg reject raised tops and obvious gaps.
        // This is a local route approximation, not an edge/ledge navigator.
        for (int i = 1; i <= 2; i++)
        {
            Vector3 center = Vector3.Lerp(start, end, i * 0.5f);
            if (!TryProbeContact(center + normal * 0.18f, -normal, clearance + 0.30f,
                    selfBody, out SpiderSurfaceContact contact, out _) ||
                Vector3.Dot(contact.Normal, normal) < 0.92f ||
                MathF.Abs(Vector3.Dot(contact.Point - source.Point, normal)) >
                    MathF.Max(_profile.ContactToleranceWorld * 2f, 0.04f))
                return false;
        }
        return true;
    }

    private bool TryProbeContact(
        Vector3 origin,
        Vector3 direction,
        float distance,
        BodyID selfBody,
        out SpiderSurfaceContact contact,
        out float hitDistance)
    {
        direction = NormalizeOrZero(direction);
        if (direction.LengthSquared() <= Epsilon * Epsilon || !TakeRaycast())
        {
            contact = default;
            hitDistance = float.MaxValue;
            return false;
        }

        bool didHit = _scene.Raycast(
            origin,
            direction,
            distance,
            out SceneRaycastHit hit,
            selfBody,
            collideWithBackFaces: true,
            excludedBodies: _ignoredBodies, walkableOnly: true);

        _debugProbes.Add(new DebugProbe(
            origin,
            didHit ? hit.Position : origin + direction * distance,
            didHit));

        if (!didHit || Vector3.Dot(hit.Normal, -direction) < 0.08f)
        {
            contact = default;
            hitDistance = float.MaxValue;
            return false;
        }

        contact = CreateContact(hit, 1f);
        hitDistance = hit.Distance;
        _debugCandidates.Add(contact);
        return true;
    }

    private static bool IsUsableTransition(Vector3 candidateNormal, Vector3 currentNormal)
    {
        candidateNormal = NormalizeOrZero(candidateNormal);
        currentNormal = NormalizeOrZero(currentNormal);
        if (candidateNormal.LengthSquared() <= Epsilon * Epsilon ||
            currentNormal.LengthSquared() <= Epsilon * Epsilon)
        {
            return false;
        }

        float alignment = Vector3.Dot(candidateNormal, currentNormal);

        // Ignore the same face and the exact opposite face. Adjacent faces and
        // smooth slopes remain valid, including transitions slightly over 90°.
        return alignment < 0.82f && alignment > -0.42f;
    }

    private void AddProbe(
        List<ProbeHit> hits,
        Vector3 origin,
        Vector3 direction,
        float distance,
        BodyID selfBody,
        Vector3 expectedNormal,
        float priority)
    {
        direction = NormalizeOrZero(direction);
        if (direction.LengthSquared() <= Epsilon * Epsilon || !TakeRaycast())
            return;

        bool didHit = _scene.Raycast(
            origin,
            direction,
            distance,
            out SceneRaycastHit hit,
            selfBody,
            collideWithBackFaces: true,
            excludedBodies: _ignoredBodies, walkableOnly: true);
        _debugProbes.Add(new DebugProbe(origin, didHit ? hit.Position : origin + direction * distance, didHit));
        if (!didHit)
            return;

        // The resolved normal must face the probe even when the underlying
        // triangle was reached through its back face.
        if (Vector3.Dot(hit.Normal, -direction) < 0.10f)
            return;

        hits.Add(new ProbeHit(hit, priority));
    }

    private SpiderSurfaceContact CreateContact(
        in SceneRaycastHit hit,
        float confidence,
        Vector3? pointOverride = null,
        Vector3? normalOverride = null)
    {
        Vector3 point = pointOverride ?? hit.Position;
        Vector3 normal = NormalizeOrZero(normalOverride ?? hit.Normal);
        if (normal.LengthSquared() <= Epsilon * Epsilon)
            return default;

        RigidBody? supportBody = hit.RigidBody;
        Vector3 localPoint = point;
        Vector3 localNormal = normal;
        if (hit.BodyID.IsValid)
        {
            Quaternion inverse = Quaternion.Inverse(_scene.Physics.GetBodyRotation(hit.BodyID));
            localPoint = Vector3.Transform(point - _scene.Physics.GetBodyPosition(hit.BodyID), inverse);
            localNormal = Vector3.Transform(normal, inverse);
        }

        return new SpiderSurfaceContact(
            true,
            point,
            normal,
            hit.BodyID,
            supportBody,
            localPoint,
            localNormal,
            confidence);
    }

    private static bool BuildTangentBasis(Vector3 normal, Vector3 desiredForward, out Vector3 forward, out Vector3 right)
    {
        normal = NormalizeOrZero(normal);
        if (normal.LengthSquared() <= Epsilon * Epsilon)
        {
            forward = Vector3.Zero;
            right = Vector3.Zero;
            return false;
        }

        forward = desiredForward - normal * Vector3.Dot(desiredForward, normal);
        forward = NormalizeOrZero(forward);
        if (forward.LengthSquared() <= Epsilon * Epsilon)
            forward = BuildFallbackTangent(normal);
        if (forward.LengthSquared() <= Epsilon * Epsilon)
        {
            right = Vector3.Zero;
            return false;
        }

        right = NormalizeOrZero(Vector3.Cross(forward, normal));
        if (right.LengthSquared() <= Epsilon * Epsilon)
            return false;

        forward = NormalizeOrZero(Vector3.Cross(normal, right));
        return forward.LengthSquared() > Epsilon * Epsilon;
    }

    private static Vector3 BuildFallbackTangent(Vector3 normal)
    {
        normal = NormalizeOrZero(normal);
        if (normal.LengthSquared() <= Epsilon * Epsilon)
            return Vector3.Zero;

        Vector3 reference = Vector3.UnitX;
        float smallestAlignment = MathF.Abs(Vector3.Dot(normal, reference));
        Vector3[] basisCandidates = { Vector3.UnitY, Vector3.UnitZ };
        foreach (Vector3 candidate in basisCandidates)
        {
            float alignment = MathF.Abs(Vector3.Dot(normal, candidate));
            if (alignment >= smallestAlignment)
                continue;

            reference = candidate;
            smallestAlignment = alignment;
        }

        return NormalizeOrZero(Vector3.Cross(normal, reference));
    }

    private static Vector3 NormalizeOrFallback(Vector3 value, Vector3 fallback)
    {
        Vector3 normalizedValue = NormalizeOrZero(value);
        if (normalizedValue.LengthSquared() > Epsilon * Epsilon)
            return normalizedValue;
        return NormalizeOrZero(fallback);
    }

    private static Vector3 NormalizeOrZero(Vector3 value)
    {
        return value.LengthSquared() > Epsilon * Epsilon
            ? Vector3.Normalize(value)
            : Vector3.Zero;
    }

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) &&
        float.IsFinite(value.Y) &&
        float.IsFinite(value.Z);

    private static Vector3 ProjectOnPlane(Vector3 value, Vector3 normal) =>
        value - normal * Vector3.Dot(value, normal);

    public void OnDrawGizmos(DebugDrawer drawer)
    {
        //foreach (DebugProbe probe in _debugProbes)
        //{
        //    Vector3 color = probe.Hit ? new Vector3(0.15f, 0.85f, 1f) : new Vector3(0.9f, 0.15f, 0.15f);
        //    drawer.PushLine(probe.Start, probe.End, color);
        //}

        //foreach (SpiderSurfaceContact candidate in _debugCandidates)
        //{
        //    if (!candidate.IsValid)
        //        continue;
        //    drawer.DrawSphere(candidate.Point, Quaternion.Identity, 0.05f, new Vector3(1f, 0.85f, 0.1f));
        //    drawer.PushLine(candidate.Point, candidate.Point + candidate.Normal * 0.35f, new Vector3(1f, 0.85f, 0.1f));
        //}

        //if (_lastBodyContact.IsValid)
        //{
        //    drawer.DrawSphere(_lastBodyContact.Point, Quaternion.Identity, 0.10f, new Vector3(0.1f, 1f, 0.3f));
        //    drawer.PushLine(_lastBodyContact.Point, _lastBodyContact.Point + _lastBodyContact.Normal * 0.75f, new Vector3(0.1f, 1f, 0.3f));
        //}
    }

    private readonly struct ProbeHit
    {
        public ProbeHit(SceneRaycastHit hit, float priority)
        {
            Hit = hit;
            Priority = priority;
        }

        public SceneRaycastHit Hit { get; }
        public float Priority { get; }
    }

    private readonly struct DebugProbe
    {
        public DebugProbe(Vector3 start, Vector3 end, bool hit)
        {
            Start = start;
            End = end;
            Hit = hit;
        }

        public Vector3 Start { get; }
        public Vector3 End { get; }
        public bool Hit { get; }
    }
}

public readonly struct SpiderSurfaceContact
{
    public SpiderSurfaceContact(
        bool isValid,
        Vector3 point,
        Vector3 normal,
        BodyID bodyId,
        RigidBody? supportBody,
        Vector3 localPoint,
        Vector3 localNormal,
        float confidence)
    {
        IsValid = isValid;
        Point = point;
        Normal = normal;
        BodyId = bodyId;
        SupportBody = supportBody;
        LocalPoint = localPoint;
        LocalNormal = localNormal;
        Confidence = confidence;
    }

    public bool IsValid { get; }
    public Vector3 Point { get; }
    public Vector3 Normal { get; }
    public BodyID BodyId { get; }
    public RigidBody? SupportBody { get; }
    public Vector3 LocalPoint { get; }
    public Vector3 LocalNormal { get; }
    public float Confidence { get; }

    public SpiderSurfaceContact WithWorldPose(Vector3 point, Vector3 normal) => new(
        IsValid,
        point,
        normal,
        BodyId,
        SupportBody,
        LocalPoint,
        LocalNormal,
        Confidence);
}
