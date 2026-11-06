using System;
using System.Numerics;
using Fuse.Physics;
using Fuse.Scene;
using JoltPhysicsSharp;

namespace Fuse.Enemy;

/// <summary>
/// Resolves local physical support for a pursuit target, not navigation data.
/// Target positions are feet/support positions, not capsule centres.
/// </summary>
public sealed class SpiderTargetSurfaceResolver
{
    private const float Epsilon = 0.0001f;
    private const float ProbeLift = 0.2f;
    private const float FootprintRadius = 0.35f;
    private const float MinimumSupportAlignment = 0.5f;

    private readonly SceneManager _scene;

    public SpiderTargetSurfaceResolver(SceneManager scene)
    {
        _scene = scene ?? throw new ArgumentNullException(nameof(scene));
    }

    public bool TryResolvePlayer(
        Vector3 feetPosition,
        CharacterVirtual character,
        float searchDistance,
        out SpiderTargetSurface surface)
    {
        surface = default;
        if (!IsFinite(feetPosition) || !float.IsFinite(searchDistance) || searchDistance <= 0f)
            return false;

        Vector3 up = NormalizeOrZero(character.Up);
        if (up == Vector3.Zero)
            up = Vector3.UnitY;

        // CharacterVirtual has no queryable body unless InnerBodyShape was set.
        BodyID? excludedBody = character.InnerBodyID.IsValid ? character.InnerBodyID : null;
        Vector3 supportPosition = character.GroundPosition;
        Vector3 supportOffset = supportPosition - feetPosition;
        if (character.GroundState == GroundState.OnGround &&
            IsFinite(supportPosition) &&
            supportOffset.LengthSquared() <= 0.75f * 0.75f &&
            MathF.Abs(Vector3.Dot(supportOffset, up)) <= 0.5f &&
            Vector3.Dot(NormalizeOrZero(character.GroundNormal), up) >= MinimumSupportAlignment)
        {
            // On a curb the capsule's actual contact can be beside and above
            // its lowest point. Re-query that contact: native ground data can
            // be stale after teleportation or refer to a trigger/actor.
            if (TryProbe(supportPosition, up, 0.1f, excludedBody, out var support) &&
                Vector3.DistanceSquared(support.Point, supportPosition) <= 0.08f * 0.08f)
            {
                surface = new SpiderTargetSurface(true, support.Point, support.Normal,
                    support.BodyId, Vector3.Distance(support.Point, feetPosition));
                return true;
            }
        }

        return TryResolve(feetPosition, up, searchDistance, out surface, excludedBody);
    }

    public bool TryResolve(
        Vector3 targetPosition,
        Vector3 preferredNormal,
        float searchDistance,
        out SpiderTargetSurface surface,
        BodyID? excludedBody = null)
    {
        surface = default;
        if (!IsFinite(targetPosition) || !float.IsFinite(searchDistance) || searchDistance <= 0f)
            return false;

        Vector3 up = NormalizeOrZero(preferredNormal);
        if (up == Vector3.Zero)
            up = Vector3.UnitY;

        // Always prefer support directly under the feet. Symmetric rays from
        // +/- half the search distance can enter the opposite face of a wall,
        // ceiling or capsule; SceneManager also flips normals against the ray.
        if (TryProbe(targetPosition, up, searchDistance, excludedBody, out surface))
            return true;

        Vector3 reference = MathF.Abs(up.X) < 0.8f ? Vector3.UnitX : Vector3.UnitZ;
        Vector3 forward = Vector3.Normalize(Vector3.Cross(up, reference));
        Vector3 right = Vector3.Cross(forward, up);
        ReadOnlySpan<Vector3> offsets = stackalloc Vector3[]
        {
            forward * FootprintRadius, -forward * FootprintRadius,
            right * FootprintRadius, -right * FootprintRadius
        };

        float bestDistance = float.MaxValue;
        foreach (Vector3 offset in offsets)
        {
            if (!TryProbe(targetPosition + offset, up, searchDistance, excludedBody, out var candidate))
                continue;

            float distance = Vector3.Distance(candidate.Point, targetPosition);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            surface = new SpiderTargetSurface(true, candidate.Point, candidate.Normal,
                candidate.BodyId, distance);
        }

        return surface.IsValid;
    }

    private bool TryProbe(
        Vector3 feetPosition,
        Vector3 up,
        float searchDistance,
        BodyID? excludedBody,
        out SpiderTargetSurface surface)
    {
        surface = default;
        Vector3 origin = feetPosition + up * ProbeLift;
        float length = searchDistance + ProbeLift;
        Vector3 direction = -up * length;
        var ray = new Ray(in origin, in direction);
        var settings = new RayCastSettings
        {
            BackFaceModeTriangles = BackFaceMode.IgnoreBackFaces,
            BackFaceModeConvex = BackFaceMode.IgnoreBackFaces,
            TreatConvexAsSolid = true
        };
        using var broadPhase = new DefaultBroadPhaseLayerFilter();
        using var objectLayer = new DefaultObjectLayerFilter();
        using var bodies = new SurfaceBodyFilter(excludedBody, null, _scene.NonWalkableBodies);
        using var shapes = new DefaultShapeFilter();
        var hits = new List<RayCastResult>(1);
        if (!_scene.Physics.NarrowPhaseQuery.CastRay(ray, settings,
                CollisionCollectorType.ClosestHit, hits, broadPhase, objectLayer, bodies, shapes) ||
            hits.Count == 0)
        {
            return false;
        }

        RayCastResult hit = hits[0];
        if (!hit.BodyID.IsValid || !float.IsFinite(hit.Fraction) || hit.Fraction * length <= Epsilon)
            return false;

        Vector3 point = origin + direction * hit.Fraction;
        Vector3 normal;
        var locks = _scene.Physics.BodyLockInterface;
        locks.LockRead(hit.BodyID, out BodyLockRead readLock);
        if (!readLock.Succeeded)
            return false;
        try
        {
            if (readLock.Body is not { } body)
                return false;
            var subShape = new SubShapeID(hit.subShapeID2);
            // SceneManager.Raycast flips this normal to face the ray. Support
            // needs the original normal, otherwise an underside looks valid.
            normal = NormalizeOrZero(body.GetWorldSpaceSurfaceNormal(in subShape, in point));
        }
        finally { locks.UnlockRead(readLock); }

        if (!IsFinite(point) || Vector3.Dot(normal, up) < MinimumSupportAlignment)
            return false;

        surface = new SpiderTargetSurface(true, point, normal, hit.BodyID,
            Vector3.Distance(point, feetPosition));
        return true;
    }

    private static Vector3 NormalizeOrZero(Vector3 value) =>
        IsFinite(value) && float.IsFinite(value.LengthSquared()) && value.LengthSquared() > Epsilon * Epsilon
            ? Vector3.Normalize(value)
            : Vector3.Zero;

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) &&
        float.IsFinite(value.Y) &&
        float.IsFinite(value.Z);
}

public readonly struct SpiderTargetSurface
{
    public SpiderTargetSurface(
        bool isValid,
        Vector3 point,
        Vector3 normal,
        BodyID bodyId,
        float distance)
    {
        IsValid = isValid;
        Point = point;
        Normal = normal;
        BodyId = bodyId;
        Distance = distance;
    }

    public bool IsValid { get; }
    public Vector3 Point { get; }
    public Vector3 Normal { get; }
    public BodyID BodyId { get; }
    public float Distance { get; }
}
