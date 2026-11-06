using JoltPhysicsSharp;

namespace Fuse.Physics;

public sealed class DefaultBodyFilter : BodyFilter
{
    public DefaultBodyFilter() => OwnsHandle = true;
    protected override bool ShouldCollide(BodyID bodyId) => true;
    protected override bool ShouldCollideLocked(Body body) => true;
}

public sealed class DefaultShapeFilter : ShapeFilter
{
    public DefaultShapeFilter() => OwnsHandle = true;
}

public sealed class DefaultBroadPhaseLayerFilter : BroadPhaseLayerFilter
{
    public DefaultBroadPhaseLayerFilter() => OwnsHandle = true;
    protected override bool ShouldCollide(BroadPhaseLayer layer) => true;
}

public sealed class DefaultObjectLayerFilter : ObjectLayerFilter
{
    public DefaultObjectLayerFilter() => OwnsHandle = true;
    protected override bool ShouldCollide(ObjectLayer layer) => true;
}
