using JoltPhysicsSharp;

namespace Fuse.Physics;

/// <summary>Locomotion sees solid scenery, never damage/trigger sensors or actors.</summary>
public sealed class SurfaceBodyFilter : BodyFilter
{
    private readonly BodyID? _self;
    private readonly IReadOnlySet<BodyID>? _ignored;
    private readonly IReadOnlySet<BodyID> _actors;

    public SurfaceBodyFilter(
        BodyID? self,
        IReadOnlySet<BodyID>? ignored,
        IReadOnlySet<BodyID> actors)
    {
        OwnsHandle = true;
        _self = self;
        _ignored = ignored;
        _actors = actors;
    }
    protected override bool ShouldCollide(BodyID id) =>
        id != _self && _ignored?.Contains(id) != true && !_actors.Contains(id);

    protected override bool ShouldCollideLocked(Body body) =>
        !body.IsSensor && ShouldCollide(body.ID);
}
