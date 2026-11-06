using System.Numerics;
using Fuse.Animation;
using Fuse.Enemy;
using static Fuse.Animation.SpiderLocomotionMath;
using static Assertions;

// No framework, renderer, physics runtime, imported rig, or random seed required.
var tests = new List<(string Name, Action Body)>();
void Test(string name, Action body) => tests.Add((name, body));

foreach (var (name, up, tangent) in new (string, Vector3, Vector3)[]
{
    ("floor", Vector3.UnitY, Vector3.UnitX),
    ("wall", Vector3.UnitX, Vector3.UnitZ),
    ("ceiling", -Vector3.UnitY, Vector3.UnitX),
    ("back wall", Vector3.UnitZ, Vector3.UnitX),
})
{
    Test($"ContainsSupport/{name}: hull, projection, boundary and signed margins", () =>
    {
        Vector3 origin = new(3, -2, 5);
        Vector3 across = Vector3.Cross(up, tangent);
        Vector3[] square =
        [
            origin + tangent + across, origin - tangent - across,
            origin + tangent - across, origin - tangent + across,
            origin, origin + tangent + across, // Interior point and duplicate, deliberately unsorted.
        ];
        Check(ContainsSupport(square, origin, up, 0f), "center must be supported");
        Check(ContainsSupport(square, origin + up * 20f, up, 0f), "normal displacement must not affect projection");
        Check(ContainsSupport(square, origin, up, 1f), "exact inset boundary is inclusive");
        Check(!ContainsSupport(square, origin, up, 1.01f), "margin must erode the hull");
        Check(ContainsSupport(square, origin + tangent * 0.7f, up, 0.25f), "inside positive margin");
        Check(!ContainsSupport(square, origin + tangent * 0.8f, up, 0.25f), "inside hull but outside inset");
        Check(ContainsSupport(square, origin + tangent, up, 0f), "edge is inclusive");
        Check(ContainsSupport(square, origin + tangent + across, up, 0f), "vertex is inclusive");
        Check(!ContainsSupport(square, origin + tangent * 1.05f, up, 0f), "outside hull");
        Check(ContainsSupport(square, origin + tangent * 1.05f, up, -0.1f), "negative margin expands hull");
        Check(!ContainsSupport(square, origin + tangent * 1.15f, up, -0.1f), "outside expanded hull");
        Array.Reverse(square);
        Check(ContainsSupport(square, origin, up, 0.25f), "input order must not matter");
    });

    Test($"ContainsSupport/{name}: degenerate sets", () =>
    {
        Vector3 p = new(2, -3, 4);
        Vector3[][] sets =
        [
            [], [p], [p - tangent, p + tangent], [p, p, p, p],
            [p + tangent, p - tangent, p, p + tangent * 2f],
            [p, p + up, p - up, p + up * 3f], // Distinct 3D points, one projected point.
        ];
        foreach (Vector3[] points in sets)
            foreach (float margin in new[] { -0.1f, 0f, 0.1f })
                Check(!ContainsSupport(points, p, up, margin), $"degenerate support ({points.Length} points, margin {margin})");
    });
}

Test("ContainsSupport/triangle uses convex hull, not bounding box", () =>
{
    Vector3[] triangle = [Vector3.Zero, new(2, 0, 0), new(0, 0, 2)];
    Check(ContainsSupport(triangle, new(0.5f, 0, 0.5f), Vector3.UnitY, 0.1f), "triangle interior");
    Check(!ContainsSupport(triangle, new(1.5f, 0, 1.5f), Vector3.UnitY, 0f), "outside diagonal but inside AABB");
    Check(ContainsSupport(triangle, new(1, 0, 1), Vector3.UnitY, 0f), "diagonal boundary");
});

Test("StepPoint/flat arc, endpoints, clamping and symmetry", () =>
{
    Vector3 start = new(-1, 2, 3), end = new(3, 2, 3);
    const float lift = 0.7f;
    Near(StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, -1), start, 1e-6f, "clamp before start");
    Near(StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, 0), start, 1e-6f, "lift-off");
    Near(StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, 1), end, 1e-6f, "touchdown");
    Near(StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, 2), end, 1e-6f, "clamp after end");
    Near(StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, 0.5f), (start + end) * 0.5f + Vector3.UnitY * lift, 1e-6f, "midpoint");
    float previousX = start.X;
    for (int i = 0; i <= 128; i++)
    {
        float t = i / 128f;
        Vector3 p = StepPoint(start, end, Vector3.UnitY, Vector3.UnitY, lift, t);
        Check(p.X >= previousX - 2e-6f, $"horizontal progress at {t}");
        Check(p.Y >= start.Y - 1e-6f && p.Y <= start.Y + lift + 1e-6f, $"arc bounds at {t}");
        Near(p.Z, start.Z, 1e-6f, "no lateral drift");
        Vector3 reverse = StepPoint(end, start, Vector3.UnitY, Vector3.UnitY, lift, 1f - t);
        Near(p, reverse, 5e-6f, $"time reversal at {t}");
        previousX = p.X;
    }
});

foreach (var (name, from, to) in new (string, Vector3, Vector3)[]
{
    ("floor", Vector3.UnitY, Vector3.UnitY),
    ("floor to wall", Vector3.UnitY, Vector3.UnitX),
    ("floor to ceiling", Vector3.UnitY, -Vector3.UnitY),
    ("zero normal fallback", Vector3.Zero, Vector3.Zero),
})
{
    Test($"StepPoint/{name}: finite C1 trajectory and zero endpoint velocity", () =>
    {
        Vector3 start = new(-0.5f, 0, 0.2f), end = new(1.5f, 0.4f, -0.3f);
        Vector3 Sample(float t) => StepPoint(start, end, from, to, 0.7f, t);
        const float h = 1f / 512f;
        Near(Sample(0), start, 1e-6f, "start");
        Near(Sample(1), end, 1e-6f, "end");
        Near((Sample(h) - Sample(0)) / h, Vector3.Zero, 0.03f, "lift-off velocity");
        Near((Sample(1) - Sample(1 - h)) / h, Vector3.Zero, 0.03f, "touchdown velocity");
        Vector3 previous = Sample(0);
        Vector3 previousVelocity = (Sample(h) - previous) / h;
        for (int i = 1; i <= 512; i++)
        {
            Vector3 p = Sample(i * h);
            FiniteVector(p, $"sample {i}");
            Vector3 velocity = (p - previous) / h;
            Check(velocity.Length() < 10f, $"position discontinuity at sample {i}: velocity {velocity}");
            Check(Vector3.Distance(velocity, previousVelocity) < 0.25f, $"velocity discontinuity at sample {i}");
            previous = p;
            previousVelocity = velocity;
        }
    });
}

Test("StepPoint/zero lift and stationary contact", () =>
{
    Vector3 start = new(1, 2, 3), end = new(4, 2, 3);
    for (int i = 0; i <= 32; i++)
    {
        float t = i / 32f;
        Vector3 p = StepPoint(start, end, Vector3.UnitY, Vector3.UnitX, 0, t);
        Near(p.Y, start.Y, 1e-6f, "zero lift stays on segment");
        Near(p.Z, start.Z, 1e-6f, "zero lift stays on segment");
        Near(StepPoint(start, start, Vector3.UnitY, Vector3.UnitY, 0, t), start, 1e-6f, "stationary contact");
    }
});

foreach (var (name, from, preferred) in new (string, Vector3, Vector3)[]
{
    ("preferred axis", Vector3.UnitY, Vector3.UnitX),
    ("parallel preferred axis", Vector3.UnitY, Vector3.UnitY),
    ("zero preferred axis", Vector3.UnitX, Vector3.Zero),
    ("non-unit vectors", new(2, -3, 4), new(1, 2, 3)),
})
{
    Test($"RotationBetween/opposites: {name}", () =>
    {
        Quaternion rotation = RotationBetween(from, -from, preferred);
        UnitQuaternion(rotation, "opposite rotation");
        Near(Vector3.Transform(Vector3.Normalize(from), rotation), -Vector3.Normalize(from), 2e-5f, "must map to opposite vector");
        if (Vector3.Cross(from, preferred).LengthSquared() > 1e-6f)
        {
            Vector3 projected = Vector3.Normalize(preferred - Vector3.Normalize(from) * Vector3.Dot(preferred, Vector3.Normalize(from)));
            Vector3 axis = Vector3.Normalize(new Vector3(rotation.X, rotation.Y, rotation.Z));
            Near(MathF.Abs(Vector3.Dot(axis, projected)), 1f, 2e-5f, "honor projected preferred axis");
        }
    });
}

Test("RotationBetween/identity, quarter turn and near-opposite stability", () =>
{
    Near(Vector3.Transform(Vector3.UnitX, RotationBetween(Vector3.UnitX, Vector3.UnitX, Vector3.Zero)), Vector3.UnitX, 1e-6f, "identity");
    Near(Vector3.Transform(Vector3.UnitX, RotationBetween(Vector3.UnitX, Vector3.UnitZ, Vector3.UnitY)), Vector3.UnitZ, 1e-6f, "quarter turn");
    foreach (float offset in new[] { -0.01f, -0.001f, 0f, 0.001f, 0.01f })
    {
        Vector3 target = Vector3.Normalize(new Vector3(offset, -1, 0));
        Quaternion rotation = RotationBetween(Vector3.UnitY, target, Vector3.UnitZ);
        UnitQuaternion(rotation, "near-opposite rotation");
        // The antiparallel branch deliberately chooses a preferred axis near 180 degrees.
        Near(Vector3.Transform(Vector3.UnitY, rotation), target, 0.003f, $"near-opposite offset {offset}");
    }
});

Test("AngularVelocity/quaternion signs do not change world-space velocity", () =>
{
    Quaternion previous = Quaternion.CreateFromYawPitchRoll(0.7f, -0.4f, 0.2f);
    Vector3 axis = Vector3.Normalize(new Vector3(1, -2, 3));
    const float angle = 0.36f, dt = 0.12f;
    Quaternion current = Quaternion.CreateFromAxisAngle(axis, angle) * previous;
    foreach (float previousSign in new[] { -1f, 1f })
        foreach (float currentSign in new[] { -1f, 1f })
            Near(AngularVelocity(previous * previousSign, current * currentSign, dt), axis * (angle / dt), 5e-6f, $"signs {previousSign}, {currentSign}");
    Near(AngularVelocity(previous, previous, dt), Vector3.Zero, 1e-6f, "unchanged orientation");
    Near(AngularVelocity(previous, -previous, dt), Vector3.Zero, 1e-6f, "sign-only change");
});

Test("AngularVelocity/shortest arc, tiny motion and nonpositive dt", () =>
{
    Quaternion longArc = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, float.DegreesToRadians(270));
    Near(AngularVelocity(Quaternion.Identity, longArc, 0.5f), -Vector3.UnitZ * MathF.PI, 2e-6f, "270 degrees uses -90 degree arc");
    Quaternion halfTurn = Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI);
    Near(AngularVelocity(Quaternion.Identity, halfTurn, 0.5f).Length(), MathF.Tau, 2e-6f, "half-turn speed (axis is ambiguous at exactly pi)");
    Quaternion tiny = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 1e-7f);
    Near(AngularVelocity(Quaternion.Identity, tiny, 0.1f), Vector3.Zero, 1e-6f, "tiny motion threshold");
    foreach (float dt in new[] { 0f, -1f, 1e-6f })
        Near(AngularVelocity(Quaternion.Identity, longArc, dt), Vector3.Zero, 0, $"dt {dt}");
});

foreach (bool distal in new[] { false, true })
{
    string kind = distal ? "three segments" : "two segments";
    Test($"IK/{kind}: calibrated lengths and reachable contacts", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), new SpiderJointLimits());
        Near(ik.L1, 1, 1e-6f, "L1");
        Near(ik.L2, 1, 1e-6f, "L2");
        Near(ik.L3, distal ? 0.5f : 0, 1e-6f, "L3");
        Near(ik.Reach, distal ? 2.5f : 2, 1e-6f, "reach");
        SpiderLegPose previous = ik.Rest;
        foreach (Vector3 offset in new Vector3[] { Vector3.Zero, new(0.15f, -0.1f, 0.12f), new(-0.15f, 0.1f, -0.12f) })
        {
            Vector3 target = ik.Rest.Tip + offset;
            Check(ik.TrySolve(target, Vector3.UnitY, previous, out var pose), $"reachable contact {target}");
            VerifyPose(ik, pose, target, Vector3.UnitY);
            previous = pose;
        }
    });

    Test($"IK/{kind}: far unreachable contact is rejected, never clamped", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), new SpiderJointLimits());
        foreach (Vector3 direction in new[] { Vector3.UnitX, -Vector3.UnitY, Vector3.Normalize(new Vector3(1, 2, 3)) })
        {
            Vector3 target = ik.Rest.Hip + direction * (ik.Reach + 0.1f);
            int calls = 0;
            Check(!ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var rejected, _ => { calls++; return true; }),
                $"out-of-reach target {target}");
            Check(calls == 0, "unreachable targets must not invoke validation");
            Check(rejected == default, "failed solve must leave the result at default");
        }
    });

    Test($"IK/{kind}: small continuous target motion", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), new SpiderJointLimits());
        SpiderLegPose previous = ik.Rest;
        for (int i = 0; i <= 360; i++)
        {
            float phase = MathF.Tau * i / 360f;
            Vector3 target = ik.Rest.Tip + new Vector3(0.12f * MathF.Sin(phase), 0.06f * MathF.Sin(phase * 2), 0.08f * MathF.Sin(phase));
            Check(ik.TrySolve(target, Vector3.UnitY, previous, out var pose), $"continuous path sample {i}, target {target}");
            VerifyPose(ik, pose, target, Vector3.UnitY);
            Near(pose.Knee, previous.Knee, 0.025f, $"knee continuity at {i}");
            Near(pose.Ankle, previous.Ankle, 0.025f, $"ankle continuity at {i}");
            previous = pose;
        }
    });

    Test($"IK/{kind}: first-best acceptance invokes validation only once", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), new SpiderJointLimits());
        SpiderLegPose previous = ik.Rest;
        foreach (Vector3 offset in new Vector3[] { Vector3.Zero, new(0.15f, -0.1f, 0.12f), new(-0.15f, 0.1f, -0.12f) })
        {
            Vector3 target = ik.Rest.Tip + offset;
            Check(ik.TrySolve(target, Vector3.UnitY, previous, out var best), "unfiltered best solution");
            int calls = 0;
            bool Accept(SpiderLegPose pose)
            {
                calls++;
                VerifyPose(ik, pose, target, Vector3.UnitY);
                Check(pose == best, "first validation must receive the unfiltered best pose");
                return true;
            }
            Check(ik.TrySolve(target, Vector3.UnitY, previous, out var accepted, Accept), "first-best accepted");
            Check(calls == 1, $"expected exactly one validation, got {calls}");
            Check(accepted == best, "callback must not change the selected best pose");
            previous = accepted;
        }
    });

    Test($"IK/{kind}: ordered validation exhausts all fallbacks without a budget", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), WideLimits());
        Vector3 target = ik.Rest.Tip + new Vector3(0.08f, -0.04f, 0.06f);
        var candidates = new List<SpiderLegPose>();
        float previousScore = -1f;
        bool Reject(SpiderLegPose pose)
        {
            VerifyPose(ik, pose, target, Vector3.UnitY);
            float score = Vector3.DistanceSquared(pose.Knee, ik.Rest.Knee) +
                Vector3.DistanceSquared(pose.Ankle, ik.Rest.Ankle) +
                0.05f * Vector3.DistanceSquared(pose.Knee, ik.Rest.Knee);
            Check(score >= previousScore, $"validation scores must be nondecreasing: {score} after {previousScore}");
            previousScore = score;
            candidates.Add(pose);
            return false;
        }
        Check(!ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var rejected, Reject), "all candidates rejected");
        Check(rejected == default, "rejection must not return an unapproved pose");
        Check(candidates.Count == (distal ? 48 : 6), $"all geometric candidates must reach validation, got {candidates.Count}");
        for (int approvedIndex = 0; approvedIndex < candidates.Count; approvedIndex++)
        {
            int calls = 0;
            bool AcceptFallback(SpiderLegPose pose)
            {
                Check(pose == candidates[calls], $"stable validation order at candidate {calls}");
                return calls++ == approvedIndex;
            }
            Check(ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var accepted, AcceptFallback),
                $"fallback {approvedIndex} must remain available");
            Check(calls == approvedIndex + 1, "validation must stop at the first approval");
            Check(accepted == candidates[approvedIndex], "return the first approved fallback");
        }
        Check(ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var best), "unfiltered best solution");
        Check(best == candidates[0], "unfiltered result must have the lowest continuity score");
    });

    Test($"IK/{kind}: planted contact does not drift or invalidate itself", () =>
    {
        var ik = new SpiderLegKinematics(SyntheticRest(distal), new SpiderJointLimits());
        Vector3 target = ik.Rest.Tip + new Vector3(0.08f, -0.04f, 0.06f);
        Check(ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var planted), "initial planted solution");
        SpiderLegPose previous = planted;
        for (int i = 0; i < 300; i++)
        {
            Check(ik.TrySolve(target, Vector3.UnitY, previous, out var pose), $"planted frame {i}");
            VerifyPose(ik, pose, target, Vector3.UnitY);
            Near(pose.Knee, planted.Knee, 2e-4f, $"cumulative knee drift at {i}");
            Near(pose.Ankle, planted.Ankle, 2e-4f, $"cumulative ankle drift at {i}");
            previous = pose;
        }
    });
}

Test("IK/translation, yaw and mirrored synthetic calibrations", () =>
{
    SpiderLegPose rest = SyntheticRest(true);
    foreach (float mirror in new[] { -1f, 1f })
        foreach (float yaw in new[] { -1.1f, 0f, 0.8f })
        {
            Quaternion rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
            Vector3 Map(Vector3 p) => Vector3.Transform(new Vector3(p.X * mirror, p.Y, p.Z), rotation) + new Vector3(3, -2, 4);
            var transformed = new SpiderLegPose(Map(rest.Hip), Map(rest.Knee), Map(rest.Ankle), Map(rest.Tip));
            var ik = new SpiderLegKinematics(transformed, new SpiderJointLimits());
            Vector3 target = Map(rest.Tip + new Vector3(0.1f, -0.05f, 0.05f));
            Check(ik.TrySolve(target, Vector3.UnitY, transformed, out var pose), $"mirrored {mirror}, yaw {yaw}");
            VerifyPose(ik, pose, target, Vector3.UnitY);
        }
});

Test("IK/two-segment inner and outer singularities are rejected", () =>
{
    var unequal = new SpiderLegPose(Vector3.Zero, new(0.6f, 0.8f, 0), new(0.9f, 0.4f, 0), new(0.9f, 0.4f, 0));
    var ik = new SpiderLegKinematics(unequal, WideLimits());
    foreach (float distance in new[] { 0f, 0.25f, 0.5f, 1.5f, 2f })
        Check(!ik.TrySolve(new(distance, 0, 0), Vector3.UnitY, unequal, out _), $"singular/unreachable distance {distance}");
    Check(ik.TrySolve(new(0.9f, 0.2f, 0), Vector3.UnitY, unequal, out var pose), "between inner and outer radii");
    VerifyPose(ik, pose, new(0.9f, 0.2f, 0), Vector3.UnitY);
});

foreach (var (name, limits, offset) in new (string, SpiderJointLimits, Vector3)[]
{
    ("minimum knee bend", new() { MinimumKneeBendDegrees = 150, MaximumKneeBendDegrees = 165 }, Vector3.Zero),
    ("maximum knee bend", new() { MinimumKneeBendDegrees = 0, MaximumKneeBendDegrees = 20 }, Vector3.Zero),
    ("hip swing", new() { HipSwingDegrees = 0 }, new(0, 0, 0.5f)),
    ("hip yaw", new() { HipYawDegrees = 0 }, new(0, 0, 0.5f)),
    ("ankle bend", new() { MaximumAnkleBendDegrees = 0 }, Vector3.Zero),
})
{
    Test($"IK/limits reject otherwise reachable contact: {name}", () =>
    {
        SpiderLegPose rest = SyntheticRest(true);
        Vector3 target = rest.Tip + offset;
        var wide = new SpiderLegKinematics(rest, WideLimits());
        Check(wide.TrySolve(target, Vector3.UnitY, rest, out var reachable), "control: contact is geometrically reachable");
        VerifyPose(wide, reachable, target, Vector3.UnitY);
        var restricted = new SpiderLegKinematics(rest, limits);
        Check(!restricted.TrySolve(target, Vector3.UnitY, rest, out _), $"must respect {name}");
    });
}

Test("IK/zero foot pitch and knee plane remain enforced", () =>
{
    var ik = new SpiderLegKinematics(SyntheticRest(true), new SpiderJointLimits { FootPitchDegrees = 0, KneePlaneDegrees = 0 });
    Vector3 target = ik.Rest.Tip + new Vector3(0.1f, -0.04f, 0);
    Check(ik.TrySolve(target, Vector3.UnitY, ik.Rest, out var pose), "reachable with locked pitch and plane");
    VerifyPose(ik, pose, target, Vector3.UnitY);
    Near(Vector3.Normalize(pose.Tip - pose.Ankle), -Vector3.UnitY, 2e-5f, "distal segment normal alignment");
});

Test("IK/previous knee pole outside tightened plane limit is not reused", () =>
{
    SpiderLegPose rest = SyntheticRest(true);
    Quaternion swivel = Quaternion.CreateFromAxisAngle(Vector3.UnitX, float.DegreesToRadians(20));
    SpiderLegPose previous = rest with { Knee = rest.Hip + Vector3.Transform(rest.Knee - rest.Hip, swivel) };
    VerifyPose(new SpiderLegKinematics(rest, WideLimits()), previous, rest.Tip, Vector3.UnitY);
    var ik = new SpiderLegKinematics(rest, new SpiderJointLimits { KneePlaneDegrees = 0 });
    Check(ik.TrySolve(rest.Tip, Vector3.UnitY, previous, out var pose), "recover a valid knee plane");
    VerifyPose(ik, pose, rest.Tip, Vector3.UnitY);
    Check(Vector3.Distance(pose.Knee, previous.Knee) > 0.01f, "out-of-plane previous knee must be corrected");
});

Test("IK/previous foot pitch outside tightened limit is not reused", () =>
{
    SpiderLegPose rest = SyntheticRest(true);
    float pitch = float.DegreesToRadians(20);
    Vector3 ankle = rest.Tip - new Vector3(MathF.Sin(pitch), -MathF.Cos(pitch), 0) * 0.5f;
    Vector3 offset = ankle - rest.Hip;
    float distance = offset.Length();
    Vector3 axis = offset / distance;
    Vector3 pole = Vector3.Normalize(Vector3.UnitY - axis * Vector3.Dot(Vector3.UnitY, axis));
    float along = distance * 0.5f; // Synthetic proximal segments both have length one.
    Vector3 knee = rest.Hip + axis * along + pole * MathF.Sqrt(1f - along * along);
    var previous = new SpiderLegPose(rest.Hip, knee, ankle, rest.Tip);
    VerifyPose(new SpiderLegKinematics(rest, WideLimits()), previous, rest.Tip, Vector3.UnitY);
    var ik = new SpiderLegKinematics(rest, new SpiderJointLimits { FootPitchDegrees = 0 });
    Check(ik.TrySolve(rest.Tip, Vector3.UnitY, previous, out var pose), "recover a valid foot pitch");
    VerifyPose(ik, pose, rest.Tip, Vector3.UnitY);
    Near(Vector3.Normalize(pose.Tip - pose.Ankle), -Vector3.UnitY, 2e-5f, "correct previous foot pitch");
});

Test("IK/contact acceptance callback is honored", () =>
{
    var ik = new SpiderLegKinematics(SyntheticRest(true), new SpiderJointLimits());
    int calls = 0;
    bool Reject(SpiderLegPose pose) { calls++; return false; }
    Check(!ik.TrySolve(ik.Rest.Tip, Vector3.UnitY, ik.Rest, out _, Reject), "all candidates rejected");
    Check(calls > 0, "callback must see valid candidates");
    calls = 0;
    bool Accept(SpiderLegPose pose)
    {
        calls++;
        VerifyPose(ik, pose, ik.Rest.Tip, Vector3.UnitY);
        return true;
    }
    Check(ik.TrySolve(ik.Rest.Tip, Vector3.UnitY, ik.Rest, out var accepted, Accept), "accept valid contact");
    Check(calls == 1, "best accepted candidate requires exactly one callback");
    VerifyPose(ik, accepted, ik.Rest.Tip, Vector3.UnitY);
    bool AlternateContact(SpiderLegPose pose) => pose.Tip.X - pose.Ankle.X > 0.1f;
    Check(ik.TrySolve(ik.Rest.Tip, Vector3.UnitY, ik.Rest, out var alternative, AlternateContact), "find an alternative after rejecting the preferred contact");
    Check(AlternateContact(alternative), "returned candidate must satisfy the filter");
    VerifyPose(ik, alternative, ik.Rest.Tip, Vector3.UnitY);
});

Test("IK/nonfinite inputs and zero-length proximal segments are rejected", () =>
{
    var ik = new SpiderLegKinematics(SyntheticRest(true), new SpiderJointLimits());
    foreach (Vector3 invalid in new[] { new Vector3(float.NaN, 0, 0), new Vector3(0, float.PositiveInfinity, 0), new Vector3(0, 0, float.NegativeInfinity) })
    {
        Check(!ik.TrySolve(invalid, Vector3.UnitY, ik.Rest, out _), "nonfinite target");
        Check(!ik.TrySolve(ik.Rest.Tip, invalid, ik.Rest, out _), "nonfinite normal");
    }
    SpiderLegPose rest = ik.Rest;
    foreach (SpiderLegPose degenerate in new[] { rest with { Knee = rest.Hip }, rest with { Ankle = rest.Knee } })
    {
        var invalid = new SpiderLegKinematics(degenerate, new SpiderJointLimits());
        Check(!invalid.TrySolve(rest.Tip, Vector3.UnitY, degenerate, out _), "zero-length L1 or L2");
    }
});

int failures = 0;
foreach (var (name, body) in tests)
{
    try
    {
        body();
        Console.WriteLine($"PASS {name}");
    }
    catch (Exception error)
    {
        failures++;
        Console.Error.WriteLine($"FAIL {name}\n  {error.Message}");
    }
}
Console.WriteLine($"\n{tests.Count - failures}/{tests.Count} passed; {failures} failed.");
return failures == 0 ? 0 : 1;

static SpiderLegPose SyntheticRest(bool distal)
{
    Vector3 ankle = new(1.2f, 1, 0);
    return new(new(0, 1, 0), new(0.6f, 1.8f, 0), ankle, distal ? ankle - Vector3.UnitY * 0.5f : ankle);
}

static SpiderJointLimits WideLimits() => new()
{
    HipSwingDegrees = 180, HipYawDegrees = 180,
    MinimumKneeBendDegrees = 0, MaximumKneeBendDegrees = 180,
    MaximumAnkleBendDegrees = 180, KneePlaneDegrees = 180, FootPitchDegrees = 85,
};

// Independent geometric checks: do not call the solver's private limit predicate or math Angle/Normal.
static void VerifyPose(SpiderLegKinematics ik, SpiderLegPose pose, Vector3 target, Vector3 normal)
{
    const float lengthTolerance = 3e-5f, angleTolerance = 0.003f;
    for (int i = 0; i < 4; i++) FiniteVector(pose[i], $"joint {i}");
    Near(pose.Hip, ik.Rest.Hip, lengthTolerance, "anchored hip");
    Near(pose.Tip, target, 2e-6f, "exact contact, not a clamped substitute");
    Near(Vector3.Distance(pose.Hip, pose.Knee), ik.L1, lengthTolerance, "upper length");
    Near(Vector3.Distance(pose.Knee, pose.Ankle), ik.L2, lengthTolerance, "lower length");
    Near(Vector3.Distance(pose.Ankle, pose.Tip), ik.L3, lengthTolerance, "distal length");
    Vector3 upper = pose.Knee - pose.Hip, lower = pose.Ankle - pose.Knee;
    SpiderJointLimits limits = ik.Limits;
    float knee = Degrees(upper, lower);
    Check(knee >= limits.MinimumKneeBendDegrees - angleTolerance && knee <= limits.MaximumKneeBendDegrees + angleTolerance,
        $"knee bend {knee} outside [{limits.MinimumKneeBendDegrees}, {limits.MaximumKneeBendDegrees}]");
    Check(Degrees(upper, ik.Rest.Knee - ik.Rest.Hip) <= limits.HipSwingDegrees + angleTolerance, "hip swing limit");
    Vector3 horizontal = new(upper.X, 0, upper.Z);
    Vector3 restUpper = ik.Rest.Knee - ik.Rest.Hip;
    Check(Degrees(horizontal, new(restUpper.X, 0, restUpper.Z)) <= limits.HipYawDegrees + angleTolerance, "hip yaw limit");
    Vector3 restAxis = Vector3.Normalize(ik.Rest.Ankle - ik.Rest.Hip);
    Vector3 restPole = Vector3.Normalize(restUpper - restAxis * Vector3.Dot(restUpper, restAxis));
    Vector3 axis = Vector3.Normalize(pose.Ankle - pose.Hip);
    Vector3 referencePole = restPole - axis * Vector3.Dot(restPole, axis);
    Vector3 pole = upper - axis * Vector3.Dot(upper, axis);
    Check(Degrees(pole, referencePole) <= limits.KneePlaneDegrees + angleTolerance, "knee plane limit");
    if (ik.L3 > 1e-5f)
    {
        Vector3 distal = pose.Tip - pose.Ankle;
        Check(Degrees(lower, distal) <= limits.MaximumAnkleBendDegrees + angleTolerance, "ankle bend limit");
        Check(Degrees(distal, -normal) <= limits.FootPitchDegrees + angleTolerance, "foot pitch limit");
    }
}

static float Degrees(Vector3 a, Vector3 b)
{
    Check(a.LengthSquared() > 1e-12f && b.LengthSquared() > 1e-12f, "angle requires nondegenerate vectors in synthetic fixture");
    return float.RadiansToDegrees(MathF.Atan2(Vector3.Cross(a, b).Length(), Vector3.Dot(a, b)));
}

internal static class Assertions
{
    public static void UnitQuaternion(Quaternion q, string context)
    {
        Check(float.IsFinite(q.X) && float.IsFinite(q.Y) && float.IsFinite(q.Z) && float.IsFinite(q.W), $"{context}: nonfinite quaternion {q}");
        Near(q.Length(), 1f, 2e-6f, $"{context}: unit quaternion");
    }

    public static void FiniteVector(Vector3 v, string context) =>
        Check(float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z), $"{context}: nonfinite vector {v}");

    public static void Near(Vector3 actual, Vector3 expected, float tolerance, string context)
    {
        FiniteVector(actual, context);
        float error = Vector3.Distance(actual, expected);
        Check(error <= tolerance, $"{context}: expected {expected}, got {actual}, error {error:G6} > {tolerance:G6}");
    }

    public static void Near(float actual, float expected, float tolerance, string context) =>
        Check(float.IsFinite(actual) && MathF.Abs(actual - expected) <= tolerance,
            $"{context}: expected {expected:G9}, got {actual:G9}, tolerance {tolerance:G6}");

    public static void Check(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
