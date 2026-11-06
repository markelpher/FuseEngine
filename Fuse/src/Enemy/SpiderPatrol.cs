using System;
using System.Numerics;
using Fuse.Behaviours;
using Fuse.Debug;
using Fuse.Physics;
using JoltPhysicsSharp;

namespace Fuse.Enemy;

public enum SpiderAiState
{
    Resting, Patrolling, Pursuing, Navigating, Arrived, Recovering, WaitingForSurface, Disabled
}

/// <summary>
/// High-level wandering intent for the spider. This class never changes the
/// rigid body directly; SpiderSurfaceMotor owns every physical displacement.
/// </summary>
public sealed class SpiderPatrol : IGizmoDrawable
{
    public static bool Enabled = true;
    public static bool PursuitEnabled { get; private set; }

    private static Vector3 s_pursuitTarget;
    private static bool s_hasPursuitTarget;
    private static Vector3 s_pursuitTargetSurfacePoint;
    private static Vector3 s_pursuitTargetSurfaceNormal;
    private static BodyID s_pursuitTargetSurfaceBodyId = BodyID.Invalid;
    private static bool s_hasPursuitTargetSurface;

    private enum PatrolState { Idle, Walking }

    private readonly SpiderEnemy _enemy;
    private readonly PhysicsWorld _physics;
    private readonly SpiderSurfaceMotor _motor;
    private readonly SpiderSurfacePursuitPlanner _pursuitPlanner;
    private PatrolState _state = PatrolState.Idle;
    private Vector3 _travelDirection = Vector3.Zero;
    private Vector3 _targetPosition;
    private float _remainingTravel;
    // Keep movement intent independent of speed constrained by the motor/legs.
    private float _commandSpeed;
    private float _waitTimer;
    private float _waitDuration;
    private float _blockedTimer;
    private float _pursuitBlockedTimer;
    private bool _initialized;
    private bool _wasPursuing;
    private bool _pursuitArrived;
    private Vector3 _patrolOrigin;

    public SpiderAiState State { get; private set; } = SpiderAiState.Resting;
    public Vector3 NavigationTarget => _targetPosition;
    public float CurrentSpeed { get; private set; }
    public Vector3 CurrentVelocity { get; private set; }
    public Vector3 CurrentPosition => _enemy.Body.IsBuilt
        ? _enemy.Body.Position(_physics)
        : Vector3.Zero;
    public Vector3 SurfaceNormal => _motor.SurfaceNormal;
    public SpiderSurfaceContact SurfaceContact => _motor.SurfaceContact;
    public bool HasSurface => _motor.HasSurface;
    public bool IsBlocked => _motor.IsBlocked;
    public float Clearance => _motor.Clearance;
    public bool IsMovementReady => !_enemy.IsDead && _enemy.Body.IsBuilt;
    public static Vector3 PursuitTarget => s_pursuitTarget;
    public static bool HasPursuitTarget => s_hasPursuitTarget;
    public static bool HasPursuitTargetSurface => s_hasPursuitTargetSurface;
    public static Vector3 PursuitTargetSurfacePoint => s_pursuitTargetSurfacePoint;
    public static Vector3 PursuitTargetSurfaceNormal => s_pursuitTargetSurfaceNormal;
    public static BodyID PursuitTargetSurfaceBodyId => s_pursuitTargetSurfaceBodyId;
    public SpiderSurfacePursuitPlanner PursuitPlanner => _pursuitPlanner;

    [Export] public float PatrolRadius { get; set; } = 20f;
    [Export] public float MoveSpeed { get; set; } = 9.5f;
    [Export] public float Acceleration { get; set; } = 8f;
    [Export] public float Deceleration { get; set; } = 10f;
    [Export] public float WalkAnimSpeed { get; set; } = 0.5f;
    [Export] public float MinWaitTime { get; set; } = 0.7f;
    [Export] public float MaxWaitTime { get; set; } = 2.0f;
    [Export] public float MinTravelDistance { get; set; } = 4f;
    [Export] public float PursuitStopDistance { get; set; } = 1.0f;
    [Export] public float PursuitResumeDistance { get; set; } = 1.5f;

    private static readonly Random s_random = new();

    public SpiderPatrol(
        SpiderEnemy enemy,
        PhysicsWorld physics,
        SpiderSurfaceMotor motor,
        SpiderSurfaceSolver surfaceSolver)
    {
        _enemy = enemy;
        _physics = physics;
        _motor = motor;
        _pursuitPlanner = new SpiderSurfacePursuitPlanner(surfaceSolver);
        DebugDrawer.Register(this);
    }

    public static void SetPursuitEnabled(bool enabled)
    {
        PursuitEnabled = enabled;
        // F3 is an explicit chase command, including after F6 paused the AI.
        if (enabled) Enabled = true;
    }

    public static void SetPursuitTarget(Vector3 target)
    {
        if (!IsFinite(target)) { ClearPursuitTarget(); return; }
        s_pursuitTarget = target;
        s_hasPursuitTarget = true;
    }

    public static void ClearPursuitTarget()
    {
        s_pursuitTarget = Vector3.Zero;
        s_hasPursuitTarget = false;
        ClearPursuitTargetSurface();
    }

    public static void SetPursuitTargetSurface(
        Vector3 surfacePoint,
        Vector3 surfaceNormal,
        BodyID surfaceBodyId = default)
    {
        surfaceNormal = NormalizeOrZero(surfaceNormal);
        if (surfaceNormal.LengthSquared() <= 0.0001f ||
            !IsFinite(surfacePoint))
        {
            ClearPursuitTargetSurface();
            return;
        }

        s_pursuitTargetSurfacePoint = surfacePoint;
        s_pursuitTargetSurfaceNormal = surfaceNormal;
        s_pursuitTargetSurfaceBodyId = surfaceBodyId;
        s_hasPursuitTargetSurface = true;
    }

    public static void ClearPursuitTargetSurface()
    {
        s_pursuitTargetSurfacePoint = Vector3.Zero;
        s_pursuitTargetSurfaceNormal = Vector3.Zero;
        s_pursuitTargetSurfaceBodyId = BodyID.Invalid;
        s_hasPursuitTargetSurface = false;
    }

    public void Update(float dt)
    {
        if (_enemy.IsDead || !_enemy.Body.IsBuilt)
            return;

        _motor.PrepareUpdate();
        if (!Enabled)
        {
            State = SpiderAiState.Disabled;
            _commandSpeed = 0f;
            _motor.Update(dt, _travelDirection, 0f);
            CurrentVelocity = _motor.CurrentVelocity;
            CurrentSpeed = _motor.CurrentSpeed;
            UpdateAnimationSpeed();
            return;
        }

        if (!_initialized)
        {
            _travelDirection = NormalizeOrFallback(_motor.MovementDirection, _motor.Forward);
            _waitDuration = RandomWaitTime();
            _targetPosition = _enemy.Body.Position(_physics);
            _patrolOrigin = _targetPosition;
            _initialized = true;
            PlayIdleAnimation();
        }

        dt = System.Math.Clamp(dt, 0.0001f, 0.05f);
        Vector3 positionBefore = _enemy.Body.Position(_physics);
        if (_wasPursuing != PursuitEnabled)
        {
            _wasPursuing = PursuitEnabled;
            _pursuitArrived = false;
            _pursuitBlockedTimer = _blockedTimer = _commandSpeed = 0f;
            _pursuitPlanner.ClearPlan();
            _motor.ClearTransitionConstraint();
            if (!PursuitEnabled)
            {
                _patrolOrigin = positionBefore;
                BeginIdle(0.15f, 0.35f);
            }
        }

        if (PursuitEnabled)
        {
            UpdatePursuit(dt, positionBefore);
        }
        else if (_state == PatrolState.Idle)
        {
            State = SpiderAiState.Resting;
            if (_pursuitPlanner.HasPlan) _pursuitPlanner.ClearPlan();
            _motor.ClearTransitionConstraint();
            _commandSpeed = MathF.Max(0f, _commandSpeed - Deceleration * dt);
            _motor.Update(dt, _travelDirection, 0f);
            _waitTimer += dt;

            if (_waitTimer >= _waitDuration && _motor.HasSurface)
                BeginWalking();
        }
        else
        {
            State = SpiderAiState.Patrolling;
            if (_pursuitPlanner.HasPlan) _pursuitPlanner.ClearPlan();
            _motor.ClearTransitionConstraint();
            float stoppingDistance = _commandSpeed * _commandSpeed / MathF.Max(0.01f, 2f * Deceleration);
            if (_remainingTravel <= stoppingDistance + 0.4f)
                _commandSpeed = MathF.Max(0f, _commandSpeed - Deceleration * dt);
            else
                _commandSpeed = MathF.Min(MoveSpeed, _commandSpeed + Acceleration * dt);

            _motor.Update(dt, _travelDirection, _commandSpeed);
            _travelDirection = NormalizeOrFallback(
                _motor.MovementDirection,
                ProjectOnPlane(_travelDirection, _motor.SurfaceNormal));

            Vector3 positionAfter = _enemy.Body.Position(_physics);
            float travelled = ProjectOnPlane(positionAfter - positionBefore, _motor.SurfaceNormal).Length();
            _remainingTravel = MathF.Max(0f, _remainingTravel - travelled);
            _targetPosition = positionAfter + _travelDirection * _remainingTravel;

            if (_motor.IsBlocked)
                _blockedTimer += dt;
            else
                _blockedTimer = MathF.Max(0f, _blockedTimer - dt * 2f);

            if (_remainingTravel <= 0.35f || _commandSpeed <= 0.05f && _remainingTravel < 0.8f)
            {
                BeginIdle();
            }
            else if (_blockedTimer > 0.35f)
            {
                BeginIdle(0.18f, 0.45f);
            }
        }

        CurrentVelocity = _motor.CurrentVelocity;
        CurrentSpeed = _motor.CurrentSpeed;
        UpdateAnimationSpeed();
    }

    private void UpdatePursuit(float dt, Vector3 currentPosition)
    {
        State = SpiderAiState.Pursuing;
        if (!HasPursuitTarget)
        {
            State = SpiderAiState.WaitingForSurface;
            _state = PatrolState.Idle;
            _commandSpeed = _pursuitBlockedTimer = 0f;
            _pursuitArrived = false;
            if (_pursuitPlanner.HasPlan) _pursuitPlanner.ClearPlan();
            _motor.ClearTransitionConstraint();
            _motor.Update(dt, _travelDirection, 0f);
            return;
        }
        Vector3 normal = NormalizeOrZero(_motor.SurfaceNormal);
        Vector3 targetSurfaceNormal = NormalizeOrZero(
            s_pursuitTargetSurfaceNormal);
        bool hasTargetSurface =
            s_hasPursuitTargetSurface &&
            targetSurfaceNormal.LengthSquared() > 0.0001f;

        Vector3 navigationTarget = PursuitTarget;
        if (hasTargetSurface)
        {
            // Project the player position onto the resolved target surface,
            // then lift it by this spider's clearance. The planner therefore
            // receives a position where the spider body can actually exist,
            // rather than a point inside or above the target geometry.
            Vector3 targetOnSurface = PursuitTarget -
                targetSurfaceNormal * Vector3.Dot(
                    PursuitTarget - s_pursuitTargetSurfacePoint,
                    targetSurfaceNormal);
            navigationTarget = targetOnSurface +
                               targetSurfaceNormal * _motor.Clearance;
        }

        Vector3 directionToTarget = navigationTarget - currentPosition;
        _targetPosition = navigationTarget;

        Vector3 surfaceDirection = NormalizeOrZero(
            ProjectOnPlane(directionToTarget, normal));

        // A normal alone does not identify a navigable surface. The lower
        // floor and an elevated floor can both have +Y as their normal. In
        // that case the spider must first commit to an intermediate face
        // instead of treating the elevated target as a direct ground target.
        bool targetSurfaceDifferent = hasTargetSurface &&
            IsDifferentSurface(
                _motor.SurfaceContact,
                normal,
                s_pursuitTargetSurfacePoint,
                targetSurfaceNormal,
                s_pursuitTargetSurfaceBodyId,
                _motor.Clearance);

        // A local plan only guides the spider to the next surface. Once that
        // surface is active, its old world-space direction is no longer valid.
        bool reachedPlannedSurface =
            !_motor.IsTransitioning &&
            _pursuitPlanner.IsPlannedSurfaceReached(
                normal,
                currentPosition);
        if (reachedPlannedSurface)
        {
            bool preserveDetourSide =
                _pursuitPlanner.IsSameSurfaceDetourActive;
            _pursuitPlanner.ClearPlan(preserveDetourSide);
            // Blocking accumulated while crossing the edge belongs to the old
            // surface. Carrying it onto the new surface immediately creates a
            // second, unrelated local plan instead of pursuing the player.
            _pursuitBlockedTimer = 0f;
        }

        if (_motor.IsTransitioning)
        {
            State = SpiderAiState.Navigating;
            bool hasPlannedWaypoint =
                _pursuitPlanner.HasPlan &&
                _pursuitPlanner.PlannedTransition.IsValid;
            if (_pursuitPlanner.HasPlan && !hasPlannedWaypoint)
            {
                // This was only a waypoint used to traverse the old surface.
                // The motor has already found a physical edge, so let the
                // motor finish that edge and choose the next waypoint after
                // the new surface is stable.
                _pursuitPlanner.ClearPlan();
            }

            // Once the motor has crossed the edge, its transported direction
            // is the source of truth. Re-projecting the planner's world-space
            // point onto the interpolated normal can point sideways/backwards
            // at a sharp corner and fight the transition. The planner remains
            // responsible for selecting the target surface; the motor owns
            // the continuous motion across that surface change.
            Vector3 transitionDirection = NormalizeOrZero(
                ProjectOnPlane(_motor.MovementDirection, normal));
            bool hasTransitionDirection =
                transitionDirection.LengthSquared() > 0.0001f;
            float transitionDistance = float.MaxValue;

            if (!hasTransitionDirection && hasPlannedWaypoint)
            {
                // Defensive fallback for a degenerate transported tangent.
                // This is only used when the motor has no valid direction of
                // its own; it does not normally steer an active transition.
                hasTransitionDirection =
                    _pursuitPlanner.TryGetSteeringDirection(
                        currentPosition,
                        normal,
                        out transitionDirection,
                        out transitionDistance);
            }

            _state = PatrolState.Walking;
            float transitionSpeedLimit = hasTransitionDirection
                ? MoveSpeed
                : 0f;
            UpdateSpeedTowardLimit(dt, transitionSpeedLimit);

            if (hasTransitionDirection)
                _travelDirection = transitionDirection;
            SyncTransitionConstraint();
            _motor.Update(dt, _travelDirection, _commandSpeed);
            UpdatePursuitBlockedState(dt);
            return;
        }

        float normalSeparation = normal.LengthSquared() > 0.0001f
            ? MathF.Abs(Vector3.Dot(directionToTarget, normal))
            : 0f;
        float targetOutsideSurfaceDistance = MathF.Max(
            _motor.Clearance * 0.2f,
            0.1f);
        bool targetOutsideCurrentSurface =
            targetSurfaceDifferent ||
            (hasTargetSurface &&
             Vector3.Dot(normal, targetSurfaceNormal) < 0.90f) ||
            (hasTargetSurface && normalSeparation > targetOutsideSurfaceDistance);
        bool directDirectionAvailable =
            surfaceDirection.LengthSquared() > 0.0001f;

        float targetDistance = ProjectOnPlane(directionToTarget, normal).Length();
        float stopDistance = MathF.Max(0.25f, PursuitStopDistance);
        float resumeDistance = MathF.Max(stopDistance + 0.25f, PursuitResumeDistance);
        if (!targetOutsideCurrentSurface &&
            targetDistance <= (_pursuitArrived ? resumeDistance : stopDistance) + 0.01f)
        {
            // Arrival is latched: small target/support changes must not make
            // the spider repeatedly overshoot, reverse and restart its gait.
            _pursuitArrived = true;
            State = SpiderAiState.Arrived;
            _state = PatrolState.Idle;
            _commandSpeed = _pursuitBlockedTimer = 0f;
            if (_pursuitPlanner.HasPlan) _pursuitPlanner.ClearPlan();
            _motor.SetTransitionConstraint(normal);
            _motor.Update(dt, _travelDirection, 0f);
            return;
        }
        _pursuitArrived = false;

        // A same-surface detour belongs only to the current floor/plane. If
        // the player changes to another surface while the spider is moving
        // around an obstacle, discard that local waypoint and let the normal
        // transition planner take over again.
        if (_pursuitPlanner.IsSameSurfaceDetourActive &&
            (!hasTargetSurface || targetOutsideCurrentSurface))
        {
            _pursuitPlanner.ClearPlan();
        }

        // The motor has a one-second transition lock to prevent oscillation at
        // corners. During that same interval the planner must not keep using a
        // stale route from the previous face. If the final target already has
        // a valid tangent on the new face, commit to that tangent: it points
        // upward for an elevated target and downward for a lower target.
        bool transitionExitLocked =
            _motor.TransitionLockRemaining > 0f &&
            targetOutsideCurrentSurface &&
            directDirectionAvailable;
        if (transitionExitLocked)
        {
            _pursuitPlanner.ClearPlan();
            _motor.ClearTransitionConstraint();
            _travelDirection = surfaceDirection;
        }

        bool localPlanRequired =
            !directDirectionAvailable ||
            targetOutsideCurrentSurface ||
            _pursuitBlockedTimer > 0.65f;
        if (transitionExitLocked)
            localPlanRequired = false;

        bool usingSameSurfaceDetour =
            hasTargetSurface &&
            !targetOutsideCurrentSurface &&
            directDirectionAvailable &&
            !_motor.IsTransitioning &&
            _pursuitPlanner.IsSameSurfaceDetourActive;

        if (!usingSameSurfaceDetour &&
            hasTargetSurface &&
            !targetOutsideCurrentSurface &&
            directDirectionAvailable &&
            !_motor.IsTransitioning)
        {
            usingSameSurfaceDetour =
                _pursuitPlanner.TryGetSameSurfaceDetour(
                    dt,
                    currentPosition,
                    normal,
                    surfaceDirection,
                    navigationTarget,
                    _motor.Clearance,
                    _enemy.Body.Native,
                    out _);
        }

        bool hasLocalDirection = _pursuitPlanner.TryGetDirection(
            dt,
            currentPosition,
            normal,
            _motor.MovementDirection,
            navigationTarget,
            _motor.Clearance,
            _enemy.Body.Native,
            localPlanRequired,
            out Vector3 localDirection,
            targetSurfaceNormal);

        bool usingLocalDirection = false;

        if (directDirectionAvailable &&
            (!targetOutsideCurrentSurface || transitionExitLocked) &&
            _pursuitBlockedTimer <= 0.65f &&
            !usingSameSurfaceDetour)
        {
            if (_pursuitPlanner.HasPlan) _pursuitPlanner.ClearPlan();
            _travelDirection = surfaceDirection;
        }
        else if (hasLocalDirection)
        {
            _travelDirection = localDirection;
            usingLocalDirection = true;
        }

        bool canUseDirectDirection =
            directDirectionAvailable &&
            (!targetOutsideCurrentSurface || transitionExitLocked);
        bool canMove = _pursuitPlanner.HasPlan
            ? hasLocalDirection
            : canUseDirectDirection;
        if (!canMove)
        {
            // Do not invent a random direction when the target is outside the
            // current surface. Stay attached and let the planner retry after
            // its cooldown.
            State = SpiderAiState.WaitingForSurface;
            _state = PatrolState.Idle;
            _commandSpeed = MathF.Max(
                0f,
                _commandSpeed - Deceleration * dt);
            SyncTransitionConstraint(
                currentPosition,
                normal,
                hasTargetSurface,
                s_pursuitTargetSurfacePoint,
                targetSurfaceNormal,
                s_pursuitTargetSurfaceBodyId);
            _motor.Update(dt, _travelDirection, 0f);
            UpdatePursuitBlockedState(dt);
            return;
        }

        if (!usingLocalDirection)
            _travelDirection = surfaceDirection;
        _state = PatrolState.Walking;

        State = usingLocalDirection ? SpiderAiState.Navigating : SpiderAiState.Pursuing;
        float speedLimit = targetOutsideCurrentSurface ? MoveSpeed : MathF.Min(MoveSpeed,
            MathF.Min(MathF.Sqrt(2f * MathF.Max(0.01f, Deceleration) * MathF.Max(0f, targetDistance - stopDistance)),
                MathF.Max(0f, targetDistance - stopDistance) / dt));
        if (usingLocalDirection &&
            _pursuitPlanner.TryGetSteeringDirection(
                currentPosition,
                normal,
                out Vector3 waypointDirection,
                out float waypointDistance))
        {
            _travelDirection = waypointDirection;
            speedLimit = MathF.Min(speedLimit, CalculateWaypointSpeedLimit(dt, waypointDistance));
            if (speedLimit <= 0.001f &&
                _motor.TransitionLockRemaining <= 0f &&
                _pursuitPlanner.IsOnPlannedSourceSurface(normal))
            {
                // The spider is parked at the edge. Once the one-second lock
                // expires, use a controlled creep so the motor can acquire the
                // already constrained target surface.
                speedLimit = MathF.Min(MoveSpeed, 0.65f);
            }
        }

        UpdateSpeedTowardLimit(dt, speedLimit);
        if (!transitionExitLocked)
        {
            SyncTransitionConstraint(
                currentPosition,
                normal,
                hasTargetSurface,
                s_pursuitTargetSurfacePoint,
                targetSurfaceNormal,
                s_pursuitTargetSurfaceBodyId);
        }
        _motor.Update(dt, _travelDirection, _commandSpeed);
        UpdatePursuitBlockedState(dt);
    }

    private void UpdatePursuitBlockedState(float dt)
    {
        // A brief gait veto or turn is not evidence of a bad route. Replan
        // only after sustained lack of translation while intending to advance.
        float alignment = Vector3.Dot(_motor.Forward, _travelDirection);
        if (_commandSpeed > 0.5f && _motor.CurrentSpeed < 0.15f && alignment > 0.7f)
            _pursuitBlockedTimer += dt;
        else
            _pursuitBlockedTimer = MathF.Max(
                0f,
                _pursuitBlockedTimer - dt * 2f);

        if (!_pursuitPlanner.HasPlan ||
            _motor.IsTransitioning ||
            _motor.TransitionLockRemaining > 0f ||
            _pursuitBlockedTimer < 0.90f)
        {
            return;
        }

        State = SpiderAiState.Recovering;
        _pursuitPlanner.AbandonPlan();
        _motor.ClearTransitionConstraint();
        _pursuitBlockedTimer = 0f;
        _commandSpeed = 0f;
    }

    private void SyncTransitionConstraint(
        Vector3 currentPosition = default,
        Vector3 currentNormal = default,
        bool hasTargetSurface = false,
        Vector3 targetSurfacePoint = default,
        Vector3 targetSurfaceNormal = default,
        BodyID targetSurfaceBodyId = default)
    {
        if (_pursuitPlanner.IsSameSurfaceDetourActive)
        {
            _motor.SetTransitionConstraint(_motor.SurfaceNormal);
            _motor.ClearTransitionGuide();
            return;
        }

        // Near the final surface, use the resolved target contact itself as a
        // deterministic guide. This is especially important for an elevated
        // floor: the wall transition should finish on that floor rather than
        // repeatedly selecting the underside or the opposite side of its edge.
        if (hasTargetSurface &&
            TryBuildTargetSurfaceGuide(
                currentPosition,
                currentNormal,
                targetSurfacePoint,
                targetSurfaceNormal,
                targetSurfaceBodyId,
                out SpiderSurfaceContact targetGuide))
        {
            _motor.SetTransitionConstraint(targetGuide.Normal);
            _motor.SetTransitionGuide(targetGuide);
            return;
        }

        SpiderSurfaceContact transition =
            _pursuitPlanner.PlannedTransition;
        if (_pursuitPlanner.HasPlan && transition.IsValid)
        {
            _motor.SetTransitionConstraint(transition.Normal);
            _motor.SetTransitionGuide(transition);
        }
        else
        {
            if (hasTargetSurface && !IsDifferentSurface(_motor.SurfaceContact, _motor.SurfaceNormal,
                    targetSurfacePoint, targetSurfaceNormal, targetSurfaceBodyId, _motor.Clearance))
                _motor.SetTransitionConstraint(_motor.SurfaceNormal);
            else
                _motor.ClearTransitionConstraint();
            _motor.ClearTransitionGuide();
        }
    }

    private bool TryBuildTargetSurfaceGuide(
        Vector3 currentPosition,
        Vector3 currentNormal,
        Vector3 targetSurfacePoint,
        Vector3 targetSurfaceNormal,
        BodyID targetSurfaceBodyId,
        out SpiderSurfaceContact guide)
    {
        guide = default;
        currentNormal = NormalizeOrZero(currentNormal);
        targetSurfaceNormal = NormalizeOrZero(targetSurfaceNormal);
        if (currentNormal.LengthSquared() <= 0.0001f ||
            targetSurfaceNormal.LengthSquared() <= 0.0001f ||
            !IsFinite(currentPosition) ||
            !IsFinite(targetSurfacePoint) ||
            targetSurfaceBodyId.IsValid == false)
        {
            return false;
        }

        // A target with the same normal is not a transition by itself. The
        // plane/body test above still keeps the spider from using a direct
        // ground vector, while this guide is reserved for the wall -> floor
        // handoff where the normals actually change.
        float normalAlignment = Vector3.Dot(
            currentNormal,
            targetSurfaceNormal);
        if (normalAlignment >= 0.82f || normalAlignment <= -0.42f)
            return false;

        Vector3 targetCenter = targetSurfacePoint +
                               targetSurfaceNormal * _motor.Clearance;
        float distance = Vector3.Distance(currentPosition, targetCenter);
        if (!float.IsFinite(distance) || distance > 2.25f)
            return false;

        // SpiderSurfaceMotor/SpiderSurfaceSolver revalidate this contact with
        // a cast from the target surface. The guide only supplies the known
        // body, point and normal; it never bypasses collision validation.
        guide = new SpiderSurfaceContact(
            true,
            targetSurfacePoint,
            targetSurfaceNormal,
            targetSurfaceBodyId,
            null,
            targetSurfacePoint,
            targetSurfaceNormal,
            1f);
        return true;
    }

    private float CalculateWaypointSpeedLimit(float dt, float distance)
    {
        if (!float.IsFinite(distance))
            return 0f;

        float reachDistance = MathF.Max(0.12f, _motor.Clearance * 0.22f);
        float remainingDistance = MathF.Max(0f, distance - reachDistance);
        if (remainingDistance <= 0.001f)
            return 0f;

        float stoppingSpeed = MathF.Sqrt(
            2f * MathF.Max(0.01f, Deceleration) * remainingDistance);
        float frameSpeed = remainingDistance / MathF.Max(0.0001f, dt);
        return MathF.Min(MoveSpeed, MathF.Min(stoppingSpeed, frameSpeed));
    }

    private void UpdateSpeedTowardLimit(float dt, float speedLimit)
    {
        speedLimit = System.Math.Clamp(speedLimit, 0f, MoveSpeed);
        if (_commandSpeed < speedLimit)
            _commandSpeed = MathF.Min(speedLimit, _commandSpeed + Acceleration * dt);
        else
            _commandSpeed = MathF.Max(speedLimit, _commandSpeed - Deceleration * dt);

        // Never allow one simulation step to pass beyond the active waypoint.
        _commandSpeed = MathF.Min(_commandSpeed, speedLimit);
    }

    private void BeginWalking()
    {
        Vector3 normal = _motor.SurfaceNormal;
        BuildTangentBasis(normal, _motor.MovementDirection, out Vector3 forward, out Vector3 right);

        float angle = (float)(s_random.NextDouble() * MathF.Tau);
        _travelDirection = NormalizeOrFallback(
            forward * MathF.Cos(angle) + right * MathF.Sin(angle),
            forward);

        float radius = MathF.Max(1f, PatrolRadius);
        Vector3 position = _enemy.Body.Position(_physics);
        Vector3 fromHome = ProjectOnPlane(position - _patrolOrigin, normal);
        if (fromHome.LengthSquared() > radius * radius * 0.49f)
            _travelDirection = NormalizeOrFallback(-fromHome, forward);
        float minimum = MathF.Min(MinTravelDistance, radius);
        float desiredTravel = minimum + (float)s_random.NextDouble() * MathF.Max(0.5f, radius - minimum);
        float along = Vector3.Dot(fromHome, _travelDirection);
        float boundary = -along + MathF.Sqrt(MathF.Max(0f, along * along + radius * radius - fromHome.LengthSquared()));
        _remainingTravel = MathF.Min(desiredTravel, MathF.Max(0.1f, boundary - 0.25f));
        _targetPosition = position + _travelDirection * _remainingTravel;
        _blockedTimer = 0f;
        _waitTimer = 0f;
        _state = PatrolState.Walking;
        PlayWalkAnimation();
    }

    private void BeginIdle(float? minimumWait = null, float? maximumWait = null)
    {
        _state = PatrolState.Idle;
        _commandSpeed = 0f;
        CurrentVelocity = Vector3.Zero;
        _waitTimer = 0f;
        _blockedTimer = 0f;
        _waitDuration = minimumWait.HasValue && maximumWait.HasValue
            ? minimumWait.Value + (float)s_random.NextDouble() * (maximumWait.Value - minimumWait.Value)
            : RandomWaitTime();
        _targetPosition = _enemy.Body.Position(_physics);
        PlayIdleAnimation();
    }

    private float RandomWaitTime() =>
        MinWaitTime + (float)s_random.NextDouble() * MathF.Max(0f, MaxWaitTime - MinWaitTime);

    private void UpdateAnimationSpeed()
    {
        var animator = _enemy.Entity?.Animator;
        if (animator != null)
        {
            if (_state == PatrolState.Walking) PlayWalkAnimation(); else PlayIdleAnimation();
            animator.Speed = _state == PatrolState.Idle ? 1f :
                MathF.Max(0.1f, MoveSpeed > 0.001f ? CurrentSpeed / MoveSpeed * WalkAnimSpeed : 0f);
        }
    }

    private void PlayIdleAnimation()
    {
        var animator = _enemy.Entity?.Animator;
        if (animator == null)
            return;
        PlayLocomotionClip("Idle");
    }

    private void PlayWalkAnimation()
    {
        PlayLocomotionClip("Walk");
    }

    private void PlayLocomotionClip(string name)
    {
        var animator = _enemy.Entity?.Animator;
        var model = animator?.Model;
        if (animator == null || model == null) return;
        string? clip = model.Clips.Keys.FirstOrDefault(k => k.Equals(name, StringComparison.OrdinalIgnoreCase)) ??
            model.Clips.Keys.FirstOrDefault(k => k.Contains(name, StringComparison.OrdinalIgnoreCase)) ?? model.DefaultClipName;
        animator.LoopOverride = true;
        if (!string.IsNullOrEmpty(clip)) animator.CrossFade(clip, 0.25f);
    }

    private static void BuildTangentBasis(
        Vector3 normal,
        Vector3 desiredForward,
        out Vector3 forward,
        out Vector3 right)
    {
        normal = NormalizeOrZero(normal);
        if (normal.LengthSquared() <= 0.0001f)
        {
            forward = Vector3.Zero;
            right = Vector3.Zero;
            return;
        }

        forward = NormalizeOrZero(ProjectOnPlane(desiredForward, normal));
        if (forward.LengthSquared() <= 0.0001f)
            forward = BuildFallbackTangent(normal, desiredForward);

        right = NormalizeOrZero(Vector3.Cross(forward, normal));
        forward = NormalizeOrZero(Vector3.Cross(normal, right));
    }

    private static Vector3 ProjectOnPlane(Vector3 value, Vector3 normal) =>
        value - normal * Vector3.Dot(value, normal);

    private static Vector3 NormalizeOrFallback(Vector3 value, Vector3 fallback)
    {
        if (value.LengthSquared() > 0.0001f)
            return Vector3.Normalize(value);
        if (fallback.LengthSquared() > 0.0001f)
            return Vector3.Normalize(fallback);
        return Vector3.Zero;
    }

    private static Vector3 NormalizeOrZero(Vector3 value) =>
        value.LengthSquared() > 0.0001f ? Vector3.Normalize(value) : Vector3.Zero;

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) &&
        float.IsFinite(value.Y) &&
        float.IsFinite(value.Z);

    private static bool IsDifferentSurface(
        in SpiderSurfaceContact currentContact,
        Vector3 currentNormal,
        Vector3 targetPoint,
        Vector3 targetNormal,
        BodyID targetBodyId,
        float clearance)
    {
        currentNormal = NormalizeOrZero(currentNormal);
        targetNormal = NormalizeOrZero(targetNormal);
        if (!currentContact.IsValid ||
            currentNormal.LengthSquared() <= 0.0001f ||
            targetNormal.LengthSquared() <= 0.0001f ||
            !IsFinite(currentContact.Point) ||
            !IsFinite(targetPoint))
        {
            return true;
        }

        if (Vector3.Dot(currentNormal, targetNormal) < 0.90f)
            return true;

        // A single mesh/body can contain several parallel floors. Compare the
        // signed distance between their support planes so an upper floor is
        // not mistaken for the floor under the spider.
        float planeSeparation = MathF.Abs(Vector3.Dot(
            targetPoint - currentContact.Point,
            currentNormal));
        float planeTolerance = MathF.Max(0.1f, clearance * 0.2f);
        if (!float.IsFinite(planeSeparation) ||
            planeSeparation > planeTolerance)
        {
            return true;
        }

        // BodyID is intentionally only a supporting identity signal. A level
        // floor is often made of several collision bodies, and those pieces
        // must remain one continuous surface when they share the same plane.
        // The motor/planner will still validate the actual crossing.
        _ = targetBodyId;
        return false;
    }

    private static Vector3 BuildFallbackTangent(Vector3 normal, Vector3 preferred)
    {
        normal = NormalizeOrZero(normal);
        if (normal.LengthSquared() <= 0.0001f)
            return Vector3.Zero;

        Vector3 tangent = NormalizeOrZero(ProjectOnPlane(preferred, normal));
        if (tangent.LengthSquared() > 0.0001f)
            return tangent;

        Vector3 reference = Vector3.UnitX;
        float smallestAlignment = MathF.Abs(Vector3.Dot(normal, reference));
        Vector3[] candidates = { Vector3.UnitY, Vector3.UnitZ };
        foreach (Vector3 candidate in candidates)
        {
            float alignment = MathF.Abs(Vector3.Dot(normal, candidate));
            if (alignment >= smallestAlignment)
                continue;
            reference = candidate;
            smallestAlignment = alignment;
        }

        return NormalizeOrZero(Vector3.Cross(normal, reference));
    }

    public void OnDrawGizmos(DebugDrawer drawer)
    {
        if (_enemy.IsDead || !_enemy.Body.IsBuilt)
            return;

        Vector3 position = _enemy.Body.Position(_physics);
        Vector3 normal = _motor.SurfaceNormal;
        BuildTangentBasis(normal, _travelDirection, out Vector3 forward, out Vector3 right);

        Vector3 stateColor = _state == PatrolState.Walking
            ? new Vector3(0.1f, 1f, 0.25f)
            : new Vector3(1f, 0.85f, 0.15f);

        drawer.DrawBox(_targetPosition, Quaternion.Identity, new Vector3(0.14f), new Vector3(1f, 0.2f, 0.2f));
        drawer.PushLine(position, _targetPosition, stateColor);
        drawer.PushLine(position, position + _travelDirection * 2f, new Vector3(0.1f, 0.55f, 1f));

        if (PursuitEnabled && s_hasPursuitTargetSurface &&
            IsFinite(s_pursuitTargetSurfacePoint) &&
            s_pursuitTargetSurfaceNormal.LengthSquared() > 0.0001f)
        {
            Vector3 targetNormal = NormalizeOrZero(
                s_pursuitTargetSurfaceNormal);
            Vector3 targetCenter = s_pursuitTargetSurfacePoint +
                                   targetNormal * _motor.Clearance;
            if (IsFinite(targetCenter))
            {
                // Magenta marks the actual resolved surface under/around the
                // player. The red box above is the body-centre navigation
                // target, while this point identifies the plane that must be
                // reached before direct pursuit is allowed.
                drawer.DrawSphere(
                    s_pursuitTargetSurfacePoint,
                    Quaternion.Identity,
                    0.16f,
                    new Vector3(1f, 0.1f, 0.85f));
                drawer.PushLine(
                    s_pursuitTargetSurfacePoint,
                    s_pursuitTargetSurfacePoint + targetNormal * 0.9f,
                    new Vector3(1f, 0.1f, 0.85f));
            }
        }

        const int segments = 32;
        Vector3 previous = position + forward * PatrolRadius;
        for (int i = 1; i <= segments; i++)
        {
            float angle = MathF.Tau * i / segments;
            Vector3 next = position + (forward * MathF.Cos(angle) + right * MathF.Sin(angle)) * PatrolRadius;
            drawer.PushLine(previous, next, new Vector3(0.35f, 0.35f, 0.1f));
            previous = next;
        }
    }
}
