using System.Collections.Generic;
using System.Numerics;
using Fuse.Physics;
using Fuse.Scene;
using Fuse.AssetManagement;
using Fuse.Renderer;
using JoltPhysicsSharp;


namespace Fuse.Enemy;

public sealed class EnemySystem : IDisposable
{
    private readonly PhysicsWorld _physics;
    private readonly Scene.SceneManager _sceneManager;
    private readonly AssetManager _assets;
    private readonly List<Enemy> _enemies = new();
    private readonly List<SpiderEnemy> _spiders = new();
    private readonly Dictionary<BodyID, SpiderEnemy> _spiderDamageBodies = new();
    private readonly HashSet<BodyID> _spiderMovementBodiesForRaycast = new();
    private readonly SpiderTargetSurfaceResolver _spiderTargetSurfaceResolver;
    private Player.Player? _player;
    private float _contactDamageCooldown;
    private const float PursuitProbeInterval = 0.1f;
    private const float PursuitSupportSearchDistance = 0.6f;
    private const float PursuitSurfaceRetention = 0.35f;
    private const float PursuitRetentionDistance = 1.5f;
    private Player.Player? _pursuitPlayer;
    private SpiderTargetSurface _pursuitSurface;
    private Vector3 _pursuitResolvedFeet;
    private float _pursuitProbeCooldown;
    private float _pursuitSurfaceAge;
    private bool _pursuitWasGrounded;

    public EnemySystem(PhysicsWorld physics, Scene.SceneManager sceneManager, AssetManager assets)
    {
        _physics = physics;
        _sceneManager = sceneManager;
        _assets = assets;
        _spiderTargetSurfaceResolver = new SpiderTargetSurfaceResolver(sceneManager);
    }

    public void SetPlayer(Player.Player player) => _player = player;

    public IReadOnlyList<Enemy> GetEnemies() => _enemies;
    public Enemy SpawnEnemy(Vector3 position, float health = 100f)
    {
        string id = $"enemy_{_enemies.Count}";
        var enemy = new Enemy(id, health, _assets);
        enemy.Initialize(_physics, _sceneManager, position);
        _enemies.Add(enemy);
        return enemy;
    }

    public IReadOnlyList<SpiderEnemy> GetSpiders() => _spiders;

    public SpiderEnemy? SpawnSpider(Vector3 position, float health = 80f, Vector3? surfaceNormal = null)
    {
        string id = $"spider_{_spiders.Count}";
        var spider = new SpiderEnemy(id, health, _assets);
        try
        {
            spider.Initialize(_physics, _sceneManager, position, surfaceNormal);
        }
        catch (InvalidOperationException exception) when (!spider.Body.IsBuilt)
        {
            spider.Dispose();
            Core.Logger.Warn($"Could not spawn spider: {exception.Message}");
            return null;
        }
        _spiders.Add(spider);
        RegisterSpiderDamageBodies(spider);
        return spider;
    }

    public void Update(float dt)
    {
        if (SpiderPatrol.PursuitEnabled && _player != null && !_player.IsDead &&
            float.IsFinite(_player.FeetPosition.LengthSquared()))
        {
            Vector3 feetPosition = _player.FeetPosition;
            CharacterVirtual character = _player.NativeCharacter;
            bool grounded = !_player.IsNoclip && character.GroundState == GroundState.OnGround;
            float elapsed = float.IsFinite(dt) && dt > 0f ? dt : 0f;
            bool displaced = _pursuitSurface.IsValid &&
                Vector3.DistanceSquared(feetPosition, _pursuitResolvedFeet) >
                PursuitRetentionDistance * PursuitRetentionDistance;
            bool removedSupport = _pursuitSurface.IsValid &&
                (!_physics.BodyInterface.IsAdded(_pursuitSurface.BodyId) ||
                 _sceneManager.NonWalkableBodies.Contains(_pursuitSurface.BodyId));
            if (!ReferenceEquals(_pursuitPlayer, _player) || displaced || removedSupport)
            {
                _pursuitPlayer = _player;
                _pursuitSurface = default;
                _pursuitProbeCooldown = 0f;
                _pursuitSurfaceAge = 0f;
                _pursuitWasGrounded = false;
            }

            _pursuitSurfaceAge += elapsed;
            _pursuitProbeCooldown -= elapsed;
            if (!_player.IsNoclip && (_pursuitProbeCooldown <= 0f || (grounded && !_pursuitWasGrounded)))
            {
                // One shared query at 10 Hz, at most six local rays (normally
                // one). Landing, target replacement and teleports query now.
                _pursuitProbeCooldown = PursuitProbeInterval;
                if (_spiderTargetSurfaceResolver.TryResolvePlayer(
                        feetPosition, character, PursuitSupportSearchDistance,
                        out SpiderTargetSurface targetSurface))
                {
                    _pursuitSurface = targetSurface;
                    _pursuitSurfaceAge = 0f;
                    _pursuitResolvedFeet = feetPosition;
                }
            }
            _pursuitWasGrounded = grounded;

            if (_pursuitSurface.IsValid && _pursuitSurfaceAge <= PursuitSurfaceRetention)
            {
                // Retain the physical point, not an unverified projection of
                // the jumping player's new position onto an old support plane.
                SpiderPatrol.SetPursuitTarget(_pursuitSurface.Point);
                SpiderPatrol.SetPursuitTargetSurface(
                    _pursuitSurface.Point, _pursuitSurface.Normal, _pursuitSurface.BodyId);
            }
            else
            {
                _pursuitSurface = default;
                SpiderPatrol.SetPursuitTarget(feetPosition);
                SpiderPatrol.ClearPursuitTargetSurface();
            }
        }
        else
        {
            _pursuitPlayer = null;
            _pursuitSurface = default;
            _pursuitProbeCooldown = 0f;
            _pursuitSurfaceAge = 0f;
            _pursuitWasGrounded = false;
            SpiderPatrol.ClearPursuitTarget();
            // Clearing only the surface leaves Patrol pursuing its static raw
            // target. An unavailable/dead player must stop pursuit altogether.
            SpiderPatrol.SetPursuitEnabled(false);
        }

        for (int i = _enemies.Count - 1; i >= 0; i--)
        {
            var e = _enemies[i];
            if (e.IsDead)
            {
                e.OnDeath(_physics, _sceneManager);
                e.Dispose();
                _enemies.RemoveAt(i);
            }
            else
            {
                e.Update(dt, _physics);
            }
        }

        for (int i = _spiders.Count - 1; i >= 0; i--)
        {
            var s = _spiders[i];

            if (s.IsDead)
            {
                UnregisterSpiderDamageBodies(s);
                if (s.Body.IsBuilt)
                    _sceneManager.ActiveScene.UnregisterBody(s.Entity);
                s.OnDeath(_physics, _sceneManager);
                s.Update(dt, _physics);
                
                if (s.CanBeRemoved)
                {
                    s.Dispose();
                    _sceneManager.ActiveScene.Remove(s.Entity);
                    _spiders.RemoveAt(i);
                }
            }
            else
            {
                s.Update(dt, _physics);
            }
        }
    }

    public void UpdateContactDamage(float dt)
    {
        if (_player == null || _player.IsDead) return;

        _contactDamageCooldown -= dt;
        if (_contactDamageCooldown > 0f) return;

        Vector3 playerPos = _player.Position;

        foreach (var e in _enemies)
        {
            if (e.IsDead || e.Character == null) continue;
            Vector3 enemyPos = e.Character.Position;
            float dist = Vector3.Distance(playerPos, enemyPos);
            if (dist < 1.8f)
            {
                Vector3 dir = Vector3.Normalize(playerPos - enemyPos);
                _player.TakeDamage(15f, dir);
                _contactDamageCooldown = 0.5f;
                return;
            }
        }

        foreach (var s in _spiders)
        {
            if (s.IsDead || !s.Body.IsBuilt) continue;
            Vector3 spiderPos = s.Body.Position(_physics);
            float dist = Vector3.Distance(playerPos, spiderPos);
            if (dist < 1.8f)
            {
                Vector3 dir = Vector3.Normalize(playerPos - spiderPos);
                _player.TakeDamage(15f, dir);
                _contactDamageCooldown = 0.5f;
                return;
            }
        }
    }

    public bool TryGetEnemy(BodyID bodyId, out IEnemy? enemy)
    {
        if (_spiderDamageBodies.TryGetValue(
                bodyId,
                out SpiderEnemy? damageSpider))
        {
            if (!damageSpider.IsDead)
            {
                enemy = damageSpider;
                return true;
            }

            // A dead spider destroys its live hit proxy on the next update.
            // Remove stale entries defensively in case a raycast happens in
            // between those two operations.
            _spiderDamageBodies.Remove(bodyId);
        }

        var entity = _sceneManager.ActiveScene.GetEntityByBody(bodyId);
        if (entity != null)
        {
            var e = _enemies.Find(e => e.Entity.Id == entity.Id);
            if (e != null) { enemy = e; return true; }
            var s = _spiders.Find(s => s.Entity.Id == entity.Id);
            if (s != null) { enemy = s; return true; }
        }
        enemy = null;
        return false;
    }

    /// <summary>
    /// A arma deve atingir as cápsulas articuladas do corpo, não a cápsula
    /// auxiliar do CharacterVirtual. Só exclui a cápsula principal quando a
    /// aranha realmente possui um proxy de dano ativo.
    /// </summary>
    public IReadOnlySet<BodyID> GetSpiderMovementBodiesForDamageRaycast()
    {
        _spiderMovementBodiesForRaycast.Clear();

        foreach (SpiderEnemy spider in _spiders)
        {
            if (spider.IsDead ||
                !spider.Body.IsBuilt ||
                spider.DamageBody == null ||
                !spider.DamageBody.IsBuilt)
            {
                continue;
            }

            _spiderMovementBodiesForRaycast.Add(spider.Body.Native);
        }

        return _spiderMovementBodiesForRaycast;
    }

    public void DrawDebug(
        Renderer.MasterRenderer renderer,
        Camera camera,
        float aspect,
        Matrix4x4? viewOverride = null)
    {
        var enemyTex = _assets.GetTexture(Core.Bible.Tex(Core.Bible.EnemyIcon));
        var view = viewOverride ?? camera.GetViewMatrix();
        var proj = camera.GetProjectionMatrix(aspect);

        for (int i = 0; i < _enemies.Count; i++)
        {
            var enemy = _enemies[i];
            if (!enemy.IsDead)
            {
                var pos = enemy.Entity.Transform.Position;
                pos.Y += 2.0f; // Acima da cápsula
                renderer.QueueBillboard(view, proj, enemyTex.ID, pos, new Vector2(0.5f, 0.5f), new Vector4(1, 0, 0, 0.8f));
            }
        }

        for (int i = 0; i < _spiders.Count; i++)
        {
            var spider = _spiders[i];
            if (!spider.IsDead)
            {
                var pos = spider.Entity.Transform.Position;
                pos.Y += 2.0f; // Acima da cápsula
                renderer.QueueBillboard(view, proj, enemyTex.ID, pos, new Vector2(0.5f, 0.5f), new Vector4(1, 0, 0, 0.8f));
            }
        }
    }

    public void Clear()

    {
        for (int i = _enemies.Count - 1; i >= 0; i--)
        {
            var e = _enemies[i];

            _sceneManager.ActiveScene.Remove(e.Entity);
            e.Character?.Dispose();

            if (e.Body.IsBuilt)
            {
                _physics.DestroyBody(e.Body.Native);
                e.Body.Destroy();
            }

            e.Dispose();
        }

        _enemies.Clear();

        for (int i = _spiders.Count - 1; i >= 0; i--)
        {
            var s = _spiders[i];
            UnregisterSpiderDamageBodies(s);
            _sceneManager.ActiveScene.Remove(s.Entity);
            s.Dispose();

            if (s.Body.IsBuilt)
            {
                _physics.DestroyBody(s.Body.Native);
                s.Body.Destroy();
            }

        }

        _spiders.Clear();
        _spiderDamageBodies.Clear();
        _spiderMovementBodiesForRaycast.Clear();
    }

    private void RegisterSpiderDamageBodies(SpiderEnemy spider)
    {
        SpiderDamageBody? damageBody = spider.DamageBody;
        if (damageBody == null)
            return;

        foreach (BodyID bodyId in damageBody.BodyIds)
        {
            if (bodyId.IsValid)
                _spiderDamageBodies[bodyId] = spider;
        }
    }

    private void UnregisterSpiderDamageBodies(SpiderEnemy spider)
    {
        var staleBodyIds = new List<BodyID>();

        foreach (KeyValuePair<BodyID, SpiderEnemy> entry
                 in _spiderDamageBodies)
        {
            if (ReferenceEquals(entry.Value, spider))
                staleBodyIds.Add(entry.Key);
        }

        foreach (BodyID bodyId in staleBodyIds)
            _spiderDamageBodies.Remove(bodyId);
    }

    public void Dispose()
    {
        Clear();
    }
}
