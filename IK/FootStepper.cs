using System.Collections;
using UnityEngine;

public class FootStepper : MonoBehaviour
{
    // -- Scene references ------------------------------------------------------
    [Header("References")]
    public Transform footTarget;
    public Transform homeTransform;
    [Tooltip("Ankle/toe endpoint ? projected DOWN to ground at startup.")]
    public Transform footEffector;
    public FootStepper otherFoot;

    // -- Step parameters -------------------------------------------------------
    [Header("Step Settings")]
    [Range(0f, 15f)]
    public float stepThreshold      = 3f;

    [Range(0f, 8f)]
    public float stepReach          = 3f;

    [Range(0f, 5f)]
    public float minStepAhead       = 1.5f;

    [Range(0f, 5f)]   public float stepHeight          = 2f;
    [Range(0.05f, 1f)]public float stepDuration        = 0.35f;
    [Range(0f, 2f)]   public float stepOvershootFactor = 1.0f;

    // -- Gait Timing -----------------------------------------------------------
    [Header("Gait Timing")]
    [Range(0f, 2f)] public float stanceDuration    = 0.45f;
    [Range(0f, 2f)] public float initialStanceDelay;

    // -- Pre-Swing -------------------------------------------------------------
    [Header("Pre-Swing (Toe Push-Off)")]
    [Range(0f, 0.3f)] public float preSwingDuration = 0.08f;
    [Range(0f, 3f)]   public float preSwingLift     = 0.8f;

    // -- Heel-to-Toe Roll ------------------------------------------------------
    [Header("Heel-to-Toe Roll")]
    [Range(0f, 5f)]      public float heelToToeLength = 1f;
    [Range(0.5f, 0.95f)] public float heelStrikeRatio = 0.75f;

    // -- Ground detection ------------------------------------------------------
    [Header("Ground")]
    public LayerMask groundLayer     = ~0;
    [Range(0f, 20f)] public float raycastStartHeight = 10f;

    // -- Obstacle Detection -----------------------------------------------------
    [Header("Obstacle Detection")]
    public LayerMask obstacleLayer;
    [Range(0f, 5f)]  public float obstacleScanRadius = 0.3f;

    // -- Debug -----------------------------------------------------------------
    [Header("Debug")]
    public bool debugDraw = true;

    public bool IsMoving { get; private set; }

    /// <summary>
    /// The Y world-position where this foot last committed to a surface.
    /// Updated only at the END of a step (not during swing) so GaitController
    /// can use it as the body height target when the next step begins.
    /// </summary>
    public float PlantedSurfaceY { get; private set; }

    /// <summary>Current live foot target Y position (world space).</summary>
    public float FootPlantedY => footTarget != null ? footTarget.position.y : 0f;

    // -- Private ---------------------------------------------------------------
    private Vector3 _lastHomePos;
    private Vector3 _homeVelocity;
    private float   _stanceCooldown;
    private Vector3 _lastWalkDir = Vector3.right;

    // Pre-allocated buffer for LiftOffObstacle overlap checks
    private readonly Collider[] _overlapBuffer = new Collider[4];

    private void Start()
    {
        if (footTarget == null || homeTransform == null)
        {
            Debug.LogError($"[FootStepper] {name}: footTarget or homeTransform not assigned!", this);
            enabled = false;
            return;
        }

        if (footEffector != null)
        {
            Vector3 ground = SnapToSurface(footEffector.position);
            homeTransform.position = ground;
            footTarget.position    = ground;
        }

        PlantedSurfaceY = footTarget.position.y;
        _stanceCooldown = initialStanceDelay;
        _lastHomePos    = homeTransform.position;
    }

    private void Update()
    {
        _homeVelocity = (homeTransform.position - _lastHomePos) / Time.deltaTime;
        _lastHomePos  = homeTransform.position;

        Vector3 flatVel = new Vector3(_homeVelocity.x, 0f, _homeVelocity.z);
        if (flatVel.sqrMagnitude > 0.01f)
            _lastWalkDir = flatVel.normalized;

        if (_stanceCooldown > 0f)
            _stanceCooldown -= Time.deltaTime;

        TryStep();

        if (debugDraw)
        {
            Debug.DrawLine(footTarget.position, homeTransform.position, Color.yellow);
            Debug.DrawRay(homeTransform.position, Vector3.up * stepThreshold, Color.cyan);
        }
    }

    private void TryStep()
    {
        if (IsMoving) return;
        if (_stanceCooldown > 0f) return;
        if (otherFoot != null && otherFoot.IsMoving) return;

        // Use only horizontal velocity ? vertical body bob/lift must not trigger steps
        Vector3 flatVel = new Vector3(_homeVelocity.x, 0f, _homeVelocity.z);
        if (flatVel.sqrMagnitude < 0.01f) return;

        if (Vector3.Distance(footTarget.position, homeTransform.position) > stepThreshold)
            StartCoroutine(Step());
    }

    private IEnumerator Step()
    {
        IsMoving = true;

        Vector3 liftOff = footTarget.position;
        Vector3 flatVel = new Vector3(_homeVelocity.x, 0f, _homeVelocity.z);
        Vector3 walkDir = flatVel.sqrMagnitude > 0.01f ? flatVel.normalized : _lastWalkDir;

        // -- Phase 0: pre-swing ------------------------------------------------
        if (preSwingDuration > 0f)
        {
            Vector3 preTarget = liftOff + Vector3.up * preSwingLift;
            float   elapsed   = 0f;
            while (elapsed < preSwingDuration)
            {
                elapsed += Time.deltaTime;
                float t = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(elapsed / preSwingDuration));
                footTarget.position = Vector3.Lerp(liftOff, preTarget, t);
                yield return null;
            }
            liftOff = footTarget.position;
        }

        // -- Compute landing ---------------------------------------------------
        Vector3 predicted  = homeTransform.position
                           + _homeVelocity * (stepDuration * stepOvershootFactor)
                           + walkDir * stepReach;

        // -- Obstacle adjustment ----------------------------------------------
        predicted = AdjustForObstacles(predicted, walkDir);

        Vector3 footCenter = LiftOffObstacle(SnapToSurface(predicted));

        // Guarantee this foot lands past the other foot.
        if (otherFoot != null && otherFoot.footTarget != null)
        {
            float projOther = Vector3.Dot(otherFoot.footTarget.position, walkDir);
            float projMine  = Vector3.Dot(footCenter, walkDir);
            if (projMine < projOther + minStepAhead)
            {
                footCenter += walkDir * (projOther + minStepAhead - projMine);
                footCenter  = LiftOffObstacle(SnapToSurface(footCenter));
            }
        }

        float   half     = heelToToeLength * 0.5f;
        Vector3 heelLand = LiftOffObstacle(SnapToSurface(footCenter - walkDir * half));
        Vector3 toeLand  = LiftOffObstacle(SnapToSurface(footCenter + walkDir * half));

        // -- Phase 1: swing arc ------------------------------------------------
        float flightDur = stepDuration * heelStrikeRatio;
        float el        = 0f;
        while (el < flightDur)
        {
            el += Time.deltaTime;
            float t = Mathf.Clamp01(el / flightDur);
            Vector3 pos = Vector3.Lerp(liftOff, heelLand, Mathf.SmoothStep(0f, 1f, t));
            pos.y += stepHeight * Mathf.Sin(t * Mathf.PI);
            footTarget.position = pos;
            yield return null;
        }
        footTarget.position = heelLand;

        // -- Phase 2: heel ? toe roll ------------------------------------------
        float rollDur = stepDuration * (1f - heelStrikeRatio);
        el = 0f;
        while (el < rollDur)
        {
            el += Time.deltaTime;
            float t = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(el / rollDur));
            footTarget.position = Vector3.Lerp(heelLand, toeLand, t);
            yield return null;
        }
        footTarget.position = toeLand;

        // Commit the final planted Y ? GaitController reads this when the next step starts
        PlantedSurfaceY = footTarget.position.y;

        _stanceCooldown = stanceDuration;
        IsMoving = false;
    }

    private Vector3 SnapToSurface(Vector3 point)
    {
        Vector3 origin = point + Vector3.up * raycastStartHeight;
        LayerMask combined = groundLayer | obstacleLayer;
        if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit,
                            raycastStartHeight * 3f, combined))
        {
            if (debugDraw)
                Debug.DrawRay(origin, Vector3.down * hit.distance, Color.magenta, stepDuration);
            return hit.point;
        }
        return point;
    }

    /// <summary>
    /// Checks if the predicted landing point's path intersects an obstacle along the X axis.
    /// If so, returns a corrected point on top of the obstacle.
    /// </summary>
    private Vector3 AdjustForObstacles(Vector3 predictedLanding, Vector3 walkDir)
    {
        if (obstacleLayer.value == 0) return predictedLanding;

        Vector3 footPos = footTarget.position;
        float   dist    = Mathf.Abs(predictedLanding.x - footPos.x);

        if (dist < 0.01f) return predictedLanding;

        // Raise origin to mid-leg height so the sphere hits the box's side face,
        // not the ground-level bottom edge
        Vector3 castOrigin = footPos + Vector3.up * (stepHeight * 0.5f);

        if (debugDraw)
            Debug.DrawRay(castOrigin, walkDir * dist, Color.green);

        if (Physics.SphereCast(castOrigin, obstacleScanRadius, walkDir, out RaycastHit hit,
                dist, obstacleLayer))
        {
            Bounds  bounds        = hit.collider.bounds;
            Vector3 topPoint      = new Vector3(hit.point.x, bounds.max.y, hit.point.z);
            Vector3 aboveObstacle = topPoint + Vector3.up * 0.5f;

            if (Physics.Raycast(aboveObstacle, Vector3.down, out RaycastHit topHit,
                    2f, obstacleLayer | groundLayer))
            {
                if (debugDraw)
                    Debug.DrawLine(footPos, topHit.point, Color.red, stepDuration);
                return topHit.point;
            }
            return topPoint;
        }

        return predictedLanding;
    }

    /// <summary>
    /// If <paramref name="point"/> is inside an obstacle collider, snaps it up to
    /// the top surface of that obstacle.  Acts as a safety net after SnapToSurface.
    /// </summary>
    private Vector3 LiftOffObstacle(Vector3 point)
    {
        if (obstacleLayer.value == 0) return point;

        int count = Physics.OverlapSphereNonAlloc(point, 0.05f, _overlapBuffer, obstacleLayer);
        if (count == 0) return point;

        float topY = point.y;
        for (int i = 0; i < count; i++)
            topY = Mathf.Max(topY, _overlapBuffer[i].bounds.max.y);

        Vector3 lifted = new Vector3(point.x, topY, point.z);

        if (debugDraw)
            Debug.DrawLine(point, lifted, Color.red, stepDuration);

        return lifted;
    }
#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        if (homeTransform != null)
        {
            Gizmos.color = IsMoving ? Color.red : Color.yellow;
            DrawCircle(homeTransform.position, stepThreshold);
        }
        if (footTarget != null)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(footTarget.position, 0.25f);
        }
    }

    private static void DrawCircle(Vector3 c, float r, int seg = 32)
    {
        Vector3 prev = c + new Vector3(r, 0, 0);
        for (int i = 1; i <= seg; i++)
        {
            float   a  = i * 360f / seg * Mathf.Deg2Rad;
            Vector3 nx = c + new Vector3(Mathf.Cos(a) * r, 0f, Mathf.Sin(a) * r);
            Gizmos.DrawLine(prev, nx);
            prev = nx;
        }
    }
#endif
}
