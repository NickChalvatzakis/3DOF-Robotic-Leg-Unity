using UnityEngine;
public class GaitController : MonoBehaviour
{
    [Header("Foot Steppers")]
    public FootStepper leftFoot;
    public FootStepper rightFoot;
    [Header("Movement")]
    [Range(0f, 20f)] public float walkSpeed = 3f;
    public bool useKeyboardInput = true;
    [Header("Body Bob")]
    [Range(0f, 1f)]    public float bodyBobAmount    = 0.06f;
    [Range(0.1f, 10f)] public float bodyBobFrequency = 4f;
    [Header("Body Height Adaptation")]
    [Range(1f, 20f)] public float bodyHeightSpeed = 8f;
    public bool    IsMoving      { get; private set; }
    public Vector3 MoveDirection { get; private set; }
    public void SetMoveDirection(Vector3 d) => _externalDirection = d;
    private float   _baseLocalY;
    private float   _initialBodyOffset;
    private float   _targetBodyY;
    private float   _bobPhase;
    private Vector3 _externalDirection;
    private void Start()
    {
        float initialFootY = 0f;
        if (leftFoot != null && leftFoot.footTarget != null)
            initialFootY = leftFoot.footTarget.position.y;
        else if (rightFoot != null && rightFoot.footTarget != null)
            initialFootY = rightFoot.footTarget.position.y;
        _initialBodyOffset = transform.position.y - initialFootY;
        _baseLocalY        = transform.position.y;
        _targetBodyY       = transform.position.y;
    }
    private void Update()
    {
        GatherInput();
        ApplyLocomotion();
        ApplyBodyHeight();
        ApplyBodyBob();
    }
    private void GatherInput()
    {
        MoveDirection = useKeyboardInput
            ? new Vector3(Input.GetAxis("Horizontal"), 0f, 0f)
            : _externalDirection;
        if (MoveDirection.sqrMagnitude > 1f) MoveDirection = MoveDirection.normalized;
        IsMoving = MoveDirection.sqrMagnitude > 0.01f;
    }
    private void ApplyLocomotion()
    {
        if (!IsMoving) return;
        transform.position += MoveDirection * (walkSpeed * Time.deltaTime);
    }
    private void ApplyBodyHeight()
    {
        // _targetBodyY only updates when a foot STARTS its swing, set to the
        // stance foot's committed PlantedSurfaceY.
        //   Foot 1 plants on box  -> body stays low, leg bends  (no change yet)
        //   Press A, foot 2 swings -> _targetBodyY = foot1.PlantedSurfaceY + offset
        //   Body lerps up DURING foot 2 swing  (the push-off lift)
        // Stepping down is identical in reverse.
        bool leftMoving  = leftFoot  != null && leftFoot.IsMoving;
        bool rightMoving = rightFoot != null && rightFoot.IsMoving;
        if (leftMoving && rightFoot != null)
            _targetBodyY = rightFoot.PlantedSurfaceY + _initialBodyOffset;
        else if (rightMoving && leftFoot != null)
            _targetBodyY = leftFoot.PlantedSurfaceY + _initialBodyOffset;
        // Both planted -> _targetBodyY unchanged; body has already settled.
        _baseLocalY = Mathf.Lerp(_baseLocalY, _targetBodyY, Time.deltaTime * bodyHeightSpeed);
    }
    private void ApplyBodyBob()
    {
        if (IsMoving)
            _bobPhase += bodyBobFrequency * Time.deltaTime;
        else
            _bobPhase = Mathf.Lerp(_bobPhase, Mathf.Round(_bobPhase), 5f * Time.deltaTime);
        float bobOffset = bodyBobAmount * Mathf.Sin(_bobPhase * Mathf.PI * 2f);
        Vector3 pos = transform.position;
        pos.y = _baseLocalY + bobOffset;
        transform.position = pos;
    }
}