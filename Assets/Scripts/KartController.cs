using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Weight distribution")]
    [SerializeField, Range(0, 1)] private float _frontAxisShare = 0.5f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;

    [Header("Handbrake")]
    [SerializeField] private KeyCode handbrakeKey = KeyCode.Space;
    [SerializeField] private float handbrakeBrakeForce = 6000f;

    private InputAction _moveAction;
    private float _throttleInput;
    private float _steepInput;
    private bool _handbrakePressed;

    public float drag;
    public float downforce;
    public float flSuspension;

    private float _frontLeftNormalForce, _frontRightNormalForce, _rearLeftNormalForce, _rearRightNormalForce;
    private Rigidbody _rigidbody;
    private Vector3 g = Physics.gravity;

    [SerializeField] private float engineTorque = 400f;
    [SerializeField] private float wheelRadius = 0.3f;
    [SerializeField] private float maxSpeed = 20;

    [Header("Steering")]
    [SerializeField] private float maxSteeringAngle;

    [Header("Telemetry source")]
    [SerializeField] private CarSuspension _carSuspension;

    private float dragValue;
    private float downforceValue;
    private float flSuspensionForce;

    private GUIStyle telemetryStyle;
    private GUIStyle headerStyle;
    private GUIStyle valueStyle;
    private Texture2D bgTexture;

    private Quaternion frontLeftInitialRot;
    private Quaternion frontRightInitialRot;

    [Header("Tyre friction")]
    [SerializeField] private float frictionCoefficient = 1f;
    [SerializeField] private float lateralStiffnes = 80f;
    [SerializeField] private float rollingResistance;

    private float speedAlongForward = 0f;
    private float Fx = 0f;
    private float Fy = 0f;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();
        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_import) Initialize();

        frontLeftInitialRot = _frontLeftWheel.localRotation;
        frontRightInitialRot = _frontRightWheel.localRotation;
        ComputeStaticWheelLoad();
        InitializeGUIStyles();
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
            frictionCoefficient = _kartConfig.frictionCoefficient;
            rollingResistance = _kartConfig.rollingResistance;
            maxSteeringAngle = _kartConfig.maxSteerAngle;
            _gearRatio = _kartConfig.gearRatio;
            wheelRadius = _kartConfig.wheelRadius;
            lateralStiffnes = _kartConfig.lateralStiffness;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();

        if (_carSuspension != null)
        {
            dragValue = _carSuspension.drag;
            downforceValue = _carSuspension.downforce;
            flSuspensionForce = _carSuspension.flSuspension;
        }
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steepInput = Mathf.Clamp(move.x, -1, 1);
        _throttleInput = Mathf.Clamp(move.y, -1, 1);

        _handbrakePressed = Input.GetKey(handbrakeKey); // Ручник
    }

    void RotateFrontWheels()
    {
        float steerAngle = maxSteeringAngle * _steepInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = frontLeftInitialRot * steerRot;
        _frontRightWheel.localRotation = frontRightInitialRot * steerRot;
    }

    void ComputeStaticWheelLoad()
    {
        float mass = _rigidbody.mass;
        float totalWeight = mass * Mathf.Abs(g.y);
        float frontWeight = totalWeight * _frontAxisShare;
        float rearWeight = totalWeight - frontWeight;
        _frontRightNormalForce = frontWeight * 0.5f;
        _frontLeftNormalForce = _frontRightNormalForce;
        _rearRightNormalForce = rearWeight * 0.5f;
        _rearLeftNormalForce = _rearRightNormalForce;
    }

    private void ApplyEngineForces()
    {
        Vector3 forward = transform.forward;
        float speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, forward);
        if (_throttleInput > 0 && speedAlongForward > maxSpeed) return;

        float driveTorque = engineTorque * _throttleInput;
        float driveForcePerWheel = driveTorque / wheelRadius / 2;
        Vector3 forceRear = forward * driveForcePerWheel;

        _rigidbody.AddForceAtPosition(forceRear, _rearLeftWheel.position, ForceMode.Force);
        _rigidbody.AddForceAtPosition(forceRear, _rearRightWheel.position, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        ApplyEngineForces();

        ApplyWheelForce(_frontLeftWheel, _frontLeftNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_frontRightWheel, _frontRightNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_rearLeftWheel, _rearLeftNormalForce, isSteer: false, isDrive: true);
        ApplyWheelForce(_rearRightWheel, _rearRightNormalForce, isSteer: false, isDrive: true);
    }

    void ApplyWheelForce(Transform wheel, float normalForce, bool isSteer, bool isDrive)
    {
        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 velocity = _rigidbody.GetPointVelocity(wheelPos);
        float vlong = Vector3.Dot(velocity, wheelForward);
        float vlat = Vector3.Dot(velocity, wheelRight);

        Fx = 0f;
        Fy = 0f;

        if (isDrive)
        {
            speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
            float engineTorqueOut = _engine.Simulate(_throttleInput, speedAlongForward, Time.fixedDeltaTime);
            float totalWheelTorque = engineTorqueOut * _gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f;
            Fx += wheelTorque / wheelRadius;

            if (_handbrakePressed)
            {
                float brakeDir = vlong > 0 ? -1f : (vlong < 0 ? 1f : -1f);
                Fx += brakeDir * handbrakeBrakeForce;
            }
        }
        else if (isSteer)
        {
            float rooling = -rollingResistance * vlong;
            Fx += rooling;
        }

        float fyRaw = -lateralStiffnes * vlat;
        Fy += fyRaw;

        float frictionlimit = frictionCoefficient * normalForce;
        float forceLenght = Mathf.Sqrt(Fx * Fx + Fy * Fy);

        if (forceLenght > frictionlimit)
        {
            float scale = frictionlimit / forceLenght;
            Fy += scale;
            Fx += scale;
        }

        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rigidbody.AddForceAtPosition(force, wheel.position, ForceMode.Force);
    }


    private void InitializeGUIStyles()
    {
        bgTexture = MakeTex(1, 1, new Color(0.05f, 0.08f, 0.15f, 0.9f));

        telemetryStyle = new GUIStyle();
        telemetryStyle.fontSize = 18;
        telemetryStyle.normal.textColor = Color.white;
        telemetryStyle.alignment = TextAnchor.MiddleLeft;
        telemetryStyle.padding = new RectOffset(6, 6, 2, 2);

        headerStyle = new GUIStyle(telemetryStyle);
        headerStyle.fontSize = 20;
        headerStyle.normal.textColor = new Color(0.3f, 0.8f, 1f, 1f);
        headerStyle.alignment = TextAnchor.MiddleCenter;

        valueStyle = new GUIStyle(telemetryStyle);
        valueStyle.normal.textColor = new Color(1f, 0.9f, 0.4f, 1f);
        valueStyle.alignment = TextAnchor.MiddleRight;
    }

    void OnGUI()
    {
        int panelWidth = 340;
        int panelHeight = 260;
        int margin = 10;

        // панель справа
        Rect panelRect = new Rect(
            Screen.width - panelWidth - margin,
            margin,
            panelWidth,
            panelHeight
        );

        GUI.DrawTexture(panelRect, bgTexture);

        GUILayout.BeginArea(new Rect(panelRect.x + 10, panelRect.y + 10, panelWidth - 20, panelHeight - 20));

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        GUILayout.Label("Telemetry Dashboard", headerStyle, GUILayout.Height(24));
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        GUILayout.Space(4);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Drag:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.Label($"{dragValue:0.0} N", valueStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Downforce:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.Label($"{downforceValue:0.0} N", valueStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("FL Suspension:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.Label($"{flSuspensionForce:0.0} N", valueStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.EndHorizontal();

        GUILayout.Space(4);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Speed:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.Label($"{speedAlongForward:0.0} m/s ({(speedAlongForward * 3.6f):0.0} km/h)", valueStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Engine RPM:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.Label($"{_engine.CurrentRpm:0} RPM", valueStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Engine Torque:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.Label($"{_engine.CurrentTorque:0.0} N·m", valueStyle, GUILayout.Width(150), GUILayout.Height(22));
        GUILayout.EndHorizontal();

        GUILayout.Space(4);

        GUIStyle handbrakeStyle = new GUIStyle(telemetryStyle);
        handbrakeStyle.alignment = TextAnchor.MiddleCenter;
        handbrakeStyle.normal.textColor = _handbrakePressed
            ? new Color(1f, 0.4f, 0.4f, 1f)
            : new Color(0.6f, 0.6f, 0.6f, 1f);

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        GUILayout.Label("HANDBRAKE " + (_handbrakePressed ? "ON" : "OFF"),
            handbrakeStyle, GUILayout.Width(260), GUILayout.Height(22));
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        GUILayout.Space(4);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Fx:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.Label($"{Fx:0.0} N", valueStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Fy:", telemetryStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.Label($"{Fy:0.0} N", valueStyle, GUILayout.Width(150), GUILayout.Height(20));
        GUILayout.EndHorizontal();

        GUILayout.EndArea();
    }

    private Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; i++)
            pix[i] = col;

        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        return result;
    }
}
