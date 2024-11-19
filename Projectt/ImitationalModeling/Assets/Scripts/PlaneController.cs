using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneController : MonoBehaviour
{
    public bool AirBrakeDeployed => _airbrakeDeployed;

    [SerializeField] private Rigidbody _rigidbody;

    [Header("VelocityParams")]
    private Vector3 _currentVelocity;
    private Vector3 _currentLocalVelocity;
    private Vector3 _currentLocalAngularVelocity;

    [Header("AOA params")]
    private float _angleOfAttack;
    private float _angleOfAttackYaw;
    
    private Vector3 _localGForce;
    private Vector3 _lastVelocity;                          // Velocity difference
    [Header("G-Force")]
    [SerializeField] private float _gLimit;
    [SerializeField] private float _gLimitPitch;

    [Header("Throttle")]                                        //temp values!
    [SerializeField] private float _maxThrust = 200000;               
    private float _throttle;                                // 0-1 value of Throttle percent work
    public float Throttle { get { return _throttle; } private set { _throttle = value; } }

    [SerializeField] private float _throttleSpeed;          // Throttle change speed
    private float _throttleInput;

    [Header("Lift")]

    [SerializeField] private float _liftPower;
    [SerializeField] private AnimationCurve _angleOfAttackLiftCurve; 
    [SerializeField] private float _inducedDrag;                             // Air flow contr drug
    [SerializeField] private AnimationCurve _inducedDragCurve;

    [Header("Flaps")]
    [SerializeField] private float _flapsLiftPower;
    [SerializeField] private float _flapsAngleOfAttackBias;         //изменение угла атаки при развернутых закрылках
    [SerializeField] float _flapsDrag;
    [SerializeField] private float _flapsRetractSpeed;          // Max flaps deploy speed
    [SerializeField] private bool _flapsDeployed;

    [Header("Rudder")]
    [SerializeField] private float _rudderPower;                            // Yaw lift from rudder // мощность управляющего руля, который создает подъемную силу по оси рысканья (yaw).
    [SerializeField] private AnimationCurve _angleOfAttackRudderCurve;
    [SerializeField] private AnimationCurve _rudderInducedDragCurve;

    [Header("Drag")]
    [SerializeField] AnimationCurve _dragForward;
    [SerializeField] AnimationCurve _dragBack;
    [SerializeField] AnimationCurve _dragLeft;
    [SerializeField] AnimationCurve _dragRight;
    [SerializeField] AnimationCurve _dragTop;
    [SerializeField] AnimationCurve _dragBottom;

    [Header("Steering")]
    [SerializeField] private Vector3 _turnSpeed;
    [SerializeField] private Vector3 _turnAcceleration;

    [SerializeField] private AnimationCurve _steeringCurve;
    [SerializeField] private Vector3 _currentControlInput;

    [Header("Air Density And Temperature")]
    private const float _seaLevelTemperature = 288.15f;         // Standart 15C on sea level
    private const float _temperatureLapseRate = -0.0065f;       // 6.5 per 1 km

    [Header("Air brake")]                                // Flaps: lift+drag; Airbrake
    [SerializeField] float _airbrakeDrag;
    [SerializeField] private bool _airbrakeDeployed;

    [Header("Landing Gear")]
    [SerializeField] private PhysicMaterial _landingGearBrakesMaterial;
    [SerializeField] private PhysicMaterial _landingGearDefaultMaterial;
    [SerializeField] private List<Collider> _landingGear;

    [Header("Inner calculative params")] 
    private Quaternion _invRotation;

    public Vector3 EffectiveInput { get; private set; }

    [Header("Objects")]
    [SerializeField] private List<GameObject> _graphicsObjects;
    private bool _isDestroyed = false;



    public void PlaneCollisionCheck(Collision collision)
    {
        for (var i = 0; i < collision.contactCount; i++)
        {
            var contact = collision.contacts[i];

            if (_landingGear.Contains(contact.thisCollider))
            {
                return;
            }

            _rigidbody.isKinematic = true;
            _rigidbody.position = contact.point;
            _rigidbody.rotation = Quaternion.Euler(0, _rigidbody.rotation.eulerAngles.y, 0);

            foreach (var go in _graphicsObjects)
            {
                go.SetActive(false);
            }

            Die();

            return;
        }
    }

    private void FixedUpdate()
    {
        _invRotation = Quaternion.Inverse(_rigidbody.rotation);             // Quaternion inversion to get into local from global

        var deltaTime = Time.fixedDeltaTime;

        CalculateParametrs(deltaTime);
        UpadateParametrs(deltaTime);
        Logs();

    }

    private void CalculateParametrs(float deltaTime)
    {
        CalculateState();
        CalculateAOA();
        CalculateGForces(deltaTime);
    }

    private void UpadateParametrs(float deltaTime)
    {

        UpdateFlaps();

        UpdateLift();
        UpdateThrust();
        UpdateThrottle(deltaTime);
        UpdateSteering(deltaTime);

        UpdateDrag();
    }


    #region Calculation
    private Vector3 CalculateGForceLimit(Vector3 input)
    {
        return Utilities.Scale6(input,
            _gLimit, _gLimitPitch,
            _gLimit, _gLimit,
            _gLimit, _gLimit) * 9.81f;
    }

    private float CalculateSteering(float dt, float angularVelocity, float targetVelocity, float acceleration)      // angular velocity
    {
        var difference = targetVelocity - angularVelocity;
        var accel = acceleration * dt;

        return Mathf.Clamp(difference, -accel, accel);
    }
    private void CalculateState()                                       // Current Velocity params calculation
    {
        _currentVelocity = _rigidbody.velocity;
        _currentLocalVelocity = _invRotation * _currentVelocity;
        _currentLocalAngularVelocity = _invRotation * _rigidbody.angularVelocity;    
    }
    
    private void CalculateAOA()                                   //  AOA calculation
    {
        if (_currentLocalVelocity.sqrMagnitude <= 0.1f)                     // Low velocity implimentation
        {
            _angleOfAttack = 0;
            _angleOfAttackYaw = 0;
        
            return;
        }

        _angleOfAttack = Mathf.Atan2(-_currentLocalVelocity.y, _currentLocalVelocity.z);
        _angleOfAttackYaw = Mathf.Atan2(_currentLocalVelocity.x, _currentLocalVelocity.z);      // Yaw Aoa

    }

    private Vector3 CalculateGForce(Vector3 angularVelocity, Vector3 velocity)
    {
        return Vector3.Cross(angularVelocity, velocity);            // vector multiplication
    }
    private float CalculateGLimiter(Vector3 controlInput, Vector3 maxAngularVelocity)                       // Coef of input to G-force
    {
        var maxInput = controlInput.normalized;

        var limit = CalculateGForceLimit(maxInput);

        var maxGForce = CalculateGForce(Vector3.Scale(maxInput, maxAngularVelocity), _currentLocalVelocity);

        if (maxGForce.magnitude > limit.magnitude)
        {
            return limit.magnitude / maxGForce.magnitude;
        }

        return 1;
    }
    // (blindness effect impl)
    private void CalculateGForces(float deltaTime)                  // G-force calculation
    {
        var acceleration = (_currentVelocity - _lastVelocity) / deltaTime;

        _localGForce = _invRotation * acceleration;

        _lastVelocity = _currentVelocity;
    }

    private Vector3 CalculateLift(float angleOfAttack, Vector3 rightAxis, float liftPower,
        AnimationCurve animationCurve, AnimationCurve inducedDragCurve)
    {
        var liftVelocityOnWings = Vector3.ProjectOnPlane(_currentLocalVelocity, rightAxis);         // To wing
        var liftVelocityOnWingsSqr = liftVelocityOnWings.sqrMagnitude;                              // Air wing-flow speed
        var liftCoefficient = animationCurve.Evaluate(angleOfAttack * Mathf.Rad2Deg);

        var liftForce = liftVelocityOnWingsSqr * liftCoefficient * liftPower;
        var liftDirection = Vector3.Cross(liftVelocityOnWings.normalized, rightAxis);
        var lift = liftDirection * liftForce;

        var dragForce = liftCoefficient * liftCoefficient * _inducedDrag;           
        var dragDirection = -liftVelocityOnWings.normalized;
        var inducedDrag = dragDirection * liftVelocityOnWingsSqr * dragForce
                          * inducedDragCurve.Evaluate(Mathf.Max(0, _currentLocalVelocity.z));

        return lift + inducedDrag;          // result force return
    }


    private float CalculateAirDensity(float altitude)
    {
        var temperature = _seaLevelTemperature + _temperatureLapseRate * altitude;
        var pressure = 101325 * Mathf.Pow(1 + (_temperatureLapseRate * altitude) / _seaLevelTemperature, -9.80665f / (_temperatureLapseRate * 287.05f));        // Начальное давление на уровне моря — 101325 Па.
        var density = pressure / (287.05f * temperature);

        return Mathf.Max(density, 0.1f);
    }

    #endregion

    #region Update

    private void UpdateSteering(float dt)
    {
        var speed = Mathf.Max(0, _currentLocalVelocity.z);
        var steeringPower = _steeringCurve.Evaluate(speed);

        var airDensity = CalculateAirDensity(_rigidbody.position.y);
        // Расчет масштабирования силы управления с учетом перегрузок
        var gForceScaling = CalculateGLimiter(_currentControlInput, _turnSpeed * Mathf.Deg2Rad * steeringPower * airDensity);

        // Целевая угловая скорость с учетом ограничений по перегрузкам
        var targetAV = Vector3.Scale(_currentControlInput, _turnSpeed * steeringPower * gForceScaling);
        var av = _currentLocalAngularVelocity * Mathf.Rad2Deg;

        var correction = new Vector3(                                                           // Correction to 3D space
            CalculateSteering(dt, av.x, targetAV.x, _turnAcceleration.x * steeringPower),
            CalculateSteering(dt, av.y, targetAV.y, _turnAcceleration.y * steeringPower),
            CalculateSteering(dt, av.z, targetAV.z, _turnAcceleration.z * steeringPower));

        _rigidbody.AddRelativeTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);         // Torque moment apply
        
        
        var correctionInput = new Vector3(
               Mathf.Clamp((targetAV.x - av.x) / _turnAcceleration.x, -1, 1),
               Mathf.Clamp((targetAV.y - av.y) / _turnAcceleration.y, -1, 1),
               Mathf.Clamp((targetAV.z - av.z) / _turnAcceleration.z, -1, 1)
           );
        var effectiveInput = (correctionInput + _currentControlInput) * gForceScaling;

        //Получение нового свойства
        EffectiveInput = new Vector3(
            Mathf.Clamp(effectiveInput.x, -1, 1),
            Mathf.Clamp(effectiveInput.y, -1, 1),
            Mathf.Clamp(effectiveInput.z, -1, 1)
        );
    }

    private void UpdateThrust()
    {
        var thrust = _throttle * _maxThrust;
        _rigidbody.AddRelativeForce(thrust * Vector3.forward);

        Debug.Log($"Thrust: {thrust}");
    }

    private void UpdateThrottle(float dt)
    {
        float target = 0;

        if (_throttleInput > 0)
        {
            target = 1;
        }

        _throttle = Utilities.MoveTo(_throttle, target, _throttleSpeed * Mathf.Abs(_throttleInput), dt);


        // Airbrakes
        _airbrakeDeployed = _throttle == 0 && _throttleInput == -1;     // Aibrake deploy for 0 throttle

        if (_airbrakeDeployed)                                          // LandingGear PM_change lift/down
        {
            foreach (var lg in _landingGear)
            {
                lg.sharedMaterial = _landingGearBrakesMaterial;         
            }
        }
        else
        {
            foreach (var lg in _landingGear)
            {
                lg.sharedMaterial = _landingGearDefaultMaterial;
            }
        }
    }

    private void UpdateDrag()
    {
        var globalVelocity = _rigidbody.velocity;
        var globalVelocitySqr = globalVelocity.sqrMagnitude;

        var airbrakeDrag = _airbrakeDeployed ? _airbrakeDrag : 0f;
        var flapsDrag = _flapsDeployed ? _flapsDrag : 0f;

        var dragCoefficientToDirections = Utilities.Scale6(
            globalVelocity.normalized,
            _dragRight.Evaluate(Mathf.Abs(globalVelocity.x)),
            _dragLeft.Evaluate(Mathf.Abs(globalVelocity.x)),
            _dragTop.Evaluate(Mathf.Abs(globalVelocity.y)),
            _dragBottom.Evaluate(Mathf.Abs(globalVelocity.y)),
            _dragForward.Evaluate(Mathf.Abs(globalVelocity.z) + airbrakeDrag + flapsDrag),      // Counter force for forward movement    
            _dragBack.Evaluate(Mathf.Abs(globalVelocity.z))
        );

        var drag = dragCoefficientToDirections.magnitude * globalVelocitySqr * -globalVelocity.normalized;

        _rigidbody.AddForce(drag, ForceMode.Force);

    }
    private void UpdateLift()
    {
        if (_currentLocalVelocity.sqrMagnitude < 1f)
        {
            return;
        }

        var airDensity = CalculateAirDensity(_rigidbody.position.y);

        var flapsLiftPower = FlapsDeployed ? _flapsLiftPower : 0;
        var flapsAngleOfAttackBias = FlapsDeployed ? _flapsAngleOfAttackBias : 0;

        var effectiveAoA = _angleOfAttack + (flapsAngleOfAttackBias * Mathf.Deg2Rad);

        var liftMultiplier = 1f;

        var liftForce = CalculateLift(effectiveAoA,
            Vector3.right, (_liftPower + flapsLiftPower) * airDensity * liftMultiplier,
            _angleOfAttackLiftCurve, _inducedDragCurve);

        var yawForce = CalculateLift(_angleOfAttackYaw, Vector3.up, _rudderPower * airDensity,
            _angleOfAttackRudderCurve, _rudderInducedDragCurve);

        _rigidbody.AddRelativeForce(liftForce);
        _rigidbody.AddRelativeForce(yawForce);
    }

    private void UpdateFlaps()
    {
        if (_currentLocalVelocity.z > _flapsRetractSpeed)
        {
            FlapsDeployed = false;
        }
    }

    #endregion

    #region Input Handle
    public void SetThrottleInput(float input)
    {
        _throttleInput = input;
    }

    public void SetControlInput(Vector3 input)
    {
        _currentControlInput = Vector3.ClampMagnitude(input, 1);
    }

    public bool FlapsDeployed
    {
        get => _flapsDeployed;

        private set
        {
            _flapsDeployed = value;

            foreach (var lg in _landingGear)
            {
                lg.enabled = value;
            }
        }
    }


    public void ToggleFlaps()
    {
        if (_currentLocalVelocity.z < _flapsRetractSpeed)
        {
            FlapsDeployed = !FlapsDeployed;
        }
    }
    #endregion

    #region Debug
    private void Logs() 
    {
        Debug.Log($"LocVelocity: {_currentLocalVelocity}, AoA: {_angleOfAttack * Mathf.Rad2Deg}");
        Debug.Log($"Lift: {_liftPower}");
        Debug.Log($"Throttle: {_throttle}");
    }
    #endregion

    private void Die()
    {
        _throttle = 0;
        _isDestroyed = true;

        _rigidbody.isKinematic = true;

        foreach (var go in _graphicsObjects)
        {
            go.SetActive(false);
        }

    }
}
