using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class PlaneController : MonoBehaviour
{
    [Header("Inner calculative params")] 
    private Quaternion _invRotation;
    [SerializeField] private float _baseMass = 14300;
    
    [SerializeField] private bool _engineOff = false;
    public bool IsGrounded = true;

    public bool EngineOff
    {
        get { return _engineOff; }
        private set { _engineOff = value; }
    }

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
    
    [Header("Angular Drag")]
    [SerializeField] private Vector3 _angularDrag = new Vector3(1f, 1f, 1f);
    
    [Header("Stall")]
    [SerializeField] private float _stallAngle = 17f;
    [SerializeField] private float _stallRecoveryForce = 10f;
    //[SerializeField] private float _stallTorqueMultiplier = 1f;//момент силы, возникающий на винте
                                                               //или роторе при потере подъемной силы и изменении угла атаки

    private bool _isStalled = false;
    
    [Header("Steering")]
    [SerializeField] private Vector3 _turnSpeed;
    [SerializeField] private Vector3 _turnAcceleration;

    [SerializeField] private AnimationCurve _steeringCurve;
    [SerializeField] private Vector3 _currentControlInput;

    [Header("Glide")]
    [SerializeField] private float _glideRotationSpeed = 30f;
    [SerializeField] private float _glideDamping = 0.5f;
    /* это способность самолета илилетательного аппарата сохранять
      стабильность и контролируемость во время планирующего полета.
       Данный термин обычно относят к характеристикам, которые определяют,
        как быстро и эффективно самолет может реагировать на изменения угла
         атаки или отклонения от заданного курса при планировании.*/
    
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
    
    [Header("Fuel")]
    [SerializeField] private float _fuelCapacity = 6100f;
    [SerializeField] private float _fuelConsumptionRate = 0.5f;
    [SerializeField] private float _fuelDensity = 0.8f;

    [SerializeField] private float _fuelAmount;
    

    public Vector3 EffectiveInput { get; private set; }

    [Header("Objects")]
    [SerializeField] private List<GameObject> _graphicsObjects;
    private bool _isDestroyed = false;

    [Header("Turbulence")]
    [SerializeField] private float _turbulenceStrength = 0.05f;
    [SerializeField] private float _turbulenceFrequency = 1f;
    
    private void Start()
    {
        _fuelAmount = _fuelCapacity;
        UpdateMass();

        _lastVelocity = _rigidbody.velocity;
    }
    
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
        if (!IsGrounded)
        {
            UpdateTurbulence();
        }

        UpdateFlaps();

        if (!_engineOff)
        {
            UpdateThrust();
            UpdateThrottle(deltaTime);
        }
        else {
            UpdateGlide();
        }

        UpdateLift();
        UpdateDrag();
        UpdateAngularDrag();
        
        UpdateSteering(deltaTime);
        
        UpdateRecoveryStall();
        UpdateFuel(deltaTime);
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
        _isStalled = Mathf.Abs(_angleOfAttack) > _stallAngle;
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
    private void UpdateAngularDrag()
    {
        var av = _currentLocalAngularVelocity;
    
        if (av.sqrMagnitude == 0)
        {
            return;
        }

        var drag = Vector3.Scale(av.sqrMagnitude * -av.normalized, _angularDrag);
        Debug.Log("AngularDrag: "+ drag.ToString());
        _rigidbody.AddRelativeTorque(drag, ForceMode.Acceleration);
    }
    
    private void UpdateGlide()
    {
        if (!_engineOff)
        {
            return;
        }
        
        Debug.Log("Gliding");
        if (_rigidbody.velocity.sqrMagnitude < 0.1f)
        {
            return;
        }

        var forward = _rigidbody.velocity.normalized;
        var up = Vector3.up;
        
    //Creates a rotation with the specified forward and upwards directions.
        var targetRotation = Quaternion.LookRotation(forward, up);

        Debug.Log(targetRotation);
        /*
        current	The vector being managed.
        target	The vector.
*/
        _rigidbody.rotation = Quaternion.RotateTowards(_rigidbody.rotation, targetRotation, _glideRotationSpeed * Time.deltaTime);
        Debug.Log(Quaternion.RotateTowards(_rigidbody.rotation, targetRotation, _glideRotationSpeed * Time.deltaTime));
        _rigidbody.angularVelocity = Vector3.Lerp(_rigidbody.angularVelocity, Vector3.zero, Time.deltaTime * _glideDamping);
    }
    
    private void UpdateRecoveryStall()
    {
        if (!_isStalled)
        {
            return;
        }
        
        var stabilizationTorque = new Vector3(-_currentLocalAngularVelocity.x, 
            -_currentLocalAngularVelocity.y, -_currentLocalAngularVelocity.z);
        _rigidbody.AddRelativeTorque(stabilizationTorque * _stallRecoveryForce, ForceMode.Acceleration);    
        
    }
    
    private void UpdateTurbulence()
    {
        var turbulence = new Vector3(
            Random.Range(-_turbulenceStrength, _turbulenceStrength),
            Random.Range(-_turbulenceStrength, _turbulenceStrength),
            Random.Range(-_turbulenceStrength, _turbulenceStrength)
        );

        _rigidbody.AddRelativeTorque(turbulence, ForceMode.Acceleration);
        Debug.DrawLine(transform.position, transform.position+turbulence*100f, Color.cyan);
        Debug.Log("Turbulence:"+turbulence);
    }

    private void UpdateMass()
    {
        var fuelMass = _fuelAmount * _fuelDensity;
        _rigidbody.mass = _baseMass + fuelMass;
    }
    private void UpdateFuel(float deltaTime)
    {
        if (_fuelAmount <= 0 && !_engineOff)
        {
            _engineOff = true;
            _throttle = 0;
            _fuelAmount = 0;
    
            Debug.Log("Топливо закончилось, двигатель выключен.");
        }
        else if (_throttle > 0 && !_engineOff)
        {
            var fuelConsumed = _throttle * _fuelConsumptionRate * deltaTime;
            _fuelAmount = _fuelAmount - fuelConsumed;
            UpdateMass();
        }

        
    }
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
            _dragForward.Evaluate(Mathf.Abs(globalVelocity.z) + airbrakeDrag + flapsDrag), // Counter force for forward movement
            _dragBack.Evaluate(Mathf.Abs(globalVelocity.z))
        );

        var drag = dragCoefficientToDirections.magnitude * globalVelocitySqr * -globalVelocity.normalized;
        _rigidbody.AddForce(drag, ForceMode.Force);
        
        
        if (airbrakeDrag != 0 && Vector3.Project(globalVelocity, transform.forward).magnitude > 30)
        {
            var x = -transform.forward * (Vector3.Project(globalVelocity,
                transform.forward).magnitude)/(0.025f*airbrakeDrag);
            Debug.DrawLine(transform.position, transform.position + Vector3.Project(globalVelocity, transform.forward)*10, Color.blue);
            Debug.DrawLine(transform.position, transform.position + x*10, Color.yellow);
            _rigidbody.AddForce(x, ForceMode.Acceleration);
            Debug.Log("AirbrakesDrag"+x);
        }
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

    public void GearToggleHandler()
    {
        foreach (var lg in _landingGear)
        {
            lg.enabled = !lg.enabled;
        }
    }

    public void SetThrottleInput(float input)
    {
        _throttleInput = input;
        
        // Airbrakes
        _airbrakeDeployed = _throttle == 0 && _throttleInput == -1;     // Aibrake deploy for 0 throttle
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
        Debug.Log($"Velocity: {_currentLocalVelocity}, AoA: {_angleOfAttack}°, " +
                  $"IsStalled: {_isStalled}");Debug.Log($"Lift: {_liftPower}");
        Debug.Log($"Throttle: {_throttle}");
        Debug.Log($"AngularVelocity: {_currentLocalAngularVelocity}");
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
