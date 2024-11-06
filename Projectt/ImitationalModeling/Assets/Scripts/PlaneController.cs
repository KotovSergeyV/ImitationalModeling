using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    [SerializeField] private Rigidbody _rigidbody;

    [Header("VelocityParams")]
    private Vector3 _currentVelocity;
    private Vector3 _currentLocalVelocity;
    private Vector3 _currentLocalAngularVelocity;

    [Header("AOA params")]
    private float _angleOfAttack;
    private float _angleOfAttackYaw;
    
    [Header("G-Force")]
    private Vector3 _localGForce;
    private Vector3 _lastVelocity;                          // Velocity difference



    [Header("Inner calculative params")] 
    private Quaternion _invRotation;
    
    
    private void FixedUpdate()
    {
        _invRotation = Quaternion.Inverse(_rigidbody.rotation);             // Quaternion inversion to get into local from global
        
        var deltaTime = Time.fixedDeltaTime;

        CalculateState();
        CalculateGForces(deltaTime);
    }
    
    
    private void CalculateState()                                       // Current Velocity params calculation
    {
        _currentVelocity = _rigidbody.velocity;
        _currentLocalVelocity = _invRotation * _currentVelocity;
        _currentLocalAngularVelocity = _invRotation * _rigidbody.angularVelocity;
        
        CalculateAOA();
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

        Debug.Log($"LocVelocity: {_currentLocalVelocity}, AoA: {_angleOfAttack * Mathf.Rad2Deg}");
    }
    
    
    // (blindness effect impl)
    private void CalculateGForces(float deltaTime)                  // G-force calculation
    {
        var acceleration = (_currentVelocity - _lastVelocity) / deltaTime;

        _localGForce = _invRotation * acceleration;

        _lastVelocity = _currentVelocity;
    }
}
