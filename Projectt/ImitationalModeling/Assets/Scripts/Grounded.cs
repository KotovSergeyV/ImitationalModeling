using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grounded : MonoBehaviour
{
    [SerializeField] private PlaneController _planeController;
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag( "Floor"))
        {
            _planeController.IsGrounded = true;
        }
    }
    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Floor"))
        {
            _planeController.IsGrounded = false;
        }
    }
}
