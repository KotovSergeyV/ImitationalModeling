using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneCollision : MonoBehaviour
{

    private void OnCollisionEnter(Collision collision)
    {
        FindObjectOfType<PlaneController>().PlaneCollisionCheck(collision);
    }

}
