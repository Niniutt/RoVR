using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class ROV : MonoBehaviour
{
    private Vector3 pose;               // Current pose data
    private Quaternion attitude;        // Current attitude data
    private float pressure;             // Current pressure data
    private DataObject data;            // DataObject instance
    private Int32 i = 0;                // Simple iterator to read data.

    // Start is called before the first frame update
    void Start()
    {
        // Starts the ROV control system.
        // The starting point should be denoted by 0 in all directions
        Debug.Log("ROV started");
        if(data == null)
        {
            data = ScriptableObject.CreateInstance<DataObject>();
        }
        pose = data.pose[i];
        attitude = data.attitude[i];
        pressure = data.pressure[i];
        Debug.Log("ROV Control startup complete...");
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log("ROV status updating");
        if((i < data.pose.Count) && (i < data.attitude.Count) && (i < data.pressure.Count))
        {
            i++;
            pose = data.pose[i];
            attitude = data.attitude[i];
            pressure = data.pressure[i];
        }
    }
}
