using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataObject : ScriptableObject
{
    [SerializeField]

    public List<Vector3> positionDelta = new List<Vector3>();
    

    public List<float> depthLog = new List<float>();
    public List<float> velocityLog = new List<float>();
    public List<float> batteryLog = new List<float>();

    public float time = 0;
    public float time_before_ascent = 0;
    public float battery = 0;
    public float batteryLifetime = 0;
    public float pressure = 0;
    public float depth = 0;
    public Vector4 orientation = new Vector4();
    public Vector3 angularVelocity = new Vector3();
    public Vector3 linearAcceleration = new Vector3();
    public Vector3 dopplerVelocity = new Vector3(); // meters per second
}
