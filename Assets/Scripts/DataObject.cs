using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataObject : ScriptableObject
{
    [SerializeField]

    public List<Vector3> positionDelta = new List<Vector3>();
    // list of positions to average out for logging
    List<Vector3> positionBatch = new List<Vector3>();
    // List of positions averaged from X number of positions for the UI graphs
    List<Vector3> positionLog = new List<Vector3>();

    [SerializeField]
    public List<string> message = new List<string>();
    public float time = 0;
    public float time_before_ascent = 0;
    public float battery = 0;
    public float batteryLifetime = 0;
    public float pressure = 0;
    public float depth = 0;
    public Vector4 orientation = new Vector4();
    public Vector3 angularVelocity = new Vector3();
    public Vector3 linearAcceleration = new Vector3();
    public Vector3 dopplerVelocity = new Vector3();
}
