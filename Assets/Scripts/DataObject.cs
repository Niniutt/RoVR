using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataObject : ScriptableObject
{
    [SerializeField]
    public List<Vector3> positionDelta = new List<Vector3>();
    [SerializeField]
    public List<string> message = new List<string>();
}
