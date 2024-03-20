using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROVMetrics : ScriptableObject
{
    public int maxDepth = 200;
    public int depthWarningThreshold = 175;
    public int batteryWarningThreshold = 25;
    public int batteryAlertThreshold = 15;
    public List<GameObject> newNotification = new List<GameObject>();
    public List<GameObject> notificationLog = new List<GameObject>();
}
