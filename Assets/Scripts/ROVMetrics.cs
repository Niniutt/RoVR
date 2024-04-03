using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class ROVMetrics : ScriptableObject
{
    public int maxDepth = 200;
    public int depthWarningThreshold = 75;
    public int batteryWarningThreshold = 25;
    public int batteryAlertThreshold = 15;
    public static List<GameObject> newNotifications = new List<GameObject>();
    public List<GameObject> notificationLogs = new List<GameObject>();
    //private List<GameEventListener> listeners = new List<GameEventListener>();

    public void OnNewNotification(GameObject notification)
    {
        newNotifications.Add(notification);
    }
}
