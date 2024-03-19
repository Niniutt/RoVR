using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROVController : MonoBehaviour
{
    private DataObject data;
    private ROVMetrics metrics;

    // Start is called before the first frame update
    void Start()
    {
        data = ScriptableObject.CreateInstance<DataObject>();
        metrics = ScriptableObject.CreateInstance<ROVMetrics>();
    }

    // Update is called once per frame
    void Update()
    {
        // Checking depth and battery levels
        if (data.depth < metrics.depthWarningThreshold)
        {
            UINotification notification = new UINotification();
            notification.setNotification("Depth Alert", "Approaching Maximum depth", NotificationType.WARNING);
            metrics.newNotification.Add(notification);
        }
        else if (data.depth < metrics.maxDepth)
        {
            UINotification notification = new UINotification();
            notification.setNotification("Depth Alert", "Reaching Maximum depth", NotificationType.ALERT);
            metrics.newNotification.Add(notification);
        }
        if (data.battery < metrics.batteryWarningThreshold)
        {
            UINotification notification = new UINotification();
            notification.setNotification("Battery Alert", "Battery is low", NotificationType.WARNING);
            metrics.newNotification.Add(notification);
        }
        else if (data.battery < metrics.batteryAlertThreshold)
        {
            UINotification notification = new UINotification();
            notification.setNotification("Battery Alert", "Battery is critically low", NotificationType.ALERT);
            metrics.newNotification.Add(notification);
        }
    }
}
