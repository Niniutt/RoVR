using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROVController : MonoBehaviour
{
    private DataObject data;
    private ROVMetrics metrics;
    public GameObject uiNotification;
    Vector3 orientationOffset = new Vector3(-90, 0, 0);
    int integer = 0;
    float timer = 0;

    // Start is called before the first frame update
    void Start()
    {
        data = ScriptableObject.CreateInstance<DataObject>();
        metrics = ScriptableObject.CreateInstance<ROVMetrics>();
    }

    // Update is called once per frame
    void Update()
    {
        timer += Time.deltaTime;
        if (integer < 2)
        {
            if (timer > 5)
            {
                timer = 0;
                integer++;
                GameObject notification = Instantiate(uiNotification, transform);
                notification.GetComponent<UINotification>().setNotification("Depth Alert", "This is a Depth test", NotificationType.DEPTH, NotificationLevel.WARNING);
                metrics.OnNewNotification(notification);

            }
        }
        // Checking depth and battery levels
        if (data.depth >= metrics.depthWarningThreshold && data.depth < metrics.maxDepth)
        {
            GameObject notification = Instantiate(uiNotification, transform);
            notification.GetComponent<UINotification>().setNotification("Depth Alert", "Approaching Maximum depth", NotificationType.DEPTH, NotificationLevel.WARNING);
            metrics.OnNewNotification(notification);
        }
        else if (data.depth >= metrics.maxDepth)
        {
            GameObject notification = Instantiate(uiNotification, transform);
            notification.GetComponent<UINotification>().setNotification("Depth Alert", "Reaching Maximum depth", NotificationType.DEPTH, NotificationLevel.ALERT);
            metrics.OnNewNotification(notification);
        }

        //if (data.battery < metrics.batteryWarningThreshold && data.battery > metrics.batteryAlertThreshold)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Battery Alert", "Battery is low", NotificationType.BATTERY, NotificationLevel.WARNING);
        //    metrics.OnNewNotification(notification);
        //}
        //else if (data.battery < metrics.batteryAlertThreshold)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Battery Alert", "Battery is critically low", NotificationType.BATTERY, NotificationLevel.ALERT);
        //    metrics.OnNewNotification(notification);
        //}
    }

    public void OnOrientationUpdate(Vector4 orientation)
    {
        //transform.rotation = Quaternion.Euler(-90, 0, 0) * new Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        transform.rotation = Quaternion.Euler(orientationOffset) * new Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    }
}
