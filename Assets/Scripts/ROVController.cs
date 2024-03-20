using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROVController : MonoBehaviour
{
    private DataObject data;
    private ROVMetrics metrics;
    public GameObject uiNotification;
    Vector3 orientationOffset = new Vector3(0, 0, 90);

    // Start is called before the first frame update
    void Start()
    {
        data = ScriptableObject.CreateInstance<DataObject>();
        metrics = ScriptableObject.CreateInstance<ROVMetrics>();
    }

    // Update is called once per frame
    void Update()
    {
        //// Checking depth and battery levels
        //if (data.depth < metrics.depthWarningThreshold)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Depth Alert", "Approaching Maximum depth", NotificationType.WARNING);
        //    metrics.newNotification.Add(notification);
        //}
        //else if (data.depth < metrics.maxDepth)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Depth Alert", "Reaching Maximum depth", NotificationType.ALERT);
        //    metrics.newNotification.Add(notification);
        //}
        //if (data.battery < metrics.batteryWarningThreshold)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Battery Alert", "Battery is low", NotificationType.WARNING);
        //    metrics.newNotification.Add(notification);
        //}
        //else if (data.battery < metrics.batteryAlertThreshold)
        //{
        //    GameObject notification = Instantiate(uiNotification, transform);
        //    notification.GetComponent<UINotification>().setNotification("Battery Alert", "Battery is critically low", NotificationType.ALERT);
        //    metrics.newNotification.Add(notification);
        //}
    }

    public void OnOrientationUpdate(Vector4 orientation)
    {
        transform.rotation = Quaternion.Euler(-90, 0, 0) * new Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    }
}
