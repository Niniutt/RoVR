using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.PlasticSCM.Editor.WebApi;
using UnityEngine;

public class UIDepth : MonoBehaviour
{
    public TextMeshProUGUI depth;
    public TextMeshProUGUI pressure;
    public TextMeshProUGUI currTime;
    public TextMeshProUGUI timeTilAscent;
    public DataManager depthDataManager;
    public Transform notificationParent;
    private int lastCheckedNotificationCount = 0;
    private int activeNotifications = 3;
    private int maxNotificatoins = 4;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        CheckForNotification();
    }

    public void OnDepthUpdate(float depthValue)
    {
        depth.text = depthValue.ToString() + "m";

        if (DataObject.batteryLog.Count % 50 == 0)
        {
            depthDataManager.CustomUpdate(new Vector2(DataObject.time, depthValue));
        }
    }
    public void OnPressureUpdate(float pressureValue)
    {
        pressure.text = ((int)pressureValue).ToString() + "atm";

    }
    public void OnTimeBeforeAscentUpdate(float time)
    {
        float seconds = (int)(time % 60);
        timeTilAscent.text = ((int)(time/60)).ToString() + "m " + (seconds).ToString() + "s";
    }

    public void OnTimeUpdate(float time)
    {
        // Debug.Log("Time: " + time)
        currTime.text = "Current time: " + ((int)time).ToString() + "s";

    }
    // TODO: make this an interface
/// <summary>
/// Check for new notifications, display them if new notifications are available (priority to depth notifications if found in the first 10)
/// and limit the number of notifications displayed
/// </summary>
    public void CheckForNotification()
    {
        if (ROVMetrics.newNotifications.Count > lastCheckedNotificationCount)
        { 
            // If notifications are above the limit, remove all notifications,
            // only show the most recent depth notification and the next 2 notifications
            if (activeNotifications > maxNotificatoins)
            {
                foreach (Transform t in notificationParent)
                {
                    Destroy(t.gameObject);
                }
                activeNotifications = 0;
                for (int i = 0; i > 10; i--)
                {
                    GameObject notification = ROVMetrics.newNotifications[ROVMetrics.newNotifications.Count - i];
                    if(notification.GetComponent<UINotification>().type == NotificationType.DEPTH)
                    { 
                        Instantiate(ROVMetrics.newNotifications[ROVMetrics.newNotifications.Count - i], notificationParent);
                        activeNotifications++;
                        break;
                    }
                }
                foreach (GameObject notification in ROVMetrics.newNotifications)
                {
                    if (activeNotifications >= maxNotificatoins)
                        break;

                    Instantiate(notification, notificationParent);
                    activeNotifications++;
                }
            }
            else
            {
                Instantiate(ROVMetrics.newNotifications[ROVMetrics.newNotifications.Count - 1], notificationParent);
                lastCheckedNotificationCount = ROVMetrics.newNotifications.Count;
                activeNotifications++;
            }
        }
    }
}
