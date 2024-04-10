using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIBattery : MonoBehaviour
{
    public TextMeshProUGUI messageTest;
    public TextMeshProUGUI currentBattery;
    public TextMeshProUGUI batteryRemainingEst;
    public TextMeshProUGUI timeTilAscent;
    public DataManager batteryDataManager;
    //public TextMeshProUGUI batteryNeededForAscent;
    public Transform notificationParent;
    private int lastCheckedNotificationCount = 0;
    private int activeNotifications = 2;
    private int maxNotificatoins = 3;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROVMetrics instance
    }

    // Update is called once per frame
    void Update()
    {
        CheckForNotification();
    }

    public void OnBatteryUpdate(float battery)
    {
        currentBattery.text = ((int)battery).ToString() + "%";
        
        if (DataObject.batteryLog.Count % 50 == 0)
        {
            batteryDataManager.CustomUpdate(new Vector2(DataObject.time, battery));
            // Debug.Log("Add point" + Time.deltaTime);
        }  
    }
    public void OnBatteryLifetimeUpdate(float time)
    {
        batteryRemainingEst.text = ((int)(time / 60)).ToString() + "m " + ((int)(time % 60)).ToString() + "s";
    }
    public void OnTimeBeforeAscentUpdate(float time)
    {
        float seconds = (int)(time % 60);
        timeTilAscent.text = ((int)(time / 60)).ToString() + "m " + (seconds).ToString() + "s";
    }

    public void OnNewNotification(GameObject notification)
    {
        Debug.Log("New Notification");
    }

    // TODO: make this an interface
    /// <summary>
    /// Check for new notifications, display them if new notifications are available and limit the number of notifications displayed
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
                for (int i = 0; i > (10 <= ROVMetrics.newNotifications.Count? 10: ROVMetrics.newNotifications.Count); i--)
                {
                    GameObject notification = ROVMetrics.newNotifications[ROVMetrics.newNotifications.Count - i];
                    if (notification.GetComponent<UINotification>().type == NotificationType.BATTERY)
                    {
                        Instantiate(ROVMetrics.newNotifications[ROVMetrics.newNotifications.Count - i], notificationParent);
                        activeNotifications++;
                        break;
                    }
                }
                foreach (GameObject notification in ROVMetrics.newNotifications)
                {
                    Instantiate(notification, notificationParent);
                    activeNotifications++;
                    if (activeNotifications >= 3)
                    {
                        return;
                    }
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
