using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;


public enum NotificationType
{
    BATTERY, DEPTH// INFO, ERROR
}
public enum NotificationLevel
{
    WARNING, ALERT// INFO, ERROR
}
public class UINotification : MonoBehaviour
{
    [SerializeField]
    private Image icon;
    [SerializeField]
    public string title = "Title";
    [SerializeField]
    public string description = "Description";
    private TextMeshProUGUI textField;
    public NotificationType type;
    public NotificationLevel level;

    private int notificationID;
    [SerializeField]
    private static int notificationIDCounter = 0;

    Color colorAlert = new Color32(0xFF, 0xC8, 0x00, 0xFF);
    Color colorWarning = new Color32(0xFF, 0xC8, 0x00, 0xFF);

    // Start is called before the first frame update
    void Start()
    {
        if (textField == null)
            textField = GetComponentInChildren<TextMeshProUGUI>();

        if (icon == null)
            icon = GetComponentInChildren<Image>();

        this.notificationID = notificationIDCounter++;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void setNotification(string title, string desctiption, NotificationType type, NotificationLevel level)
    {
        if (textField == null)
            textField = GetComponentInChildren<TextMeshProUGUI>();

        if (icon == null)
            icon = GetComponentInChildren<Image>();

        this.title = title;
        this.description = desctiption;
        textField.text = desctiption;
        this.type = type;
        this.level = level;
        switch (level)
        {
            case NotificationLevel.WARNING: icon.color = colorWarning; break;
            case NotificationLevel.ALERT: icon.color = colorAlert; break;
        }
    }
}
