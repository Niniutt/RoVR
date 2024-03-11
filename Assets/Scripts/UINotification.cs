using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;


public enum NotificationType
{
    WARNING, ALERT// INFO, ERROR
}
public class UINotification : MonoBehaviour
{
    [SerializeField]
    private Image icon;
    [SerializeField]
    private string title = "Title";
    [SerializeField]
    private string description = "Description";
    private TextMeshProUGUI textField;

    Color colorAlert = new Color32(0xFF, 0xC8, 0x00, 0xFB);
    Color colorWarning = new Color32(0xFF, 0xC8, 0x00, 0x00);

    // Start is called before the first frame update
    void Start()
    {
        if (textField == null)
            textField = GetComponentInChildren<TextMeshProUGUI>();

        if (icon == null)
            icon = GetComponentInChildren<Image>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void setNotification(string title, string desctiption, NotificationType type)
    {
        this.title = title;
        this.description = desctiption;
        textField.text = desctiption;

        switch (type)
        {
            case NotificationType.WARNING: icon.color = colorWarning; break;
            case NotificationType.ALERT: icon.color = colorAlert; break;

        }
    }
}
