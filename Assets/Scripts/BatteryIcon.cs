using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BatteryIcon : MonoBehaviour
{
    public ROVMetrics metrics;
    public Image batteryIcon;
    public Slider fill;
    private Color batteryNormal = new Color32(1, 224, 0, 1); // Light green
    private Color batteryLow = Color.yellow;
    private Color batteryCritical = Color.red;
    // Start is called before the first frame update
    void Start()
    {
        if (metrics == null)
        {
            metrics = ScriptableObject.CreateInstance<ROVMetrics>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnBatteryUpdate(float value)
    {
        fill.value = value/100;
        if (value > metrics.batteryWarningThreshold)
        {
            batteryIcon.color = batteryNormal;
        }
        else if (value > metrics.batteryAlertThreshold)
        {
            batteryIcon.color = batteryLow;
        }
        else
        {
            batteryIcon.color = batteryCritical;
        }
    }
}
