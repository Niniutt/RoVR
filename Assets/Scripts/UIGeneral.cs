using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIGeneral : MonoBehaviour
{
    public TextMeshProUGUI messageTest;
    public TextMeshProUGUI currentBattery;
    public TextMeshProUGUI batteryRemainingEst;
    public TextMeshProUGUI timeTilAscent;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnMessageUpdate(string message)
    {
        messageTest.text = message;
    }
    public void OnBatteryUpdate(float battery)
    {
        currentBattery.text = ((int)battery).ToString() + "%";

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
}
