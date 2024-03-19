using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIGeneral : MonoBehaviour
{
    public TextMeshProUGUI messageTest;
    public TextMeshProUGUI batteryRemaining;
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
        batteryRemaining.text = battery.ToString() + "%";

    }
}
