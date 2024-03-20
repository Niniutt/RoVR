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
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnDepthUpdate(float depthValue)
    {
        depth.text = depthValue.ToString() + "m";
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
}
