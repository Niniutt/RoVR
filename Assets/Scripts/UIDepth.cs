using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIDepth : MonoBehaviour
{
    public TextMeshProUGUI depth;
    public TextMeshProUGUI pressure;
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
        pressure.text = pressureValue.ToString() + "Pa";

    }
}
