using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIGeneral : MonoBehaviour
{
    public TextMeshProUGUI messageTest;
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
}
