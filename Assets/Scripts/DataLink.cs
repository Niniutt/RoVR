using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataLink : MonoBehaviour
{
    public DataManager dataManager;

    // Start is called before the first frame update
    void Start()
    {
        dataManager.Init();
    }

    // Update is called once per frame
    void Update()
    {
        // Listener for changes of data?
        dataManager.Update();
    }
}
