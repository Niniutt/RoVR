using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataLink : MonoBehaviour
{
    public DataManager dataManager;
    public UIAxisRenderer axisRenderer;
    public UILineRenderer lineRenderer;
    public UIGridRenderer gridRenderer;

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
        Vector2 gridSize = dataManager.gridSize;

        axisRenderer.gridSize = gridSize;
        lineRenderer.gridSize = gridSize;
        gridRenderer.gridSize = gridSize;

        lineRenderer.points = dataManager.points;
    }
}
