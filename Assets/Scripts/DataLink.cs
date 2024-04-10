using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataLink : MonoBehaviour
{
    // public DataManager dataManager;
    public UIAxisRenderer axisRenderer;
    public UILineRenderer lineRenderer;
    public UIGridRenderer gridRenderer;
    public SpawnNumbers spawnNumbers;

    // Update is called once per frame
    public void CustomUpdate(Vector2 gridSize, float xGrad, float yGrad, List<Vector2> points)
    {
        gridRenderer.CustomUpdate(gridSize);
        lineRenderer.CustomUpdate(gridSize, xGrad, yGrad, points);
        axisRenderer.CustomUpdate(gridSize);
        spawnNumbers.CustomUpdate(gridSize, xGrad, yGrad);
    }
}
