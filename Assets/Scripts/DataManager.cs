using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;

[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/DataManager", order = 1)]
public class DataManager : ScriptableObject
{
    // Public variables
    public Vector2 gridSize;

    public float gridThickness = 0.03f;
    public float axisThickness = 0.01f;

    public List<Vector2> points;

    public float xGrad;
    public float yGrad;

    private int minPower = -3;
    private int maxPower = 8;

    private int gMax = 10;
    private float[] possibleGraduations = { 1f, 2.5f, 5f };

    public void Init()
    {
        points = new List<Vector2>();

        points.Add(new Vector2(-5f, -5f));
        points.Add(new Vector2(-2f, -2f));
        points.Add(new Vector2(0f, 1f));
        points.Add(new Vector2(1f, 2f));
        points.Add(new Vector2(2f, 4f));
        points.Add(new Vector2(3f, 3f));
        points.Add(new Vector2(4f, 0f));
    }

    public void Update()
    {
        float xMax = points.Max(v => v.x);
        float xMin = points.Min(v => v.x);
        float yMax = points.Max(v => v.y);
        float yMin = points.Min(v => v.y);

        float xDifference = xMax - xMin;
        float yDifference = yMax - yMin;

        int xN;
        int yN;
        CalculateClosestPowerOfTen(xDifference, out xN, out xGrad);
        CalculateClosestPowerOfTen(yDifference, out yN, out yGrad);

        // Update gridSize
        gridSize = new Vector2(xMax / xGrad, yMax / yGrad);

        Debug.Log("x: " + xDifference + " " + xN + " " + xGrad);
        Debug.Log("y: " + yDifference + " " + yN + " " + yGrad);
    }

    // Calculate the closest power of ten, rounding by default
    // Example: 467 -> 2; 587 -> 2; 1760 -> 3
    private void CalculateClosestPowerOfTen(float number, out int closest, out float graduation)
    {
        // Init
        closest = maxPower;
        graduation = 1f;
        // Test for each possible power of 10 (have to know the min and max magnitude of you can find)
        for (int i = minPower; i < maxPower; i++)
        {
            float fraction = number / Mathf.Pow(10, i);
            if (fraction < 1)
            {
                closest = i - 1;
                fraction = number / Mathf.Pow(10, i - 1);
                graduation = CalculateGraduation(closest, fraction);
                if (graduation == 0) Debug.LogException(new System.Exception("Graduation was not found"));
                break;
            }
        }
    }

    // Calculate the graduation that's going to be shown on the graph
    private float CalculateGraduation(int closest, float fraction)
    {
        for (int i = 0; i < possibleGraduations.Length; i++) {
            if (fraction / possibleGraduations[i] > gMax)
            {
                continue;
            }
            else
            {
                return possibleGraduations[i] * Mathf.Pow(10, closest);
            }
        }
        return 0;
    }
}
