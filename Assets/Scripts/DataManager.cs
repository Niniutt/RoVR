using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;

public class DataManager : MonoBehaviour
{
    // Public variables
    public DataLink dataLink;

    public Vector2 gridSize;

    public float gridThickness = 0.03f;
    public float axisThickness = 0.01f;

    public List<Vector2> points = new List<Vector2>
    {
        new Vector2(0f, 0f),
        new Vector2(1f, 1f)
    };

    public float xGrad;
    public float yGrad;

    private int minPower = -3;
    private int maxPower = 8;

    private int gMax = 10;
    private float[] possibleGraduations = { 1f, 2.5f, 5f };

    public void Start()
    {
        points = new List<Vector2>{};

        possibleGraduations = new float[(maxPower - minPower + 1) * 3];
        for (int i = 0; i < maxPower - minPower + 1; i++)
        {
            possibleGraduations[3 * i + 0] = 1f * Mathf.Pow(10, minPower + i);
            possibleGraduations[3 * i + 1] = 2.5f * Mathf.Pow(10, minPower + i);
            possibleGraduations[3 * i + 2] = 5f * Mathf.Pow(10, minPower + i);
        }
    }

    private void Update()
    {
        // Nothing
    }

    public void CustomUpdate(Vector2 newPoint)
    {
        points.Add(newPoint);

        float xMax = points.Max(v => v.x);
        float xMin = points.Min(v => v.x);
        float yMax = points.Max(v => v.y);
        float yMin = 0; // points.Min(v => v.y);

        float xDifference = xMax - xMin;
        float yDifference = yMax - yMin;

        int xN;
        int yN;
        CalculateClosestPowerOfTen(xDifference, out xN, out xGrad);
        CalculateClosestPowerOfTen(yDifference, out yN, out yGrad);
        // Debug.Log("Here: " + yDifference + " " + yN + " " + yGrad);

        // Update gridSize
        gridSize = new Vector2(xMax / xGrad, yMax / yGrad);

        dataLink.CustomUpdate(gridSize, xGrad, yGrad, points);
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
                graduation = CalculateGraduation(closest, number);
                if (graduation == 0) Debug.LogException(new System.Exception("Graduation was not found"));
                break;
            }
        }
    }

    // Calculate the graduation that's going to be shown on the graph
    private float CalculateGraduation(int closest, float number)
    {
        for (int i = 0; i < possibleGraduations.Length; i++)
        {
            // Debug.Log("number: " + number + "; possiblegrad: " + possibleGraduations[i] + "; closest: " + closest);
            if (number / possibleGraduations[i] > gMax)
            {
                continue;
            }
            else
            {
                return possibleGraduations[i];//  * Mathf.Pow(10, closest);
            }
        }
        return 0;
    }
}
