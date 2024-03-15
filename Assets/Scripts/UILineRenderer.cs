using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class UILineRenderer : Graphic
{
    // Public variables
    public DataLink dataLink;

    public Vector2 gridSize;
    public UIGridRenderer gridRenderer;

    public float thickness = 0.03f;

    public List<Vector2> points;
    public float xMax;
    public float yMax;

    // Private variables
    private float xOrigin = 0;
    private float yOrigin = 0;

    private float width;
    private float height;
    private float unitWidth;
    private float unitHeight;

    // Update gridSize and points
    private void Update()
    {
        // Update origin
        if (gridRenderer != null)
        {
            xOrigin = gridRenderer.xOrigin;
            yOrigin = gridRenderer.yOrigin;
        }
        // Update gridSize
        if (dataLink != null)
        {
            if (gridSize != dataLink.dataManager.gridSize)
            {
                gridSize = dataLink.dataManager.gridSize;

                // Redraw vertices (= setting the vertices as "outdated")
                SetVerticesDirty();
            }
        }
    }

    // Create mesh and draw vertices and triangles
    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        width = rectTransform.rect.width;
        height = rectTransform.rect.height;

        xMax = points.Max(v => v.x);
        yMax = points.Max(v => v.y);

        // Closest power of ten?
        //CalculateClosestPowerOfTen(xMax);
        //CalculateClosestPowerOfTen(yMax);

        unitWidth = width / (float)gridSize.x;
        unitHeight = height / (float)gridSize.y;

        if (points.Count < 2) return;

        float angle = 0;

        for (int i = 0; i < points.Count; i++)
        {
            Vector2 point = points[i];

            if (i < points.Count - 1) angle = GetAngle(points[i], points[i + 1]);

            DrawVerticesForPoint(point, vh, angle);
        }

        for (int i = 0; i < points.Count - 1; i++)
        {
            int index = i * 2;
            vh.AddTriangle(index + 0, index + 1, index + 3);
            vh.AddTriangle(index + 3, index + 2, index + 0);
        }
    }

    // Get angle for rotation of the point vertices
    public float GetAngle(Vector2 me, Vector2 target)
    {
        return (float)(Mathf.Atan2(target.y - me.y, target.x - me.x) * (180 / Mathf.PI) + 45f);
    }

    // Drawing the vertices for one point
    private void DrawVerticesForPoint(Vector2 point, VertexHelper vh, float angle)
    {
        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = color;

        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(- thickness / 2, 0);
        vertex.position += new Vector3(xOrigin + unitWidth * point.x, yOrigin + unitHeight * point.y);
        vh.AddVert(vertex);

        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(thickness / 2, 0);
        vertex.position += new Vector3(xOrigin + unitWidth * point.x, yOrigin + unitHeight * point.y);
        vh.AddVert(vertex);
    }
}
