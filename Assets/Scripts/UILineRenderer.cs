using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class UILineRenderer : Graphic
{
    // Public variables
    public float thickness = 0.03f;

    // Private variables
    private Vector2 gridSize = new Vector2(3, 3);
    private List<Vector2> points;
    private float xGrad = 1;
    private float yGrad = 1;

    private float xOrigin = 0;
    private float yOrigin = 0;

    private float width;
    private float height;
    private float unitWidth;
    private float unitHeight;

    private void Update()
    {
        
    }

    // Update gridSize and points
    public void CustomUpdate(Vector2 gridSize, float xGrad, float yGrad, List<Vector2> points)
    {
        this.gridSize = gridSize;

        this.xGrad = xGrad;
        this.yGrad = yGrad;

        this.points = points;

        // Redraw vertices (= setting the vertices as "outdated")
        SetVerticesDirty();
    }

    // Create mesh and draw vertices and triangles
    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        width = rectTransform.rect.width;
        height = rectTransform.rect.height;

        if (points == null || points.Count == 0) {
            points = new List<Vector2>
            {
                new Vector2(0f, 0f),
                new Vector2(1f, 1f)
            };
        }

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
        if (xGrad == 0) xGrad = 1;
        if (yGrad == 0) yGrad = 1;

        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(- thickness / 2, 0);
        vertex.position += new Vector3(xOrigin + unitWidth * point.x / xGrad, yOrigin + unitHeight * point.y / yGrad);
        vh.AddVert(vertex);

        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(thickness / 2, 0);
        vertex.position += new Vector3(xOrigin + unitWidth * point.x / xGrad, yOrigin + unitHeight * point.y / yGrad);
        vh.AddVert(vertex);
    }
}
