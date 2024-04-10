using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIGridRenderer : Graphic
{
    // Public variables
    public float thickness = 0.03f;

    public float xOrigin = 0;
    public float yOrigin = 0;

    // Private variables
    private Vector2 gridSize = new Vector2(3, 3);

    private float width;
    private float height;
    private float cellWidth;
    private float cellHeight;

    private void Update()
    {
        
    }

    // Update function
    public void CustomUpdate(Vector2 gridSize)
    {
        this.gridSize = gridSize;
        
        // Redraw vertices (= setting the vertices as "outdated")
        SetVerticesDirty();
    }

    // Create mesh and draw vertices and triangles
    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        width = rectTransform.rect.width;
        height = rectTransform.rect.height;

        cellWidth = width / gridSize.x;
        cellHeight = height / gridSize.y;

        int count = 0;

        float xLim = gridSize.x;
        float yLim = gridSize.y;

        for (int y = 0; y < yLim; y++) 
        {
            for (int x = 0; x < xLim; x++)
            {
                DrawCell(x, y, count, vh);
                count++;
            }
        }

        
    }

    // Draw rectangle (of unit 1 X 1)
    private void DrawCell(int x, int y, int index, VertexHelper vh)
    {
        float xPos = xOrigin + cellWidth * x;
        float yPos = yOrigin + cellHeight * y;

        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = color;

        // Create first rectangle
        vertex.position = new Vector3(xPos, yPos);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos, yPos + cellHeight);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + cellWidth, yPos + cellHeight);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + cellWidth, yPos);
        vh.AddVert(vertex);

        //vh.AddTriangle(0, 1, 2);
        //vh.AddTriangle(2, 3, 0);

        // Inside rectangle
        float widthSqr = thickness * thickness;
        float distanceSqr = widthSqr * 2f;
        float distance = Mathf.Sqrt(distanceSqr);

        vertex.position = new Vector3(xPos + distance, yPos + distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + distance, yPos + cellHeight - distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + cellWidth - distance, yPos + cellHeight - distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + cellWidth - distance, yPos + distance);
        vh.AddVert(vertex);

        // Draw rectangle into rectangle
        int offset = index * 8;

        // Left Edge
        vh.AddTriangle(offset + 0, offset + 1, offset + 5);
        vh.AddTriangle(offset + 5, offset + 4, offset + 0);

        // Top Edge
        vh.AddTriangle(offset + 1, offset + 2, offset + 6);
        vh.AddTriangle(offset + 6, offset + 5, offset + 1);

        // Right Edge
        vh.AddTriangle(offset + 2, offset + 3, offset + 7);
        vh.AddTriangle(offset + 7, offset + 6, offset + 2);

        // Bottom Edge
        vh.AddTriangle(offset + 3, offset + 0, offset + 4);
        vh.AddTriangle(offset + 4, offset + 7, offset + 3);
    }
}
