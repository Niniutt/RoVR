using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIGridRenderer : Graphic
{
    public Vector2Int gridSize = new Vector2Int(1, 1);
    public float thickness = 10.0f;

    float width;
    float height;
    float cellWidth;
    float cellHeight;

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        width = rectTransform.rect.width;
        height = rectTransform.rect.height;

        cellWidth = width / (float)gridSize.x;
        cellHeight = height / (float)gridSize.y;

        int count = 0;

        for (int y = 0; y < gridSize.y; y++) 
        {
            for (int x = 0; x < gridSize.x; x++)
            {
                DrawCell(x, y, count, vh);
                count++;
            }
        }

        
    }

    private void DrawCell(int x, int y, int index, VertexHelper vh)
    {
        float xPos = cellWidth * x;
        float yPos = cellHeight * y;

        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = color;

        // Create first rectangle
        vertex.position = new Vector3(xPos, yPos);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos, yPos + height);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + width, yPos + height);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + width, yPos);
        vh.AddVert(vertex);

        //vh.AddTriangle(0, 1, 2);
        //vh.AddTriangle(2, 3, 0);

        // Inside rectangle
        float widthSqr = thickness * thickness;
        float distanceSqr = widthSqr * 2f;
        float distance = Mathf.Sqrt(distanceSqr);

        vertex.position = new Vector3(xPos + distance, yPos + distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + distance, yPos + height - distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + width - distance, yPos + height - distance);
        vh.AddVert(vertex);

        vertex.position = new Vector3(xPos + width - distance, yPos + distance);
        vh.AddVert(vertex);

        // Draw rectangle into rectangle

        int offset = index * 8;

        // Left Edge
        vh.AddTriangle(offset + 0, offset + 1, offset + 5);
        vh.AddTriangle(offset + 5, offset + 4, offset + 0);

        // Top Edge
        vh.AddTriangle(1, 2, 6);
        vh.AddTriangle(offset + 6, offset + 5, offset + 1);

        // Right Edge
        vh.AddTriangle(offset + 2, offset + 3, offset + 7);
        vh.AddTriangle(offset + 7, offset + 6, offset + 2);

        // Bottom Edge
        vh.AddTriangle(offset + 3, offset + 0, offset + 4);
        vh.AddTriangle(offset + 4, offset + 7, offset + 3);
    }
}
