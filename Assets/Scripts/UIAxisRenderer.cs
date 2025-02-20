using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class UIAxisRenderer : Graphic
{
    // Public variables
    public List<Vector2> axisPositionsH = new List<Vector2>();
    public List<Vector2> axisPositionsV = new List<Vector2>();

    public float thickness = 0.03f;

    // Private variables
    private Vector2 gridSize = new Vector2(3, 3);

    private float xOrigin = 0;
    private float yOrigin = 0;

    private float width;
    private float height;
    private float unitWidth;
    private float unitHeight;

    private int count = 0;

    private void Update()
    {
        
    }

    // Update gridSize following UIGridRenderer's gridSize parameter
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

        unitWidth = width / (float)gridSize.x;
        unitHeight = height / (float)gridSize.y;

        count = 0;

        axisPositionsH.Clear();
        for (int i = 0; i < gridSize.x + 1; i++)
        {
            Vector2 point = new Vector2(xOrigin + i * unitWidth, yOrigin);
            axisPositionsH.Add(point);

            DrawRectangleFromPoint(point, 2 * thickness, vh, 0);
        }

        axisPositionsV.Clear();
        for (int i = 0; i < gridSize.y + 1; i++)
        {
            Vector2 point = new Vector2(xOrigin, yOrigin + i * unitHeight);
            axisPositionsV.Add(point);

            DrawRectangleFromPoint(point, 2 * thickness, vh, -90);
        }

        float xEnd = (gridSize.x + 1f) * unitWidth;
        float yEnd = (gridSize.y + 1f) * unitHeight;

        // Then draw axis lines
        DrawRectangleFromPoint(new Vector2(xOrigin, yOrigin), xEnd, vh, 90);
        DrawRectangleFromPoint(new Vector2(xOrigin, yOrigin), yEnd, vh, 180);

        // Then draw axis arrows
        DrawAxisArrows(xEnd + 3 * thickness, yEnd + 3 * thickness, 4 * thickness, vh);
    }

    // Drawing the vertices for one point
    private void DrawRectangleFromPoint(Vector2 point, float length, VertexHelper vh, float angle)
    {
        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = color;

        // Bottom left
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(-thickness, -thickness - length);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Top left
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(-thickness, thickness);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Top right
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(thickness, thickness);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Bottom right
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(thickness, -thickness - length);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Draw rectangle
        int offset = count * 4;
        vh.AddTriangle(offset + 0, offset + 1, offset + 2);
        vh.AddTriangle(offset + 2, offset + 3, offset + 0);
        count++;
    }

    // Drawing the arrows at the end of the axis
    private void DrawAxisArrows(float xEnd, float yEnd, float length, VertexHelper vh)
    {

        // First arrow for horizontal axis
        float angle = 90;
        Vector2 point = new Vector2(xOrigin + xEnd, yOrigin);

        DrawArrowVertices(point, length, vh, angle);

        // Draw rectangle
        int offset = count * 4;
        vh.AddTriangle(offset + 0, offset + 1, offset + 2);

        // Second arrow for vertixal axis
        angle = 180;
        point = new Vector2(xOrigin, yOrigin + yEnd);

        DrawArrowVertices(point, length, vh, angle);

        // Draw rectangle
        offset += 3;
        vh.AddTriangle(offset + 0, offset + 1, offset + 2);
    }

    private void DrawArrowVertices(Vector2 point, float length, VertexHelper vh, float angle)
    {
        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = color;
        float doubleThickness = 2 * thickness;

        // Top left
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(-doubleThickness, doubleThickness);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Top right
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(doubleThickness, doubleThickness);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);

        // Bottom
        vertex.position = Quaternion.Euler(0, 0, angle) * new Vector3(0, -length);
        vertex.position += new Vector3(point.x, point.y);
        vh.AddVert(vertex);
    }
}
