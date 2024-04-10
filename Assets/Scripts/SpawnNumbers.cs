using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

// https://docs.unity3d.com/2018.2/Documentation/ScriptReference/UI.Text-text.html
public class SpawnNumbers : MonoBehaviour
{
    public GameObject canvasParent;
    public UIAxisRenderer axisRenderer;

    private enum UpDown { Down = -1, Start = 0, Up = 1 };
    private Text text;
    private Font font;

    private enum Axis { H, V };
    private List<string> names = new List<string>();
    private Vector2 offsetH = new Vector2(-5.25f, -6f);
    private Vector2 offsetV = new Vector2(-6.25f, -5.25f);

    void Awake()
    {
        // Load the Arial font from the Unity Resources folder.
        font = (Font)Resources.GetBuiltinResource(typeof(Font), "LegacyRuntime.ttf");

        // Get canvas from the GameObject.
        Canvas canvas;
        canvas = canvasParent.GetComponent<Canvas>();
        //canvas.renderMode = RenderMode.ScreenSpaceOverlay;
    }

    private void Update()
    {
        
    }

    public void CustomUpdate(Vector2 gridSize, float xGrad, float yGrad)
    {
        // Destroy previous numbers
        foreach (string name in names)
        {
            GameObject go = GameObject.Find(name);
            if (go) Destroy(go.gameObject);
        }

        // Horizontal axis
        for (int i =  0; i < gridSize.x; i++)
        {
            // Add text
            if (i >= axisRenderer.axisPositionsH.Count) continue;
            Vector2 position = axisRenderer.axisPositionsH[i];
            position += offsetH;
            AddText(position, (i * xGrad).ToString("#.00"), Axis.H, i);
        }
        // Vertical axis
        for (int j = 0; j < gridSize.y; j++)
        {
            if (j >= axisRenderer.axisPositionsV.Count) continue;
            Vector2 position = axisRenderer.axisPositionsV[j];
            position += offsetV;
            AddText(position, (j * yGrad).ToString("#.00"), Axis.V, j);
        }
    }

    private void AddText(Vector2 position, string axisText, Axis axis, int index)
    {
        // AddText: position where to add text, what text, which axis

        // Create the Text GameObject.
        GameObject textGO = new GameObject();
        string name = "AxisNumber" + axis.ToString() + index.ToString();
        names.Add(name);
        textGO.name = name;
        textGO.transform.parent = canvasParent.transform;
        // Adjust position
        // textGO.transform.localPosition = new Vector3(position.x, position.y, 0);
        textGO.AddComponent<Text>();

        // Set Text component properties.
        text = textGO.GetComponent<Text>();
        text.font = font;
        text.text = axisText;
        text.fontSize = 48;
        text.alignment = TextAnchor.MiddleCenter;

        // Provide Text position and size using RectTransform.
        RectTransform rectTransform;
        rectTransform = text.GetComponent<RectTransform>();
        rectTransform.localPosition = new Vector3(position.x, position.y, 0);
        rectTransform.sizeDelta = new Vector2(100, 100);
        rectTransform.localScale = new Vector2(0.01f, 0.01f);
    }
}
