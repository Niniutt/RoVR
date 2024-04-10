using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;
using Text = UnityEngine.UI.Text;

// https://docs.unity3d.com/2018.2/Documentation/ScriptReference/UI.Text-text.html
public class SpawnNumbers : MonoBehaviour
{
    public GameObject canvasParent;
    public UIAxisRenderer axisRenderer;
    public string identificator = "AxisNumber";

    private enum UpDown { Down = -1, Start = 0, Up = 1 };
    private Text text;
    private Font font;

    private int gMax = 10;

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

        // Create textGOs
        names.Clear();
        for (int i = 0; i < gMax; i++)
        {
            string name;
            name = identificator + Axis.H.ToString() + i.ToString();
            CreateTextGO(name);
            name = identificator + Axis.V.ToString() + i.ToString();
            CreateTextGO(name);
        }
    }

    private void Update()
    {
        
    }

    public void CustomUpdate(Vector2 gridSize, float xGrad, float yGrad)
    {
        // Deactivate surnumerous numbers
        names.Clear();
        for (int i = (int)gridSize.x; i < gMax; i++)
        {
            string name = identificator + Axis.H.ToString() + i.ToString();
            names.Add(name);
        }
        for (int i = (int)gridSize.y; i < gMax; i++)
        {
            string name = identificator + Axis.V.ToString() + i.ToString();
            names.Add(name);
        }
        foreach (string name in names)
        {
            GameObject go = GameObject.Find(name);
            if (go) { go.GetComponent<Text>().text = ""; };
        }

        // Horizontal axis
        for (int i =  0; i < gridSize.x && i < gMax; i++)
        {
            // Add text
            if (i >= axisRenderer.axisPositionsH.Count) continue;
            Vector2 position = axisRenderer.axisPositionsH[i];
            position += offsetH;
            AddText(position, (i * xGrad).ToString("#.00"), Axis.H, i);
        }
        // Vertical axis
        for (int j = 0; j < gridSize.y && j < gMax; j++)
        {
            if (j >= axisRenderer.axisPositionsV.Count) continue;
            Vector2 position = axisRenderer.axisPositionsV[j];
            position += offsetV;
            AddText(position, (j * yGrad).ToString("#.00"), Axis.V, j);
        }
    }

    // AddText: position where to add text, what text, which axis
    private void AddText(Vector2 position, string axisText, Axis axis, int index)
    {
        // Find the Text GameObject.
        string name = identificator + axis.ToString() + index.ToString();
        GameObject textGO = GameObject.Find(name);
        if (textGO)
        {
            // textGO.SetActive(true);

            // Modify textGO's position and text
            text = textGO.GetComponent<Text>();
            text.text = axisText;

            RectTransform rectTransform;
            rectTransform = text.GetComponent<RectTransform>();
            rectTransform.localPosition = new Vector3(position.x, position.y, 0);
        }
        else
        {
            Debug.Log("TextGO " + name + " not found");
        }
    }

    void CreateTextGO(string name)
    {
        GameObject textGO = new GameObject();
        textGO.name = name;

        textGO.transform.parent = canvasParent.transform;
        textGO.AddComponent<Text>();

        // Set Text component properties.
        text = textGO.GetComponent<Text>();
        text.font = font;
        // text.text = axisText;
        text.fontSize = 48;
        text.alignment = TextAnchor.MiddleCenter;

        // Provide Text position and size using RectTransform.
        RectTransform rectTransform;
        rectTransform = text.GetComponent<RectTransform>();
        rectTransform.localPosition = new Vector3(0, 0, 0);
        rectTransform.sizeDelta = new Vector2(100, 100);
        rectTransform.localScale = new Vector2(0.01f, 0.01f);
    }
}
