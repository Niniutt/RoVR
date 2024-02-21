using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using System.Collections.Generic;
using UnityEngine.UIElements;
using System;

public class TCPServer : MonoBehaviour
{
    Thread thread;
    public int connectionPort = 25001;
    TcpListener server;
    TcpClient client;
    bool running;

    // Position is the data being received in this example
    string[] message;
    List<Vector3> positions = new List<Vector3>();

    void Start()
    {
        // Receive on a separate thread so Unity doesn't freeze waiting for data
        ThreadStart ts = new ThreadStart(GetData);
        thread = new Thread(ts);
        thread.Start();
    }

    void GetData()
    {
        // Create the server
        server = new TcpListener(IPAddress.Any, connectionPort);
        server.Start();

        // Create a client to get the data stream
        client = server.AcceptTcpClient();

        // Start listening
        running = true;
        while (running)
        {
            Connection();
        }
        server.Stop();
    }

    void Connection()
    {
        // Read data from the network stream
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);

        // Decode the bytes into a string
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead);

        // Make sure we're not getting an empty string
        //dataReceived.Trim();
        if (dataReceived != null && dataReceived != "")
        {
            // Convert the received string of data to the format we are using
            message = ParseData(dataReceived);

            nwStream.Write(buffer, 0, bytesRead);
        }
    }

    public string[] ParseData(string dataString)
    {
        // Show data before parsing
        Debug.Log(dataString);

        // Split the dataString into substrings
        string[] substrings = dataString.Split(new string[] { "], " }, StringSplitOptions.None);

        // Process each substring
        foreach (string substring in substrings)
        {
            // Remove any remaining brackets
            string trimmedSubstring = substring.Replace("[", "").Replace("]", "");

            // Split the substring into key-value pairs
            string[] keyValue = trimmedSubstring.Split(':');

            string key = keyValue[0].Trim();
            string value = keyValue[1].Trim();

            // Display or process key-value pairs as needed
            Debug.Log($"Key: {key}, Value: {value}");
            QueueData(key, value);
        }

        return substrings;
    }

    void Update()
    {
        // Set this object's position in the scene according to the position received
        //transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y + rotation.y, transform.rotation.eulerAngles.z + rotation.z);
        UpdatePosition();
    }

    private void UpdatePosition()
    {
        for (int i = 0; i < positions.Count; i++) 
        { 
            transform.position += positions[i];
        }
        positions.Clear();
    }

    private void QueueData(string key, string value) 
    {
        switch (key)
        {
            case "PositionDelta" : positions.Add(parseVector(value)); break;
            default : break;
        }
    }

    private Vector3 parseVector(string valueString)
    {
        string[] stringArray = valueString.Split(',');
        if (stringArray.Length != 3)
            throw new Exception("Data Mismatch");

        return new Vector3(
            float.Parse(stringArray[0]),
            float.Parse(stringArray[1]),
            float.Parse(stringArray[2])
            );
    }
}