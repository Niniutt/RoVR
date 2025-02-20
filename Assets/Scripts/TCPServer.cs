using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using System.Collections.Generic;
using System;
using System.Collections.Concurrent;
using Unity.Collections;
using System.Drawing;

public class TCPServer : MonoBehaviour
{
    Thread thread;
    public int connectionPort = 25001;
    TcpListener server;
    TcpClient client;
    bool running;

    // Position is the data being received in this example
    private DataObject data;
    ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();


    public GameObject UIParent;
    public GameObject ROVModel;
    void Start()
    {
        if (UIParent == null)
            UIParent = GameObject.Find("UI");
        if (ROVModel == null)
            ROVModel = GameObject.Find("ROVModel");
        if (data == null)
            data = ScriptableObject.CreateInstance<DataObject>();

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
        dataReceived.Trim();
        if (dataReceived != null && dataReceived != "")
        {
            // Convert the received string of data to the format we are using
            //message = ParseData(dataReceived);
            messageQueue.Enqueue(dataReceived);

            nwStream.Write(buffer, 0, bytesRead);
        }
    }

    public void ParseData()
    {
        // Show data before parsing
        //Debug.Log(dataString);
        while (messageQueue.TryDequeue(out var message))
        {
            string data = message.Replace("{", "").Replace("}", "");

            // Split the dataString into substrings
            string[] substrings = data.Split(new string[] { "]," }, StringSplitOptions.None);
            //Debug.Log("Substings:"+ substrings.ToString());

            // Process each substring
            foreach (string substring in substrings)
            {
                string trimmedSubstring = substring.Replace("[", "").Replace("]", "");
                //Debug.Log($"Trimmed: {trimmedSubstring}");

                string[] keyValue = trimmedSubstring.Split(':');

                string key = keyValue[0].Trim();
                string value = keyValue[1].Trim();

                // Display or process key-value pairs as needed
                //Debug.Log($"Key: {key}, Value: {value}");
                QueueData(key, value);
            }
        }
    }

    void Update()
    {
        // Set this object's position in the scene according to the position received
        //transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y + rotation.y, transform.rotation.eulerAngles.z + rotation.z);
        ParseData();
        List<Vector3> positionCopy = new List<Vector3>();
        foreach (Vector3 position in data.positionDelta)
        {
            positionCopy.Add(position);
        }
        data.positionDelta.RemoveAll(x => positionCopy.Contains(x));

        UpdatePosition(positionCopy);
    }

    private void UpdatePosition(List<Vector3> pos)
    {
        foreach (var vec in pos)
        {
            transform.position += vec;
        }
    }

    private void QueueData(string key, string value) 
    {
        //Debug.Log($"Key: {key}, Value: {value}");
        switch (key)
        {
            case "time": 
            {
                DataObject.time = float.Parse(value); 
                UIParent.BroadcastMessage("OnTimeUpdate", DataObject.time, SendMessageOptions.DontRequireReceiver);
                break;    
            }
            case "battery":
            {
                data.battery = float.Parse(value); 
                DataObject.batteryLog.Add(data.battery);
                UIParent.BroadcastMessage("OnBatteryUpdate", data.battery, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "battery_lifetime":
            {
                data.batteryLifetime = float.Parse(value);
                UIParent.BroadcastMessage("OnBatteryLifetimeUpdate", data.batteryLifetime, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "time_before_ascent":
            {
                data.time_before_ascent = float.Parse(value);
                UIParent.BroadcastMessage("OnTimeBeforeAscentUpdate", data.time_before_ascent, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "pressure":
            {
                data.pressure = float.Parse(value) / 101325;
                UIParent.BroadcastMessage("OnPressureUpdate", data.pressure, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "depth":
            {
                data.depth = float.Parse(value); 
                data.depthLog.Add(data.depth);
                UIParent.BroadcastMessage("OnDepthUpdate", data.depth, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "orientation":
            {
                data.orientation = parseVector4(value);
                ROVModel.BroadcastMessage("OnOrientationUpdate", data.orientation, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "angular_velocity":
            {
                data.angularVelocity = parseVector3(value);
                ROVModel.BroadcastMessage("OnAngularVelocityUpdate", data.angularVelocity, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "linear_acceleration":
            {
                data.linearAcceleration = parseVector3(value);
                ROVModel.BroadcastMessage("OnLinearAccelerationUpdate", data.linearAcceleration, SendMessageOptions.DontRequireReceiver);
                break;
            }
            case "doppler_velocity":
            {
                data.dopplerVelocity = parseVector3(value); 
                data.velocityLog.Add(data.dopplerVelocity.magnitude);
                ROVModel.BroadcastMessage("OnDopplerVelocityUpdate", data.dopplerVelocity, SendMessageOptions.DontRequireReceiver);
                break;
            }
            default : break;
        }
    }

    private Vector3 parseVector3(string valueString)
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
    private Vector4 parseVector4(string valueString)
    {
        string[] stringArray = valueString.Split(',');
        if (stringArray.Length != 4)
            throw new Exception("Data Mismatch");

        return new Vector4(
            float.Parse(stringArray[0]),
            float.Parse(stringArray[1]),
            float.Parse(stringArray[2]),
            float.Parse(stringArray[3])
            );
    }
}