using UnityEngine;
using System;
using System.Net.Sockets;
using System.IO;
using System.Threading;

public class Ros2Receiver : MonoBehaviour
{
    public Rover rover;
    private TcpClient client;
    private StreamReader reader;
    private Thread receiveThread;
    private bool connected = false;

    private float linearVel = 0f;
    private float angularVel = 0f;

    void Start()
    {
        ConnectToROS();
    }

    void ConnectToROS()
    {
        try
        {
            client = new TcpClient("127.0.0.1", 9090); // same port as ROS2
            reader = new StreamReader(client.GetStream());
            connected = true;
            receiveThread = new Thread(ReceiveData);
            receiveThread.Start();
            Debug.Log("Connected to ROS2 TCP server");
        }
        catch (Exception e)
        {
            Debug.LogError("Connection failed: " + e.Message);
        }
    }

    void ReceiveData()
    {
        while (connected)
        {
            try
            {
                string data = reader.ReadLine();
                if (string.IsNullOrEmpty(data)) continue;

                var json = JsonUtility.FromJson<ROSData>(data);
                linearVel = json.linear_x;
                angularVel = json.angular_z;
            }
            catch (Exception e)
            {
                Debug.LogError("Error receiving: " + e.Message);
                connected = false;
            }
        }
    }

    void Update()
    {
        // Apply to rover movement
        if (rover != null)
        {
            rover.Move(linearVel, angularVel);
        }
    }

    [Serializable]
    public class ROSData
    {
        public float linear_x;
        public float angular_z;
    }

    void OnApplicationQuit()
    {
        connected = false;
        reader?.Close();
        client?.Close();
        receiveThread?.Abort();
    }
}
