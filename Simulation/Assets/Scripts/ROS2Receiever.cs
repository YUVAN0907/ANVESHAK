using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;


public class ROS2Receiever : MonoBehaviour
{
/// <summary>
/// Receives movement commands from ROS2 and sends them to Rover.cs
/// </summary>
    public Rover rover; // drag your Rover object here in Inspector

    void Start()
    {
        // Subscribe to a custom topic from ROS2
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("/rover_control", CommandCallback);
        Debug.Log("🛰️ Subscribed to /rover_control topic");
    }

    void CommandCallback(StringMsg msg)
    {
        string command = msg.data.ToLower();
        Debug.Log($"🛰️ Received command: {command}");

        switch (command)
        {
            case "forward":
                rover.Move(1f, 0f);
                break;
            case "backward":
                rover.Move(-1f, 0f);
                break;
            case "left":
                rover.Move(0.5f, -1f);
                break;
            case "right":
                rover.Move(0.5f, 1f);
                break;
            case "stop":
                rover.Move(0f, 0f);
                break;
            default:
                Debug.LogWarning($"⚠️ Unknown command: {command}");
                break;
        }
    }
}

