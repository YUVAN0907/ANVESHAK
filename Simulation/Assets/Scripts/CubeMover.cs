using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CubeMover : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    public float speed = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, MoveCube);
    }

    void MoveCube(TwistMsg msg)
    {
        Vector3 move = new Vector3((float)msg.linear.x, 0, (float)msg.linear.y);
        transform.Translate(move * speed * Time.deltaTime);
    }
}
