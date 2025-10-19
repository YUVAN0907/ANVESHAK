using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class CubeOdomPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string odomTopic = "/odom";
    public float publishRate = 10f;
    private float timeSinceLastPublish = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Register the publisher explicitly to avoid the "No registered publisher" error
        ros.RegisterPublisher<OdometryMsg>(odomTopic);
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish >= 1.0f / publishRate)
        {
            PublishOdom();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishOdom()
    {
        var odomMsg = new OdometryMsg();
        odomMsg.pose.pose.position.x = transform.position.x;
        odomMsg.pose.pose.position.y = transform.position.y;
        odomMsg.pose.pose.position.z = transform.position.z;

        odomMsg.pose.pose.orientation.x = transform.rotation.x;
        odomMsg.pose.pose.orientation.y = transform.rotation.y;
        odomMsg.pose.pose.orientation.z = transform.rotation.z;
        odomMsg.pose.pose.orientation.w = transform.rotation.w;

        ros.Publish(odomTopic, odomMsg);
    }
}
