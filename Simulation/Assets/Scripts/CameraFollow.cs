using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;   // The rover to follow
    public Vector3 offset = new Vector3(0f, 5f, -10f);  // Camera position offset
    public float smoothSpeed = 5f;  // How smoothly the camera follows

    void LateUpdate()
    {
        if (target == null) return;

        // Desired position based on rover + offset
        Vector3 desiredPosition = target.position + offset;

        // Smoothly interpolate between current and desired position
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);

        // Apply position
        transform.position = smoothedPosition;

        // Always look at the rover
        transform.LookAt(target);
    }
}
