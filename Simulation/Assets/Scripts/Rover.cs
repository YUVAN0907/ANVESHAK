using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rover : MonoBehaviour
{
    [Header("Physics")]
    public Rigidbody rigid;
    public WheelCollider wheel1, wheel2, wheel3, wheel4;
    public float moonGravity = -1.62f;   // Moon gravity (m/s²)
    public float driveSpeed = 80f;       // Adjusted drive power
    public float steerSpeed = 25f;
    public float traction = 0.6f;        // Lower = more drift
    public float slopeAssist = 1.5f;     // Extra torque uphill

    [Header("Control Mode")]
    bool useROSControl = false;
    float rosLinear, rosAngular;
    float horizontalInput, verticalInput;

    void Start()
    {
        // 🌕 Apply Moon gravity
        Physics.gravity = new Vector3(0, moonGravity, 0);

        // ⚙️ Lower center of mass for stability
        rigid.centerOfMass = new Vector3(0, -0.5f, 0);

        // 🛞 Improve wheel suspension & traction
        SetupWheelPhysics(wheel1);
        SetupWheelPhysics(wheel2);
        SetupWheelPhysics(wheel3);
        SetupWheelPhysics(wheel4);
    }

    void Update()
    {
        // Keyboard fallback control
        if (!useROSControl)
        {
            horizontalInput = Input.GetAxis("Horizontal");
            verticalInput = Input.GetAxis("Vertical");
        }
    }

    void FixedUpdate()
    {
        // Select control mode
        float motorInput = useROSControl ? rosLinear : verticalInput;
        float steerInput = useROSControl ? rosAngular : horizontalInput;

        // Compute motor torque
        float motor = motorInput * driveSpeed;
        float steer = steerInput * steerSpeed;

        // 🧭 Adjust torque based on slope angle (helps climbing)
        float slopeFactor = Mathf.Clamp01(Vector3.Dot(transform.up, Vector3.up));
        motor *= Mathf.Lerp(slopeAssist, 0.7f, slopeFactor);

        // Apply torque & steering
        wheel1.motorTorque = motor;
        wheel2.motorTorque = motor;
        wheel3.motorTorque = motor;
        wheel4.motorTorque = motor;

        wheel1.steerAngle = steer;
        wheel2.steerAngle = steer;

        // 🪶 Simulate low traction drifting
        Vector3 localVel = transform.InverseTransformDirection(rigid.linearVelocity);
        localVel.x *= traction;
        rigid.linearVelocity = transform.TransformDirection(localVel);
    }

    // Called externally by your ROS2 bridge
    public void Move(float linear, float angular)
    {
        rosLinear = linear;
        rosAngular = angular;
        useROSControl = true;
    }

    // Switch back to manual mode
    public void UseManualControl()
    {
        useROSControl = false;
    }

    // 🔧 Helper: setup suspension & friction for Moon terrain
    void SetupWheelPhysics(WheelCollider wheel)
    {
        JointSpring spring = wheel.suspensionSpring;
        spring.spring = 50000f;
        spring.damper = 4500f;
        wheel.suspensionSpring = spring;
        wheel.suspensionDistance = 0.4f;

        WheelFrictionCurve forwardFriction = wheel.forwardFriction;
        forwardFriction.stiffness = 0.8f;

        WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;
        sidewaysFriction.stiffness = 0.6f;

        wheel.forwardFriction = forwardFriction;
        wheel.sidewaysFriction = sidewaysFriction;
    }
}
