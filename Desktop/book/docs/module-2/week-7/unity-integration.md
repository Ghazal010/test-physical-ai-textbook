---
title: Week 7 - Unity Integration
description: Integrating Unity with ROS 2 for high-fidelity rendering and simulation
sidebar_position: 1
---

# Week 7: Unity Integration

## Learning Objectives
- Use Unity for high-fidelity rendering in robotics simulation
- Simulate human-robot interactions in Unity environments
- Integrate Unity with ROS 2 for bidirectional communication
- Create realistic environments in Unity for robotics applications
- Implement multi-sensor fusion in Unity-based simulation

## Prerequisites Check
- Basic understanding of Unity development
- Knowledge of ROS 2 communication patterns
- Experience with 3D modeling and rendering concepts

## Theoretical Concepts: Unity-ROS Integration

### Introduction to Unity for Robotics

Unity is a powerful 3D development platform that can be leveraged for high-fidelity robotics simulation and visualization. Unlike physics-focused simulators like Gazebo, Unity excels at:

- **High-quality rendering**: Photorealistic visuals for perception tasks
- **Complex environments**: Detailed scenes with realistic lighting and materials
- **Human interaction**: Natural interfaces for human-robot interaction studies
- **Cross-platform deployment**: Runs on multiple platforms including VR/AR

### Unity-ROS Bridge Architecture

The integration between Unity and ROS 2 typically involves:

1. **ROS TCP Connector**: Establishes communication between Unity and ROS 2
2. **Message Serialization**: Converts Unity data structures to ROS message formats
3. **Asynchronous Communication**: Non-blocking message passing for real-time performance
4. **Transform Synchronization**: Maintains consistent coordinate systems between environments

### Use Cases for Unity Integration

- **Perception Training**: Generate synthetic data with photorealistic quality
- **Human-Robot Interaction**: Natural interfaces for studying social robotics
- **Operator Interfaces**: Advanced visualization and control interfaces
- **Digital Twins**: High-fidelity representations of real-world environments

## Step-by-Step Tutorials: Unity-ROS Integration

### Tutorial 1: Setting Up Unity-ROS Bridge

First, let's understand the setup process for Unity-ROS integration. While we can't install Unity here, we'll cover the essential concepts and configuration:

The most common approach is using the **Unity Robotics Hub** which includes:
- ROS-TCP-Connector: For communication
- Unity-Robotics-Helpers: For message conversion
- Sample environments and examples

### Tutorial 2: Basic Unity-ROS Communication

Here's how to implement basic communication in Unity using C#:

```csharp
// File: Assets/Scripts/RobotController.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private string rosIPAddress = "127.0.0.1";
    [SerializeField]
    private int rosPort = 10000;

    private ROSConnection ros;
    private float publishFrequency = 10f; // Hz

    // Robot state
    private Vector3 targetPosition;
    private Quaternion targetRotation;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;
        ros.RegisterPublisher<TwistMsg>("cmd_vel");
        ros.RegisterSubscriber<Float32Msg>("sensor_data", OnSensorDataReceived);

        // Start coroutine to publish messages
        StartCoroutine(PublishCommands());
    }

    IEnumerator PublishCommands()
    {
        float delay = 1f / publishFrequency;

        while (true)
        {
            // Create and publish velocity command
            var twist = new TwistMsg();
            twist.linear = new Vector3Msg(0.5f, 0, 0); // Move forward at 0.5 m/s
            twist.angular = new Vector3Msg(0, 0, 0.2f); // Rotate at 0.2 rad/s

            ros.Publish("cmd_vel", twist);

            yield return new WaitForSeconds(delay);
        }
    }

    void OnSensorDataReceived(Float32Msg msg)
    {
        Debug.Log($"Received sensor data: {msg.data}");

        // Process sensor data and update robot behavior
        ProcessSensorData(msg.data);
    }

    void ProcessSensorData(float sensorValue)
    {
        // Example: Stop if obstacle is detected
        if (sensorValue < 0.5f)
        {
            Debug.Log("Obstacle detected! Stopping robot.");
            // In a real implementation, you'd stop the robot
        }
    }

    void Update()
    {
        // Update robot position based on target
        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime);
    }
}
```

### Tutorial 3: Creating a Unity Robot Model

Let's create a simple robot model in Unity that can be controlled via ROS:

```csharp
// File: Assets/Scripts/RobotModel.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotModel : MonoBehaviour
{
    [Header("Robot Configuration")]
    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.4f;
    public Transform leftWheel;
    public Transform rightWheel;
    public Transform sensorMount;

    [Header("Physics")]
    public float maxLinearVelocity = 1.0f;
    public float maxAngularVelocity = 1.0f;

    private float leftWheelVelocity = 0f;
    private float rightWheelVelocity = 0f;

    void Start()
    {
        if (leftWheel == null || rightWheel == null)
        {
            Debug.LogError("Wheel transforms not assigned!");
        }
    }

    public void SetDifferentialDriveVelocity(float linear, float angular)
    {
        // Convert linear and angular velocities to wheel velocities
        float leftVel = linear - (angular * wheelSeparation / 2.0f);
        float rightVel = linear + (angular * wheelSeparation / 2.0f);

        // Limit velocities
        leftVel = Mathf.Clamp(leftVel, -maxLinearVelocity, maxLinearVelocity);
        rightVel = Mathf.Clamp(rightVel, -maxLinearVelocity, maxLinearVelocity);

        leftWheelVelocity = leftVel / wheelRadius; // Convert to angular velocity
        rightWheelVelocity = rightVel / wheelRadius;

        // Debug logging
        Debug.Log($"Setting velocities - Left: {leftWheelVelocity}, Right: {rightWheelVelocity}");
    }

    void Update()
    {
        // Rotate wheels based on velocities
        if (leftWheel != null)
        {
            leftWheel.Rotate(Vector3.right, leftWheelVelocity * Mathf.Rad2Deg * Time.deltaTime);
        }

        if (rightWheel != null)
        {
            rightWheel.Rotate(Vector3.right, rightWheelVelocity * Mathf.Rad2Deg * Time.deltaTime);
        }
    }
}
```

### Tutorial 4: Unity Sensor Simulation

Create sensor simulation in Unity:

```csharp
// File: Assets/Scripts/LidarSensor.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;

public class LidarSensor : MonoBehaviour
{
    [Header("Lidar Configuration")]
    public int numberOfRays = 360;
    public float maxRange = 10.0f;
    public float minRange = 0.1f;
    public float fieldOfView = 360f;
    public LayerMask detectionLayers = -1;

    [Header("ROS Settings")]
    public string topicName = "scan";
    public float publishRate = 10f;

    private ROSConnection ros;
    private float timeSinceLastPublish = 0f;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;

        if (timeSinceLastPublish >= 1f / publishRate)
        {
            PublishLidarScan();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishLidarScan()
    {
        float[] ranges = new float[numberOfRays];
        float angleStep = fieldOfView / numberOfRays;

        for (int i = 0; i < numberOfRays; i++)
        {
            float angle = (i * angleStep - fieldOfView / 2) * Mathf.Deg2Rad;

            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            direction = transform.TransformDirection(direction);

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange, detectionLayers))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;
            }
        }

        // Create and publish LaserScan message
        var scanMsg = new LaserScanMsg();
        scanMsg.header = new std_msgs.HeaderMsg();
        scanMsg.header.stamp = new builtin_interfaces.TimeMsg();
        scanMsg.header.frame_id = "laser_frame";

        scanMsg.angle_min = -fieldOfView * Mathf.Deg2Rad / 2;
        scanMsg.angle_max = fieldOfView * Mathf.Deg2Rad / 2;
        scanMsg.angle_increment = fieldOfView * Mathf.Deg2Rad / numberOfRays;
        scanMsg.time_increment = 0.0f;
        scanMsg.scan_time = 1.0f / publishRate;
        scanMsg.range_min = minRange;
        scanMsg.range_max = maxRange;
        scanMsg.ranges = ranges;

        ros.Publish(topicName, scanMsg);
    }
}
```

### Tutorial 5: Camera Sensor Integration

Add camera sensor functionality to Unity:

```csharp
// File: Assets/Scripts/CameraSensor.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using System.Threading.Tasks;

public class CameraSensor : MonoBehaviour
{
    [Header("Camera Configuration")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 10f;

    [Header("ROS Settings")]
    public string imageTopic = "camera/image_raw";
    public string infoTopic = "camera/camera_info";

    private ROSConnection ros;
    private Camera unityCamera;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timeSinceLastPublish = 0f;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.RegisterPublisher<ImageMsg>(imageTopic);
        ros.RegisterPublisher<CameraInfoMsg>(infoTopic);

        unityCamera = GetComponent<Camera>();
        if (unityCamera == null)
        {
            unityCamera = gameObject.AddComponent<Camera>();
        }

        // Create render texture for camera capture
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        unityCamera.targetTexture = renderTexture;

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;

        if (timeSinceLastPublish >= 1f / publishRate)
        {
            CaptureAndPublishImage();
            timeSinceLastPublish = 0f;
        }
    }

    void CaptureAndPublishImage()
    {
        // Capture the camera image
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Convert to byte array for ROS message
        byte[] imageData = texture2D.EncodeToJPG(85); // 85% quality

        // Create and publish image message
        var imageMsg = new ImageMsg();
        imageMsg.header = new std_msgs.HeaderMsg();
        imageMsg.header.stamp = new builtin_interfaces.TimeMsg();
        imageMsg.header.frame_id = "camera_frame";

        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // 3 bytes per pixel (RGB)
        imageMsg.data = imageData;

        ros.Publish(imageTopic, imageMsg);

        // Publish camera info as well
        PublishCameraInfo();
    }

    void PublishCameraInfo()
    {
        var infoMsg = new CameraInfoMsg();
        infoMsg.header = new std_msgs.HeaderMsg();
        infoMsg.header.stamp = new builtin_interfaces.TimeMsg();
        infoMsg.header.frame_id = "camera_frame";

        infoMsg.width = (uint)imageWidth;
        infoMsg.height = (uint)imageHeight;

        // Set camera intrinsic parameters (these should match your camera setup)
        infoMsg.K = new double[9] {
            320, 0, imageWidth/2.0,   // fx, 0, cx
            0, 320, imageHeight/2.0,  // 0, fy, cy
            0, 0, 1                   // 0, 0, 1
        };

        infoMsg.R = new double[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // Rectification matrix
        infoMsg.P = new double[12] { 320, 0, 320, 0, 0, 320, 240, 0, 0, 0, 1, 0 }; // Projection matrix

        ros.Publish(infoTopic, infoMsg);
    }
}
```

## Code Examples with Explanations

### Example 1: Unity-ROS Bridge Manager

Create a comprehensive bridge manager for Unity:

```csharp
// File: Assets/Scripts/UnityROSManager.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class UnityROSManager : MonoBehaviour
{
    [Header("Connection Settings")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Control")]
    public string cmdVelTopic = "cmd_vel";
    public string odomTopic = "odom";

    [Header("Sensors")]
    public string scanTopic = "scan";
    public string imageTopic = "camera/image_raw";

    // Components
    private ROSConnection ros;
    private RobotModel robotModel;

    // State tracking
    private Vector3 lastPosition = Vector3.zero;
    private Quaternion lastRotation = Quaternion.identity;
    private float publishFrequency = 50f; // Hz for odometry

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Set up publishers
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<OdometryMsg>(odomTopic);

        // Set up subscribers
        ros.RegisterSubscriber<TwistMsg>(cmdVelTopic, OnVelocityCommandReceived);

        // Find robot model in scene
        robotModel = FindObjectOfType<RobotModel>();
        if (robotModel == null)
        {
            Debug.LogError("No RobotModel found in scene!");
        }

        // Start odometry publishing coroutine
        StartCoroutine(PublishOdometry());
    }

    void OnVelocityCommandReceived(TwistMsg twist)
    {
        if (robotModel != null)
        {
            // Convert ROS Twist to Unity velocity
            float linearVel = (float)twist.linear.x;
            float angularVel = (float)twist.angular.z;

            robotModel.SetDifferentialDriveVelocity(linearVel, angularVel);
        }
    }

    IEnumerator PublishOdometry()
    {
        float delay = 1f / publishFrequency;

        while (true)
        {
            if (robotModel != null)
            {
                PublishRobotOdometry();
            }

            yield return new WaitForSeconds(delay);
        }
    }

    void PublishRobotOdometry()
    {
        // Create odometry message
        var odomMsg = new OdometryMsg();
        odomMsg.header = new std_msgs.HeaderMsg();
        odomMsg.header.stamp = new builtin_interfaces.TimeMsg();
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_link";

        // Set position
        odomMsg.pose.pose.position = new geometry_msgs.PointMsg();
        odomMsg.pose.pose.position.x = transform.position.x;
        odomMsg.pose.pose.position.y = transform.position.z; // Unity Z -> ROS Y
        odomMsg.pose.pose.position.z = transform.position.y; // Unity Y -> ROS Z

        // Set orientation
        odomMsg.pose.pose.orientation = new geometry_msgs.QuaternionMsg();
        odomMsg.pose.pose.orientation.x = transform.rotation.x;
        odomMsg.pose.pose.orientation.y = transform.rotation.z; // Unity Z -> ROS Y
        odomMsg.pose.pose.orientation.z = transform.rotation.y; // Unity Y -> ROS Z
        odomMsg.pose.pose.orientation.w = transform.rotation.w;

        // Set velocities (approximate)
        Vector3 velocity = (transform.position - lastPosition) / Time.deltaTime;
        odomMsg.twist.twist.linear = new Vector3Msg(velocity.x, velocity.z, velocity.y);

        // Publish the message
        ros.Publish(odomTopic, odomMsg);

        // Update last position for next calculation
        lastPosition = transform.position;
    }

    void OnValidate()
    {
        // Ensure valid ranges for parameters
        publishFrequency = Mathf.Max(1f, publishFrequency);
    }
}
```

### Example 2: Human-Robot Interaction in Unity

Create a system for human-robot interaction simulation:

```csharp
// File: Assets/Scripts/HRIManager.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;

public class HRIManager : MonoBehaviour
{
    [Header("Interaction Settings")]
    public string speechTopic = "speech_commands";
    public string gestureTopic = "gesture_data";
    public string attentionTopic = "attention_status";

    [Header("UI Elements")]
    public InputField speechInputField;
    public Button speechSendButton;
    public Text interactionLog;

    [Header("Robot Interaction")]
    public Transform robotTransform;
    public float interactionDistance = 3.0f;

    private ROSConnection ros;
    private List<string> interactionHistory = new List<string>();

    void Start()
    {
        ros = ROSConnection.instance;
        ros.RegisterPublisher<StringMsg>(speechTopic);
        ros.RegisterPublisher<PointCloud2Msg>(gestureTopic);
        ros.RegisterPublisher<BoolMsg>(attentionTopic);

        if (speechSendButton != null)
        {
            speechSendButton.onClick.AddListener(SendSpeechCommand);
        }
    }

    void Update()
    {
        // Check for interaction with robot
        if (robotTransform != null && Vector3.Distance(transform.position, robotTransform.position) <= interactionDistance)
        {
            // Publish attention status
            var attentionMsg = new BoolMsg();
            attentionMsg.data = true;
            ros.Publish(attentionTopic, attentionMsg);

            // Visual feedback
            HighlightInteractionZone(true);
        }
        else
        {
            var attentionMsg = new BoolMsg();
            attentionMsg.data = false;
            ros.Publish(attentionTopic, attentionMsg);

            HighlightInteractionZone(false);
        }
    }

    void SendSpeechCommand()
    {
        if (speechInputField != null && !string.IsNullOrEmpty(speechInputField.text))
        {
            // Create and send speech command
            var speechMsg = new StringMsg();
            speechMsg.data = speechInputField.text;

            ros.Publish(speechTopic, speechMsg);

            // Add to interaction history
            AddToInteractionLog($"Sent: {speechMsg.data}");

            // Clear input field
            speechInputField.text = "";
        }
    }

    void HighlightInteractionZone(bool active)
    {
        // Visual feedback for interaction zone
        // This could be a particle effect, color change, etc.
        if (active)
        {
            // Highlight interaction area
            Debug.Log("Robot is in interaction range!");
        }
    }

    public void AddToInteractionLog(string message)
    {
        interactionHistory.Add($"[{System.DateTime.Now:HH:mm:ss}] {message}");

        if (interactionHistory.Count > 10) // Keep only last 10 messages
        {
            interactionHistory.RemoveAt(0);
        }

        // Update UI
        if (interactionLog != null)
        {
            interactionLog.text = string.Join("\n", interactionHistory.ToArray());
        }

        Debug.Log(message);
    }

    // Method to receive responses from ROS
    public void OnRobotResponse(string response)
    {
        AddToInteractionLog($"Robot: {response}");
    }
}
```

### Example 3: Multi-Sensor Fusion in Unity

Implement sensor fusion for multiple sensors:

```csharp
// File: Assets/Scripts/SensorFusion.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class SensorFusion : MonoBehaviour
{
    [Header("Sensor Topics")]
    public string lidarTopic = "scan";
    public string cameraTopic = "camera/image_raw";
    public string imuTopic = "imu/data";
    public string fusedTopic = "sensor_fusion_output";

    [Header("Fusion Settings")]
    public float fusionRate = 10f;
    public float lidarTimeout = 0.5f;
    public float cameraTimeout = 1.0f;

    private ROSConnection ros;

    // Sensor data storage
    private LaserScanMsg latestLidarData;
    private ImageMsg latestCameraData;
    private ImuMsg latestImuData;

    // Timestamp tracking
    private float lastLidarTime;
    private float lastCameraTime;
    private float lastImuTime;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.RegisterPublisher<PointCloud2Msg>(fusedTopic);

        // Register subscribers
        ros.RegisterSubscriber<LaserScanMsg>(lidarTopic, OnLidarDataReceived);
        ros.RegisterSubscriber<ImageMsg>(cameraTopic, OnCameraDataReceived);
        ros.RegisterSubscriber<ImuMsg>(imuTopic, OnImuDataReceived);

        // Start fusion coroutine
        StartCoroutine(PerformFusion());
    }

    void OnLidarDataReceived(LaserScanMsg scan)
    {
        latestLidarData = scan;
        lastLidarTime = Time.time;
    }

    void OnCameraDataReceived(ImageMsg image)
    {
        latestCameraData = image;
        lastCameraTime = Time.time;
    }

    void OnImuDataReceived(ImuMsg imu)
    {
        latestImuData = imu;
        lastImuTime = Time.time;
    }

    IEnumerator PerformFusion()
    {
        float delay = 1f / fusionRate;

        while (true)
        {
            // Check if we have recent data from all sensors
            bool lidarValid = (Time.time - lastLidarTime) < lidarTimeout && latestLidarData != null;
            bool cameraValid = (Time.time - lastCameraTime) < cameraTimeout && latestCameraData != null;
            bool imuValid = latestImuData != null; // IMU data is typically always available

            if (lidarValid && cameraValid && imuValid)
            {
                PerformSensorFusion();
            }

            yield return new WaitForSeconds(delay);
        }
    }

    void PerformSensorFusion()
    {
        // Create a fused sensor output
        // This is a simplified example - real fusion would be more complex
        var fusedMsg = new PointCloud2Msg();
        fusedMsg.header = new std_msgs.HeaderMsg();
        fusedMsg.header.stamp = new builtin_interfaces.TimeMsg();
        fusedMsg.header.frame_id = "sensor_fusion_frame";

        // In a real implementation, you would:
        // 1. Transform all sensor data to a common coordinate frame
        // 2. Apply sensor fusion algorithms (Kalman filters, particle filters, etc.)
        // 3. Generate a unified representation of the environment
        // 4. Handle timing synchronization between sensors

        // For this example, we'll just publish a placeholder
        fusedMsg.height = 1;
        fusedMsg.width = 0; // No points in this example
        fusedMsg.fields = new sensor_msgs.PointFieldMsg[0];
        fusedMsg.is_bigendian = false;
        fusedMsg.point_step = 0;
        fusedMsg.row_step = 0;
        fusedMsg.is_dense = true;
        fusedMsg.data = new byte[0];

        ros.Publish(fusedTopic, fusedMsg);

        Debug.Log("Sensor fusion performed with all sensor data");
    }

    void OnValidate()
    {
        fusionRate = Mathf.Max(0.1f, fusionRate); // Minimum 0.1 Hz
        lidarTimeout = Mathf.Max(0.1f, lidarTimeout);
        cameraTimeout = Mathf.Max(0.1f, cameraTimeout);
    }
}
```

## Hands-On Exercises: Unity-ROS Integration

### Exercise 1: Create a Unity Robot Controller

Create a Unity scene with a robot that can be controlled via ROS:

1. Create a Unity scene with a robot model
2. Implement ROS communication for movement commands
3. Add sensor simulation (LIDAR and camera)
4. Test the integration with a ROS node

### Exercise 2: Human-Robot Interaction Simulation

Develop a Unity environment that simulates human-robot interaction:

```csharp
// File: Assets/Scripts/InteractionZone.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InteractionZone : MonoBehaviour
{
    [Header("Interaction Settings")]
    public float interactionRadius = 2.0f;
    public LayerMask humanLayers;
    public string interactionTopic = "human_robot_interaction";

    [Header("Visual Feedback")]
    public Color normalColor = Color.blue;
    public Color interactionColor = Color.green;
    public Renderer interactionRenderer;

    private ROSConnection ros;
    private bool isInteracting = false;

    void Start()
    {
        SetupInteractionZone();
    }

    void SetupInteractionZone()
    {
        // Set up the interaction sphere collider
        SphereCollider sphere = gameObject.AddComponent<SphereCollider>();
        sphere.isTrigger = true;
        sphere.radius = interactionRadius;

        // Set up visual representation if provided
        if (interactionRenderer != null)
        {
            interactionRenderer.material.color = normalColor;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (IsHuman(other.gameObject))
        {
            StartInteraction(other.gameObject);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (IsHuman(other.gameObject))
        {
            EndInteraction(other.gameObject);
        }
    }

    bool IsHuman(GameObject obj)
    {
        return (humanLayers & (1 << obj.layer)) != 0;
    }

    void StartInteraction(GameObject human)
    {
        isInteracting = true;
        Debug.Log($"Interaction started with {human.name}");

        if (interactionRenderer != null)
        {
            interactionRenderer.material.color = interactionColor;
        }

        // In a real implementation, publish interaction start message
        // PublishInteractionMessage("interaction_started", human.name);
    }

    void EndInteraction(GameObject human)
    {
        isInteracting = false;
        Debug.Log($"Interaction ended with {human.name}");

        if (interactionRenderer != null)
        {
            interactionRenderer.material.color = normalColor;
        }

        // In a real implementation, publish interaction end message
        // PublishInteractionMessage("interaction_ended", human.name);
    }

    void Update()
    {
        if (isInteracting)
        {
            // Update interaction logic
            UpdateInteraction();
        }
    }

    void UpdateInteraction()
    {
        // Handle ongoing interaction
        // This could include gesture recognition, speech processing, etc.
    }

    public bool IsCurrentlyInteracting()
    {
        return isInteracting;
    }
}
```

### Exercise 3: Advanced Environment Simulation

Create a Unity environment with dynamic elements:

```csharp
// File: Assets/Scripts/DynamicEnvironment.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DynamicEnvironment : MonoBehaviour
{
    [Header("Dynamic Objects")]
    public List<GameObject> moveableObjects = new List<GameObject>();
    public float objectMovementSpeed = 0.5f;

    [Header("Environment Settings")]
    public Light mainLight;
    public float dayNightCycleSpeed = 0.1f;
    public AnimationCurve lightIntensityCurve;

    [Header("Weather Effects")]
    public ParticleSystem rainEffect;
    public float weatherChangeInterval = 30f;

    private float timeOfDay = 0f;
    private float weatherTimer = 0f;
    private string[] weatherTypes = {"clear", "rainy", "cloudy"};
    private int currentWeatherIndex = 0;

    void Start()
    {
        InitializeEnvironment();
    }

    void InitializeEnvironment()
    {
        // Set up moveable objects with random movement
        foreach (GameObject obj in moveableObjects)
        {
            if (obj.GetComponent<Rigidbody>() == null)
            {
                obj.AddComponent<Rigidbody>();
                obj.GetComponent<Rigidbody>().isKinematic = true; // For controlled movement
            }
        }

        // Set up weather effects
        if (rainEffect != null)
        {
            rainEffect.Stop();
        }
    }

    void Update()
    {
        UpdateDayNightCycle();
        UpdateDynamicObjects();
        UpdateWeather();
    }

    void UpdateDayNightCycle()
    {
        timeOfDay += Time.deltaTime * dayNightCycleSpeed;

        if (mainLight != null)
        {
            // Rotate the light to simulate day/night
            mainLight.transform.rotation = Quaternion.Euler(
                timeOfDay * 360f,
                45f,
                0f
            );

            // Adjust light intensity based on time of day
            float intensityFactor = lightIntensityCurve.Evaluate(timeOfDay % 1f);
            mainLight.intensity = Mathf.Lerp(0.2f, 1f, intensityFactor);
        }
    }

    void UpdateDynamicObjects()
    {
        foreach (GameObject obj in moveableObjects)
        {
            // Simple back-and-forth movement
            Vector3 moveDirection = new Vector3(
                Mathf.Sin(Time.time * objectMovementSpeed) * 0.1f,
                0,
                Mathf.Cos(Time.time * objectMovementSpeed) * 0.1f
            );

            obj.transform.position += moveDirection * Time.deltaTime;
        }
    }

    void UpdateWeather()
    {
        weatherTimer += Time.deltaTime;

        if (weatherTimer >= weatherChangeInterval)
        {
            ChangeWeather();
            weatherTimer = 0f;
        }
    }

    void ChangeWeather()
    {
        currentWeatherIndex = (currentWeatherIndex + 1) % weatherTypes.Length;

        switch (weatherTypes[currentWeatherIndex])
        {
            case "clear":
                DisableRain();
                break;
            case "rainy":
                EnableRain();
                break;
            case "cloudy":
                SetCloudy();
                break;
        }

        Debug.Log($"Weather changed to: {weatherTypes[currentWeatherIndex]}");
    }

    void EnableRain()
    {
        if (rainEffect != null)
        {
            rainEffect.Play();
        }
    }

    void DisableRain()
    {
        if (rainEffect != null)
        {
            rainEffect.Stop();
        }
    }

    void SetCloudy()
    {
        if (rainEffect != null)
        {
            rainEffect.Stop();
        }

        if (mainLight != null)
        {
            mainLight.intensity = 0.5f; // Reduced light for cloudy effect
        }
    }
}
```

## Common Pitfalls and Solutions

### Pitfall 1: Performance Issues with High-Frequency Communication
**Problem**: Unity-ROS communication causes frame rate drops due to high-frequency message passing.

**Solutions**:
- Implement message throttling based on actual needs
- Use asynchronous communication patterns
- Optimize message serialization/deserialization

```csharp
// Implement message rate limiting
public class MessageRateLimiter
{
    private float lastPublishTime;
    private float publishInterval;

    public MessageRateLimiter(float rate)
    {
        publishInterval = 1.0f / rate;
        lastPublishTime = -1000f; // Initialize to allow immediate first message
    }

    public bool ShouldPublish()
    {
        float currentTime = Time.time;
        if (currentTime - lastPublishTime >= publishInterval)
        {
            lastPublishTime = currentTime;
            return true;
        }
        return false;
    }
}
```

### Pitfall 2: Coordinate System Mismatches
**Problem**: Unity and ROS use different coordinate systems, causing incorrect transformations.

**Solution**:
- Create a coordinate transformation utility
- Consistently apply transformations at the ROS bridge level

### Pitfall 3: Large Message Sizes
**Problem**: High-resolution sensor data (especially images) creates large messages that impact performance.

**Solution**:
- Implement image compression before transmission
- Use appropriate image resolution for the task
- Consider streaming only when needed

## Review Questions

1. What are the main advantages of using Unity for robotics simulation compared to Gazebo?
2. Explain the architecture of Unity-ROS communication.
3. How do you handle coordinate system differences between Unity and ROS?
4. What are the key considerations for real-time performance in Unity-ROS integration?
5. Describe how you would implement sensor fusion in a Unity-ROS system.

## Project Assignment: Unity-ROS Integration System

Create a complete Unity-ROS integration project that includes:
1. A Unity scene with a controllable robot model
2. Multiple sensor simulations (LIDAR, camera, IMU)
3. Human-robot interaction capabilities
4. Dynamic environment elements
5. Proper coordinate system handling
6. Performance optimization techniques

Your project should:
- Demonstrate bidirectional communication between Unity and ROS
- Include at least 3 different sensor types
- Show realistic robot behavior based on sensor input
- Implement proper error handling and logging
- Be optimized for real-time performance

## Further Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Robotics Package](https://docs.unity3d.com/Packages/com.unity.robotics.ros-tcp-connector@latest)
- [Human-Robot Interaction in Unity](https://arxiv.org/abs/2008.05659)
- [Synthetic Data Generation](https://github.com/Unity-Technologies/Unity-ComputerVision)

<details>
<summary>Advanced Unity-ROS Integration</summary>

Unity's strength in high-fidelity rendering makes it ideal for perception training and human-robot interaction studies. When combined with ROS, it enables sophisticated simulation scenarios that bridge the gap between pure physics simulation and real-world deployment. Consider using Unity's built-in AI tools like ML-Agents for training perception and control models directly in the simulation environment.

</details>