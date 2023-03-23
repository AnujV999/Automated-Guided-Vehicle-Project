using UnityEngine;
using TMPro;
using System.Xml.Serialization;
using TreeEditor;
using UnityEngine.UIElements;

public class Car_controller : MonoBehaviour
{
    [Header("Car Info")]
    private Rigidbody vehicle;
    public Vector3 centreOfMass;
    public Transform car;

    // Settings
    [SerializeField] private float motorForce, breakForce, maxSteerAngle ,newSteer;
    public float maxSpeed = 100f;
    public float turnSpeed = 5f;

    // Wheel Colliders
    [SerializeField] private WheelCollider frontLeftWheelCollider, frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider, rearRightWheelCollider;

    // Wheels
    [SerializeField] private Transform frontLeftWheelTransform, frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform, rearRightWheelTransform;

    [Header("Braking")]
    public bool isBraking;
    public bool isBacking;
    public MeshRenderer carRenderer;
    public Texture2D textureBraking;
    public Texture2D textureNormal;

    [Header("Waypoints")]
    [SerializeField] private Transform[] waypoints;
    public float minDistance = 5f;
    public int currentWaypointIndex = 0;
    [SerializeField] float distance;

    [Header("Obstacles")]
    [SerializeField] private Transform[] obstac;
    public float radius = 3f;
    public float[] obstacleDistanceArray = new float[5];
    public float minAvoidDistance = 10f;

    [Header("Speedometer")]
    public TMP_Text speed;
    public int vel;
    public float velocity;
    public Transform dial;
    public float dialSpeed = 3f;

    [Header("Sensors")]
    public float sensorLength = 10f;
    public float sideSensorLength = 3f;
    public float frontSensorPos = 2.1f;
    public float frontSideSensorPosition = 0.8f;
    public float frontSensorAngle = 30f;
    public bool avoiding = false;
    public bool slowing = false;
    public int minVelocityToAvoid = 15;
    public float frontTyrePos = 1.8f;

    [Header("Steer")]
    public float steerAngle;
    public float steerLevel1;
    public float steerLevel2;
    public float steerLevel3;
    public float velocityLimit1;
    public float velocityLimit2;

    [Header("Camera")]
    public Camera rearViewCamera;
    public Camera frontViewCamera;

    [Header("Lights")]
    public Renderer[] lights;
    public Material DarkRed;
    public Material red;

    private void Start()
    {
        for(int i = 0; i < lights.Length; i++)
        {
            lights[i].material = DarkRed;
        }

        frontViewCamera.enabled = false;
        rearViewCamera.enabled = true;

        dial.rotation = Quaternion.Euler(new Vector3(0, 0, 0));

        steerAngle = 0f;
        steerLevel1 = 10f;
        steerLevel2 = 20f;
        steerLevel3 = 35f;
        velocityLimit1 = 35;
        velocityLimit2 = 25;
        isBraking = false;
        isBacking = false;
        GetComponent<Rigidbody>().centerOfMass = centreOfMass;
        vehicle = GetComponent<Rigidbody>();
    }

    void OnDrawGizmos()
    {
        for(int i = 0; i < waypoints.Length; i++)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(waypoints[i].position, 3f);
        }
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(GetComponent<Rigidbody>().centerOfMass, 5f);
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            frontViewCamera.enabled = !frontViewCamera.enabled;
            rearViewCamera.enabled = !rearViewCamera.enabled;
        }
        //Debug.Log((velocity / 100) * 300f);
        Speedometer();
    }

    private void FixedUpdate()
    {
        distance = Vector3.Distance(transform.position,waypoints[currentWaypointIndex].position);
        Sensor();
        Drive();
        Slowing();
        ApplySteer();
        HandleMotor();
        Braking();
        UpdateWheels();
    }

    private void Slowing()
    {
        Vector3 AB = waypoints[currentWaypointIndex].position - transform.position;
        Vector3 BC = waypoints[(currentWaypointIndex+1)%waypoints.Length].position - waypoints[currentWaypointIndex].position;

        float dotProduct = Vector3.Dot(AB.normalized, BC.normalized);
        steerAngle = Mathf.Acos(dotProduct) * Mathf.Rad2Deg;

        Vector3 crossProduct = Vector3.Cross(AB, BC).normalized;
        float dotProductUp = Vector3.Dot(crossProduct, Vector3.up);

        if (dotProductUp < 0) steerAngle = 360 - steerAngle;
    }

    private void Sensor()
    {
        RaycastHit hit;
        Vector3 sensorStartPos = transform.position;
        sensorStartPos.y += 0.7f;

        Vector3 frontVec = transform.forward;
        Vector3 rightVec = transform.right;

        frontVec = frontVec.normalized;
        rightVec = rightVec.normalized;

        sensorStartPos += frontVec * frontSensorPos;
        Vector3 frontSensorStart = sensorStartPos;

        bool leftAngleSensor = false;

        float avoidMultiplier = 0f;
        avoiding = false;

        sensorStartPos += rightVec * frontSideSensorPosition;

        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier -= 1f;
                avoiding = true;
                Debug.DrawLine(sensorStartPos, hit.point, Color.blue);
            }
        }

        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontSensorAngle,transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier -= 0.5f;
                avoiding = true;
                Debug.DrawLine(sensorStartPos, hit.point, Color.red);
            }
        }

        sensorStartPos -= 2 * rightVec * frontSideSensorPosition;

        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier += 1f;
                avoiding = true;
                Debug.DrawLine(sensorStartPos, hit.point, Color.blue);
            }
        }

        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier += 0.5f;
                avoiding = true;
                Debug.DrawLine(sensorStartPos, hit.point, Color.red);
            }
        }

        //checking front sensor
        if(avoidMultiplier == 0)
        {
            if (Physics.Raycast(frontSensorStart, transform.forward, out hit, sensorLength))
            {
                if (hit.collider.CompareTag("ColliderTag"))
                {
                    avoiding = true;
                    Debug.DrawLine(frontSensorStart, hit.point, Color.blue);

                    if(hit.normal.x < 0)
                    {
                        avoidMultiplier -= 1f;
                    }else if(hit.normal.x > 0)
                    {
                        avoidMultiplier += 1f;
                    }else if(hit.normal.x == 0)
                    {
                        if (!leftAngleSensor)
                        {
                            avoidMultiplier -= 1f;
                        }
                        else avoidMultiplier += 1f;
                    }
                    if (hit.distance < minAvoidDistance)
                    {
                        isBraking = true;
                    }
                    else isBraking = false;
                }
            }
        }

        //sideways sensors

        Vector3 sidePosition = transform.position;
        sidePosition.y += 0.7f;
        sidePosition += frontVec * 0.5f;

        sidePosition += frontVec * frontTyrePos;
        sidePosition -= rightVec * frontSideSensorPosition;

        if (Physics.Raycast(sidePosition, Quaternion.AngleAxis(-3*frontSensorAngle, transform.up) * -transform.right, out hit, 2*sideSensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier = 0f;
                Debug.DrawLine(sidePosition, hit.point, Color.blue);
            }
        }

        sidePosition -= frontVec * frontTyrePos;

        if (Physics.Raycast(sidePosition, -transform.right, out hit, sideSensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier = 0f;
                Debug.DrawLine(sidePosition, hit.point, Color.blue);
            }
        }

        sidePosition -= frontVec * frontTyrePos;

        if (Physics.Raycast(sidePosition, Quaternion.AngleAxis(3*frontSensorAngle, transform.up) * -transform.right, out hit, 2*sideSensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier = 0f;
                Debug.DrawLine(sidePosition, hit.point, Color.blue);
            }
        }

        sidePosition += 2 * rightVec * frontSideSensorPosition;

        if (Physics.Raycast(sidePosition, transform.right, out hit, sideSensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier = 0f;
                Debug.DrawLine(sidePosition, hit.point, Color.blue);
            }
        }

        sidePosition += 2 * frontVec * frontTyrePos;

        if (Physics.Raycast(sidePosition, transform.right, out hit, sideSensorLength))
        {
            if (hit.collider.CompareTag("ColliderTag"))
            {
                avoidMultiplier = 0f;
                Debug.DrawLine(sidePosition, hit.point, Color.blue);
            }
        }

        if (avoiding)
        {
            if (avoidMultiplier > 1f) avoidMultiplier = 1f;
            else if (avoidMultiplier < -1f) avoidMultiplier = -1f;
            lerpTurn(maxSteerAngle * avoidMultiplier);
        }
    }

    private void Drive()
    {
        Vector3 direction = waypoints[currentWaypointIndex].position - car.position;

        direction.Normalize();

        distance = Vector3.Distance(car.position, waypoints[currentWaypointIndex].position);

        if (currentWaypointIndex == waypoints.Length - 1)
        {
            currentWaypointIndex = 0;
        }
        else if (distance < minDistance)
        {
            currentWaypointIndex++;
        }
    }

    private void lerpTurn(float newSteer)
    {
        frontLeftWheelCollider.steerAngle = Mathf.Lerp(frontLeftWheelCollider.steerAngle,newSteer,Time.deltaTime * turnSpeed);
        frontRightWheelCollider.steerAngle = Mathf.Lerp(frontRightWheelCollider.steerAngle, newSteer, Time.deltaTime * turnSpeed);
    }

    private void ApplySteer()
    {
        if (avoiding) return;
        Vector3 relativeVector = transform.InverseTransformPoint(waypoints[currentWaypointIndex].position);
        newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;

        lerpTurn(newSteer);
    }

    private void Speedometer()
    {
        velocity = vehicle.velocity.magnitude * 3.6f;
        velocity = Vector3.Dot(vehicle.velocity, transform.forward)*3.6f;
        vel = (int)velocity;
        speed.text = vel.ToString();
        float newRotation = (velocity / maxSpeed) * 300f;
        dial.rotation = Quaternion.Lerp(dial.rotation, Quaternion.Euler(0, 0, -newRotation), Time.deltaTime * dialSpeed);
    }

    private void Braking()
    {
        if (isBraking)
        {
            for (int i = 0; i < lights.Length; i++)
            {
                lights[i].material = red;
            }
            rearLeftWheelCollider.brakeTorque = breakForce;
            rearRightWheelCollider.brakeTorque = breakForce;
            frontLeftWheelCollider.steerAngle = 0;
            frontRightWheelCollider.steerAngle = 0;
        }
        else
        {
            for (int i = 0; i < lights.Length; i++)
            {
                lights[i].material = DarkRed;
            }
            rearLeftWheelCollider.brakeTorque = 0;
            rearRightWheelCollider.brakeTorque = 0;
        }
    }

    private void HandleMotor()
    {
        //float maximumSpeed = maxSpeed / (2f * Mathf.PI * rearLeftWheelCollider.radius) * 60f;
        if (velocity < maxSpeed/1.5f && !isBraking && steerAngle < steerLevel1)
        {
            //carRenderer.material.mainTexture = textureNormal;
            rearLeftWheelCollider.motorTorque = motorForce;
            rearRightWheelCollider.motorTorque = motorForce;
        }
        else if(velocity < maxSpeed/1.5f && !isBraking && steerAngle < steerLevel2)
        {
            if(vel > velocityLimit1 && distance < 10f)
            {
                //carRenderer.material.mainTexture = textureBraking;
                rearLeftWheelCollider.motorTorque = 0;
                rearRightWheelCollider.motorTorque = 0;
            }
        }
        else if(velocity < maxSpeed/1.5f && !isBraking && steerAngle < steerLevel3)
        {
            if(vel > velocityLimit2 && distance < 10f)
            {
                //carRenderer.material.mainTexture = textureBraking;
                rearLeftWheelCollider.brakeTorque = breakForce;
                rearRightWheelCollider.brakeTorque = breakForce;
            }
        }
        else
        {
            //carRenderer.material.mainTexture = textureBraking;
            rearLeftWheelCollider.motorTorque = 0;
            rearRightWheelCollider.motorTorque = 0;
        }
    }
    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
        wheelTransform.position = pos;
    }
}