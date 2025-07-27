using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Base class for the two-wheel rovers (Tom & Jerry).
///
/// ▸ *All* observation gathering now happens inside each rover script.
/// ▸ Positions / velocities are expressed **in the parent Set frame**
///   (X-Z plane), not the rover’s own heading.
/// ▸ Each rover is responsible for choosing a random spawn pose on
///   <see cref="OnEpisodeBegin"/> using parameters exposed on the
///   <see cref="TagGameController"/> singleton.
/// </summary>

[RequireComponent(typeof(Rigidbody))]
public class RoverAgent : Agent
{
    // ───────── Inspector knobs ─────────
    [Header("Geometry")]
    public float wheelBaseline = 1.0f;

    [Header("Wheel Model")]
    public float maxWheelSpeed = 3.0f; // m/s when action = ±1
    public float coastTime     = 0.3f; // time-constant for first-order wheel slew

    [Header("Controller Gains")]
    public float linGain = 2.0f;       // forward-speed P-gain
    public float angGain = 1.5f;       // yaw-rate   P-gain

    // ───────── Internal state ─────────
    protected Rigidbody rb;
    protected float cmdL, cmdR;   // wheel throttle commands –1..1
    protected float vL, vR;       // wheel linear speeds (m/s)

    protected TagGameController gc;

    [Header("Per‑episode observation noise (σ)")]
    public Vector2 velSigmaRange   = new Vector2(0.00f, 0.03f);  // m/s
    public Vector2 posSigmaRange   = new Vector2(0.00f, 0.02f);  // m
    public Vector2 yawSigmaRange   = new Vector2(0.0f,  3.0f);   // deg
    public bool noiseTrainingOnly  = true;                       // no noise when not training
    public int  randomSeed         = 0;                          // 0 = don't set (non‑deterministic)

    // Per‑episode sigmas
    float velSigma, posSigma, yawSigmaDeg;

    // ───────── Unity lifecycle ─────────
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        gc = GetComponentInParent<TagGameController>();
        if (randomSeed != 0) Random.InitState(randomSeed);  // optional reproducibility
    }

    public override void OnEpisodeBegin()
    {
        // 1. Clear motion state
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        cmdL = cmdR = vL = vR = 0f;

        // 2. Choose a fresh spawn pose within the controller’s spawn bounds
        if (gc != null)
        {
            Vector3 p;
            int guard = 0;
            do
            {
                p = gc.RandomInSpawn();
            } while (gc.TooClose(p, this) && ++guard < 100);

            transform.localPosition = p;
            transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        bool applyNoise = true; //!noiseTrainingOnly || Academy.Instance.IsCommunicatorOn;

        if (applyNoise)
        {
            velSigma    = Random.Range(velSigmaRange.x, velSigmaRange.y);
            posSigma    = Random.Range(posSigmaRange.x, posSigmaRange.y);
            yawSigmaDeg = Random.Range(yawSigmaRange.x, yawSigmaRange.y);
        }
        else
        {
            velSigma = posSigma = yawSigmaDeg = 0f;
        }
    }
    
    // Box–Muller Gaussian using UnityEngine.Random + Mathf
    static float Gauss(float sigma)
    {
        float u1 = Mathf.Max(1e-6f, Random.value);
        float u2 = Random.value;
        float z0 = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        return z0 * sigma;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 vel = rb.velocity;              // world == Set frame
        Vector3 pos = transform.localPosition;

        float vx  = vel.x + Gauss(velSigma);
        float vz  = vel.z + Gauss(velSigma);
        float px  = pos.x + Gauss(posSigma);
        float pz  = pos.z + Gauss(posSigma);

        float yaw = Mathf.DeltaAngle(0f, transform.eulerAngles.y);   // [-180,180]
        yaw += Gauss(yawSigmaDeg);
        yaw  = Mathf.DeltaAngle(0f, yaw);

        sensor.AddObservation(vx);
        sensor.AddObservation(vz);
        sensor.AddObservation(px);
        sensor.AddObservation(pz);
        sensor.AddObservation(yaw);
    }

    public override void OnActionReceived(ActionBuffers act)
    {
        cmdL = Mathf.Clamp(act.ContinuousActions[0], -1f, 1f);
        cmdR = Mathf.Clamp(act.ContinuousActions[1], -1f, 1f);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0) return;

        // First-order wheel response
        float slew = maxWheelSpeed / Mathf.Max(coastTime, 1e-3f);
        vL = Mathf.MoveTowards(vL, cmdL * maxWheelSpeed, slew * dt);
        vR = Mathf.MoveTowards(vR, cmdR * maxWheelSpeed, slew * dt);

        // Desired forward / yaw velocity (differential drive kinematics)
        float vDes = 0.5f * (vL + vR);
        float wDes = (vR - vL) / wheelBaseline;

        float vCur = Vector3.Dot(rb.velocity, transform.forward);
        float wCur = rb.angularVelocity.y;

        rb.AddForce( transform.forward * (vDes - vCur) * linGain,
                      ForceMode.VelocityChange );
        rb.AddTorque( Vector3.up * (wDes - wCur) * angGain,
                      ForceMode.VelocityChange );

        // Kill sideways skids (acts like a no-slip caster wheel)
        Vector3 lateral = Vector3.Project(rb.velocity, transform.right);
        rb.AddForce(-lateral, ForceMode.VelocityChange);
    }

    // Simple keyboard heuristics for quick testing
    public override void Heuristic(in ActionBuffers actOut)
    {
        var a = actOut.ContinuousActions;
        float l = (Input.GetKey(KeyCode.W) ? 1 : 0) - (Input.GetKey(KeyCode.S) ? 1 : 0);
        float r = (Input.GetKey(KeyCode.E) ? 1 : 0) - (Input.GetKey(KeyCode.D) ? 1 : 0);
        a[0] = Mathf.Clamp(l, -1f, 1f);
        a[1] = Mathf.Clamp(r, -1f, 1f);
    }
}
