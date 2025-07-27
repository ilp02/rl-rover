using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Jerry = the evader.  Observations are Set‑frame (not body‑frame).
/// </summary>
public class RoverJerry : RoverAgent
{
    [Header("Tag parameters")]
    public Transform  chaser;     // Tom root
    public Rigidbody  chaserRb;

    public float stepReward  = 0.8f;

    public override void CollectObservations(VectorSensor s)
    {
        base.CollectObservations(s);               // 5 floats

        Vector3 relPos = chaser.position  - transform.position;
        Vector3 relVel = chaserRb.velocity - rb.velocity;

        s.AddObservation(relPos.x);                // +4 floats
        s.AddObservation(relPos.z);
        s.AddObservation(relVel.x);
        s.AddObservation(relVel.z);
    }

    float prevD;
    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        prevD = Vector3.Distance(transform.position, chaser.position);
    }

    public override void OnActionReceived(ActionBuffers act)
    {
        base.OnActionReceived(act);

        AddReward(stepReward);
        
        float d = Vector3.Distance(transform.position, chaser.position);
        // ❶ dense shaping: positive if Jerry increased the gap
        if ((d - prevD) >= 0)
        {
            AddReward(0.0f * (d - prevD));   // k = 0.5  (tune 0.3–1.0)
        }
        
        prevD = d;
    }
}