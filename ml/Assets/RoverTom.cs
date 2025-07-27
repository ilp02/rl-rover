using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Tom = the chaser.  Observations are now Set‑frame, not body‑frame.
/// Total observation size = 9 floats (5 self + 4 target).
/// </summary>
public class RoverTom : RoverAgent
{
    [Header("Tag parameters")]
    public Transform target;         // Jerry root (assign in inspector)
    public Rigidbody targetRb;

    public override void CollectObservations(VectorSensor s)
    {
        base.CollectObservations(s);                 // 5 floats (self)

        // Relative data expressed in Set frame
        Vector3 relPos = target.position - transform.position;
        Vector3 relVel = targetRb.velocity - rb.velocity;

        s.AddObservation(relPos.x);                  // +4 floats
        s.AddObservation(relPos.z);
        s.AddObservation(relVel.x);
        s.AddObservation(relVel.z);
    }

    float prevD;              // add to RoverTom
    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        prevD = Vector3.Distance(transform.position, target.position);
    }

    public override void OnActionReceived(ActionBuffers a)
    {
        base.OnActionReceived(a);

        float d = Vector3.Distance(transform.position, target.position);

        // ❶ dense shaping: positive if Tom reduced the gap
        AddReward(0.8f * (prevD - d));   // k = 0.5  (tune 0.3–1.0)
        AddReward(0.02f * Mathf.Clamp01(1f - d / 10f));

        // ❷ tiny time cost to keep him decisive
        AddReward(-0.0005f);

        prevD = d;
    }
}