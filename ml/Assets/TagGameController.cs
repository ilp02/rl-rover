using UnityEngine;
using Unity.MLAgents;

/// <summary>
/// Central gameplay controller: detects out‑of‑bounds, applies penalties, and
/// provides spawn parameters to rovers. **It no longer moves agents itself** –
/// each rover chooses its own spawn pose in <c>OnEpisodeBegin</c>.
/// </summary>
public class TagGameController : MonoBehaviour
{

    [Header("Agents")]
    public RoverTom tom;
    public RoverJerry jerry;

    [Header("Arena Bounds (half‑extents)")]
    public float arenaHalfX = 5f;   // 10 m total width
    public float arenaHalfZ = 7.5f; // 15 m total depth

    [Header("Tag parameters")]
    public float tagDistance = 2f;
    public float tagReward   = 20.0f;   // given to Tom
    public float tagPenalty  = 0.0f;  // given to Jerry

    [Header("Spawn Bounds (half‑extents)")]
    public float spawnHalfX = 4f;   // 8 m rectangle
    public float spawnHalfZ = 6f;   // 12 m rectangle

    [Header("Gameplay")]
    public float minSpawnSeparation = 2f;
    public float outPenalty = -20.0f;


    // ─────────────────── Spawn helpers ───────────────────
    public Vector3 RandomInSpawn()
    {
        return new Vector3(Random.Range(-spawnHalfX, spawnHalfX), 0f,
                           Random.Range(-spawnHalfZ, spawnHalfZ));
    }

    public bool TooClose(Vector3 pos, RoverAgent me)
    {
        // Only need to check against the ONE other rover
        RoverAgent other = (me == (RoverAgent)tom) ? (RoverAgent)jerry : tom;
        if (other == null) return false;
        return Vector3.Distance(pos, other.transform.localPosition) < minSpawnSeparation;
    }

    // ─────────────────── Gameplay loop ───────────────────
    void FixedUpdate()
    {
        // Bounds check for Tom
        if (IsOutOfBounds(tom.transform.localPosition))
            PenaliseOut(tom, jerry);

        // Bounds check for Jerry
        if (IsOutOfBounds(jerry.transform.localPosition))
            PenaliseOut(jerry, tom);

        // Tag detection
        float d = Vector3.Distance(tom.transform.localPosition, jerry.transform.localPosition);
        if (d < tagDistance)
            HandleTag();
    }

    bool IsOutOfBounds(Vector3 pos)
        => Mathf.Abs(pos.x) > arenaHalfX || Mathf.Abs(pos.z) > arenaHalfZ;

    void PenaliseOut(RoverAgent offender, RoverAgent other)
    {
        offender.AddReward(outPenalty);
        other.AddReward(Mathf.Abs(outPenalty) * 0.5f);

        offender.EndEpisode();
        other.EndEpisode();
    }
    private void HandleTag()
    {
        tom.AddReward(tagReward);
        jerry.AddReward(tagPenalty);
        tom.EndEpisode();
        jerry.EndEpisode();
    }
}