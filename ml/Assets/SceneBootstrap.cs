using UnityEngine;

public class SceneBootstrap : MonoBehaviour
{
    public GameObject setPrefab;
    public int rows = 3, cols = 4;     // 12 arenas
    public float spacingX = 20f, spacingZ = 25f;

    void Awake()
    {
        for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
        {
            Vector3 p = new Vector3(
                (c - (cols-1)*0.5f) * spacingX,
                0f,
                (r - (rows-1)*0.5f) * spacingZ);
            Instantiate(setPrefab, p, Quaternion.identity, transform);
        }
    }
}