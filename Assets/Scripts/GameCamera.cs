using Unity.Cinemachine;
using UnityEngine;

public class GameCamera : MonoBehaviour
{
    public string playerGameObjectName = "Player";
    private CinemachineCamera cineCam;
    void Start()
    {
        cineCam = GetComponent<CinemachineCamera>();
    }

    void Update()
    {
        if (cineCam.Follow == null) cineCam.Follow = GameObject.Find(playerGameObjectName).transform.Find("Thigh").transform;
    }
}
