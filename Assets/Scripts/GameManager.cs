using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    private TMP_Text depthText;
    private Slider depthBar;
    private GameObject player;
    private GameObject gameOverPanel;
    private PlayerController playerController;
    private Transform panel;
    private float depth = -359f;
    private float bestDepth = -359f;
    public bool gameOver = false;
    public Vector3 offset = Vector3.zero;
    void Start()
    {
        depthBar = GameObject.Find("Slider").GetComponent<Slider>();
        depthText = GameObject.Find("DepthText").GetComponent<TMP_Text>();
        gameOverPanel = GameObject.Find("GameOver");
        player = GameObject.Find("Player");
        playerController = player.GetComponent<PlayerController>();
        panel = GameObject.Find("Panel").transform;
        gameOverPanel.SetActive(false);
    }
    void Update()
    {
        depth = playerController.thighJoint.transform.position.y - 400f;
        bestDepth = Mathf.Max(depth, bestDepth);
        depthText.text = "Depth: " + Mathf.Round(depth) + "m";
        depthBar.value = 1f + (depth) / 400f;
        panel.transform.localPosition = new Vector3(depthBar.gameObject.GetComponent<RectTransform>().sizeDelta.x * (bestDepth + 400f) / 400f - depthBar.gameObject.GetComponent<RectTransform>().sizeDelta.x * 0.5f, 0f) + offset;
    }

    IEnumerator Reload()
    {
        yield return new WaitForSeconds(3);
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

    public void GameOver()
    {
        gameOver = true;
        gameOverPanel.SetActive(true);
        TMP_Text gameOverText = gameOverPanel.GetComponentInChildren<TMP_Text>();
        gameOverText.CrossFadeAlpha(0, 0f, false);
        gameOverText.CrossFadeAlpha(1, 1f, false);
        StartCoroutine(Reload());
    }
}
