using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public TMP_Text depthText;
    public Slider depthBar;
    public GameObject player;
    public GameObject gameOverPanel;
    private PlayerController playerController;
    public bool gameOver = false;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        playerController = player.GetComponent<PlayerController>();
        gameOverPanel.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log((playerController.thighJoint.transform.position.y - 1940f) / 1940f);
        depthText.text = "Depth: " + Mathf.Round((playerController.thighJoint.transform.position.y - 1940f)) + "m";
        depthBar.value = 1f + (playerController.thighJoint.transform.position.y - 1940f) / 1940f;
    }

    IEnumerator Reload()
    {
        yield return new WaitForSeconds(6);
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

    public void GameOver()
    {
        gameOver = true;
        gameOverPanel.SetActive(true);
        TMP_Text gameOverText = gameOverPanel.GetComponentInChildren<TMP_Text>();
        gameOverText.CrossFadeAlpha(0, 0f, false);
        gameOverText.CrossFadeAlpha(1, 5f, false);
        StartCoroutine(Reload());
    }
}
