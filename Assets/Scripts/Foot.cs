using UnityEngine;
using UnityEngine.Audio;

public class Foot : MonoBehaviour
{
    public bool isGrounded = false;
    public Collision2D collision;
    public AudioSource footstep;
    public AudioClip[] clips;
    private float timeSinceCollided = 0f;

    private void OnCollisionEnter2D(Collision2D col)
    {
        if (timeSinceCollided > 0.4f)
        {
            footstep.clip = clips[Random.Range(0, clips.Length - 1)];
            footstep.pitch = Random.value + 0.5f;
            footstep.Play();
            timeSinceCollided = 0f;
        }
    }

    private void FixedUpdate()
    {
        timeSinceCollided += Time.deltaTime;
    }
}
