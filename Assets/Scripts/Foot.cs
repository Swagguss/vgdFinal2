using UnityEngine;

public class Foot : MonoBehaviour
{
    public float frictionMax = 2500f;
    public float torqueMax = 600f;
    public AudioSource footstep;
    public AudioClip[] clips;
    public bool useFJ = true;

    Rigidbody2D rb;
    FrictionJoint2D fj;

    int contactCount = 0;
    public Collision2D collision;
    float lastStepSound = 0f;
    int framesClear = 100;

    void Awake()
    {
        rb = GetComponent<Rigidbody2D>();

        if (useFJ)
        {
            fj = gameObject.AddComponent<FrictionJoint2D>();
            fj.enabled = false;
            fj.enableCollision = true;
            fj.maxForce = frictionMax;
            fj.maxTorque = torqueMax;
            fj.autoConfigureConnectedAnchor = false;
        }
    }

    private void FixedUpdate()
    {
        if (useFJ)
        {
            if (contactCount == 0)
                framesClear++;
            else framesClear = 0;
            if (framesClear >= 5)
                fj.enabled = false;
        }
    }

    void OnCollisionEnter2D(Collision2D col)
    {
        collision = col;
        contactCount++;

        if (Time.time - lastStepSound > 0.4f)
        {
            var c = clips[Random.Range(0, clips.Length)];
            footstep.pitch = 0.75f + Random.value * 0.5f;
            footstep.volume = col.relativeVelocity.magnitude * 0.2f;
            footstep.PlayOneShot(c);
            lastStepSound = Time.time;
        }

        if (useFJ && !fj.enabled)
        {
            Vector2 local = rb.transform.InverseTransformPoint(col.contacts[0].point);
            local.y += 0.015f;
            fj.anchor = local;

            if (col.rigidbody == null)
            {
                fj.connectedBody = null;
                fj.connectedAnchor = col.contacts[0].point;
            }
            else
            {
                fj.connectedBody = col.rigidbody;
                fj.connectedAnchor =
                    col.rigidbody.transform.InverseTransformPoint(col.contacts[0].point);
            }

            fj.enabled = true;
        }
    }

    void OnCollisionExit2D(Collision2D col)
    {
        collision = null;
        contactCount = Mathf.Max(0, contactCount - 1);
    }
}
