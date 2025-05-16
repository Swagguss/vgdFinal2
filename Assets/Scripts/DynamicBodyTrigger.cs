using UnityEngine;

public class DynamicBodyTrigger : MonoBehaviour
{
    public Rigidbody2D body;
    public LayerMask mask;
    private void OnTriggerEnter2D(Collider2D collision)
    {
        if ((mask & (1 << collision.gameObject.layer)) != 0)
            body.bodyType = RigidbodyType2D.Dynamic;
    }
}
