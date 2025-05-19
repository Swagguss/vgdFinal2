using System;
using System.Collections;
using UnityEngine;
[DefaultExecutionOrder(-1)]
public class PlayerController : MonoBehaviour
{
    private RopeController currentRope = null;
    private int currentSpan = -1;

    private DistanceJoint2D footAnchorJoint = null;
    private readonly Rigidbody2D ropeAnchorRB = null;

    public GameObject thighJoint, calfJoint, footJoint;
    private Rigidbody2D thighRB, calfRB, footRB;
    private Foot foot;

    public Transform targetPos, mouseTest, thighWish;
    private BoxCollider2D bounds;

    public float Kp = 600f, Kd = 80f, torqueCap = 2000f;

    [Header("Rope anchor")]
    public float anchorRange = 1.5f;
    public LayerMask anchorLayerMask;
    public Transform selector;

    private float lenThigh, lenCalf;
    private float thighBaseRotation, calfBaseRotation;
    private float desiredThighDeg, desiredCalfDeg;
    private Vector2 wishPos;

    private Vector2 prevWishPos = Vector2.zero;
    private GameManager gameManager;

    private void Start()
    {
        gameManager = GameObject.Find("GameManager").GetComponent<GameManager>();
        thighRB = thighJoint.GetComponent<Rigidbody2D>();
        calfRB = calfJoint.GetComponent<Rigidbody2D>();
        footRB = footJoint.GetComponent<Rigidbody2D>();
        foot = footJoint.GetComponent<Foot>();
        bounds = gameObject.GetComponent<BoxCollider2D>();

        lenThigh = Vector2.Distance(thighJoint.transform.position, calfJoint.transform.position);
        lenCalf = Vector2.Distance(calfJoint.transform.position, footJoint.transform.position);

        thighBaseRotation = thighJoint.transform.localEulerAngles.z;
        calfBaseRotation = calfJoint.transform.localEulerAngles.z;
    }

    void TryToggleAnchor()
    {
        if (currentRope != null)
        {
            Destroy(footAnchorJoint);

            currentRope = null;
            currentSpan = -1;
            footAnchorJoint = null;
            return;
        }

        Collider2D[] hits = Physics2D.OverlapCircleAll(footJoint.transform.position,
                                                       anchorRange, anchorLayerMask);

        CapsuleCollider2D best = null;
        int bestIndex = -1;
        float bestD2 = float.PositiveInfinity;

        foreach (var h in hits)
        {
            var sc = h.GetComponent<CapsuleCollider2D>();
            if (!sc) continue;

            float d2 = ((Vector2)footJoint.transform.position -
                        (Vector2)h.transform.position).sqrMagnitude;
            if (d2 < bestD2) { bestD2 = d2; best = sc; bestIndex = h.GetComponent<RopeLink>().index; }
        }
        if (!best) return;

        Debug.Log(bestIndex);
        currentRope = best.GetComponentInParent<RopeController>();
        currentSpan = bestIndex;
        Rigidbody2D linkBody = currentRope.bodies[bestIndex];

        footAnchorJoint = footJoint.AddComponent<DistanceJoint2D>();
        footAnchorJoint.connectedBody = linkBody;
        footAnchorJoint.autoConfigureDistance = false;
        footAnchorJoint.distance = 0.005f;
        footAnchorJoint.enableCollision = false;
    }

    void UpdateSelectorGizmo()
    {
        Collider2D[] hits = Physics2D.OverlapCircleAll(footJoint.transform.position,
                                                       anchorRange,
                                                       anchorLayerMask);

        if (hits.Length == 0)
        {
            selector.gameObject.SetActive(false);
            return;
        }

        CapsuleCollider2D closest = null;
        float best = float.PositiveInfinity;
        foreach (var h in hits)
        {
            var sc = h.GetComponent<CapsuleCollider2D>();
            if (sc == null) continue;

            float d2 = ((Vector2)footJoint.transform.position - (Vector2)h.transform.position).sqrMagnitude;
            if (d2 < best) { best = d2; closest = sc; }
        }

        if (closest == null)
        {
            selector.gameObject.SetActive(false);
            return;
        }

        selector.gameObject.SetActive(true);
        selector.position = new Vector3(closest.transform.position.x,
                                        closest.transform.position.y,
                                        -5f);
    }

    void Update()
    {
        if (!gameManager.gameOver)
        {
            UpdateSelectorGizmo();
            if (Input.GetMouseButtonDown(0))
                TryToggleAnchor();

            wishPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            mouseTest.position = wishPos;
            targetPos.position = wishPos;

            SolveTwoBoneIK();
            Bounds playerBoundary = GetMaxBounds(thighJoint.transform.position, gameObject);
            bounds.offset = (Vector2)playerBoundary.center;
            bounds.size = (Vector2)playerBoundary.size;
        }
    }

    void SolveTwoBoneIK()
    {
        float lenTarget = Vector2.Distance(thighJoint.transform.position, targetPos.position);
        Vector2 diff = (Vector2)targetPos.position - (Vector2)thighJoint.transform.position;
        float atan = Mathf.Atan2(diff.y, diff.x) * Mathf.Rad2Deg - 90f;

        float joint0, joint1;

        if (lenThigh + lenCalf < lenTarget)
        {
            joint0 = atan - 180f;
            joint1 = 0f;
        }
        else
        {
            float a = Mathf.Acos(Mathf.Clamp((lenTarget * lenTarget + lenThigh * lenThigh - lenCalf * lenCalf) /
                                             (2f * lenTarget * lenThigh), -1f, 1f)) * Mathf.Rad2Deg;
            float b = Mathf.Acos(Mathf.Clamp((lenCalf * lenCalf + lenThigh * lenThigh - lenTarget * lenTarget) /
                                             (2f * lenCalf * lenThigh), -1f, 1f)) * Mathf.Rad2Deg;

            float thighA = atan - a;
            float calfA = 180f - b;

            float thighB = atan + a + 180f;
            float calfB = -(180f - b);

            if (calfA <= 0f) { joint0 = thighA; joint1 = calfA; }
            else { joint0 = thighB; joint1 = calfB; }
        }

        desiredThighDeg = thighBaseRotation + joint0;
        desiredCalfDeg = calfBaseRotation + joint1;
    }


    void FixedUpdate()
    {
        if (!gameManager.gameOver)
        {
            if (currentRope && ropeAnchorRB)
            {
                Vector2 target = currentRope.bodies[currentSpan].transform.position;
                ropeAnchorRB.MovePosition(target);
            }

            ApplyPD(thighRB, desiredThighDeg);
            ApplyPD(calfRB, desiredThighDeg + desiredCalfDeg);

            float tangential = Mathf.Atan2((wishPos - prevWishPos).y, (wishPos - prevWishPos).x) - Mathf.Atan2(((Vector2)thighJoint.transform.position - prevWishPos).y, ((Vector2)thighJoint.transform.position - prevWishPos).x);
            Vector2 tangent = tangential > 0 ? Vector2.right : Vector2.left;
            Vector2 mouseDelta = wishPos - prevWishPos;
            Vector2 impulse = tangent * mouseDelta.magnitude * 5f;

            thighRB.AddForce(impulse, ForceMode2D.Force);
            if (foot.collision != null && foot.collision.contacts.Length > 0)
            {
                Vector2 n = foot.collision.contacts[0].normal;
                ApplyPD(footRB, Mathf.Atan2(n.y, n.x) * Mathf.Rad2Deg);
            }

            prevWishPos = wishPos;
        }
    }

    void ApplyPD(Rigidbody2D rb, float desiredDeg)
    {
        float errorRad = Mathf.DeltaAngle(rb.rotation, desiredDeg) * Mathf.Deg2Rad;
        float velRad = rb.angularVelocity * Mathf.Deg2Rad;

        float torque = Kp * errorRad - Kd * velRad;
        torque = Mathf.Clamp(torque, -torqueCap, torqueCap);
        rb.AddTorque(torque, ForceMode2D.Force);
    }

    Bounds GetMaxBounds(Vector2 origin, GameObject g)
    {
        var b = new Bounds(origin, Vector2.zero);
        foreach (Renderer r in g.GetComponentsInChildren<Renderer>())
        {
            b.Encapsulate(r.bounds);
        }
        b.center -= transform.position;
        return b;
    }

    private void OnTriggerEnter2D(Collider2D collision)
    {
        if (!gameManager.gameOver)
            gameManager.GameOver();
    }
}