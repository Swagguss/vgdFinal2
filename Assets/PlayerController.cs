using System.Collections;
using UnityEngine;
[DefaultExecutionOrder(-1)]
public class PlayerController : MonoBehaviour
{
    public Camera cam;

    private RopeController currentRope = null;
    private int currentSpan = -1;

    private SpringJoint2D footAnchorJoint = null;
    private Rigidbody2D ropeAnchorRB = null;

    public GameObject thighJoint, calfJoint, footJoint;
    private Rigidbody2D thighRB, calfRB, footRB;
    private Foot foot;

    public Transform targetPos, mouseTest, thighWish;

    public float Kp = 600f, Kd = 80f, torqueCap = 2000f, maxJumpForce = 200f;

    [Header("Rope anchor")]
    public float anchorRange = 1.5f;
    public LayerMask anchorLayerMask;
    public Transform selector;
    public float swingSpeed = 800f;

    private float lenThigh, lenCalf;
    private float thighBaseRotation, calfBaseRotation;
    private float desiredThighDeg, desiredCalfDeg;
    private Vector2 wishPos;

    private Vector2 prevWishPos = Vector2.zero;
    private bool isCharging = false;
    private float chargeDepth = 0f;
    public float pressDeadZone = 0.1f;


    private void Start()
    {
        thighRB = thighJoint.GetComponent<Rigidbody2D>();
        calfRB = calfJoint.GetComponent<Rigidbody2D>();
        footRB = footJoint.GetComponent<Rigidbody2D>();
        foot = footJoint.GetComponent<Foot>();

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
            Destroy(ropeAnchorRB.gameObject);

            currentRope.hangingSectionIndex = -1;
            currentRope = null;
            currentSpan = -1;
            footAnchorJoint = null;
            ropeAnchorRB = null;
            return;
        }

        Collider2D[] hits = Physics2D.OverlapCircleAll(footJoint.transform.position,
                                                       anchorRange, anchorLayerMask);

        RopeController.SpanCollider best = null;
        float bestD2 = float.PositiveInfinity;

        foreach (var h in hits)
        {
            var sc = h.GetComponent<RopeController.SpanCollider>();
            if (!sc) continue;

            float d2 = ((Vector2)footJoint.transform.position -
                        (Vector2)h.transform.position).sqrMagnitude;
            if (d2 < bestD2) { bestD2 = d2; best = sc; }
        }
        if (!best) return;

        Vector2 nodePos = best.GetComponentInParent<RopeController>()
                              .GetNodePos(best.SpanIndex);

        GameObject dummy = new GameObject("RopeAnchorRB");
        ropeAnchorRB = dummy.AddComponent<Rigidbody2D>();
        ropeAnchorRB.bodyType = RigidbodyType2D.Kinematic;
        ropeAnchorRB.position = nodePos;

        footAnchorJoint = footJoint.AddComponent<SpringJoint2D>();
        footAnchorJoint.connectedBody = ropeAnchorRB;
        footAnchorJoint.autoConfigureDistance = false;
        footAnchorJoint.distance = 0.02f;
        footAnchorJoint.enableCollision = false;
        footAnchorJoint.frequency = 5f;
        footAnchorJoint.dampingRatio = 0.7f;

        currentRope = best.GetComponentInParent<RopeController>();
        currentSpan = best.SpanIndex;

        currentRope.hangingSectionIndex = currentSpan;
    }

    void Update()
    {
        UpdateSelectorGizmo();

        if (Input.GetMouseButtonDown(0))
            TryToggleAnchor();

        wishPos = cam.ScreenToWorldPoint(Input.mousePosition);
        mouseTest.position = wishPos;
        targetPos.position = wishPos;

        SolveTwoBoneIK();
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

        RopeController.SpanCollider closest = null;
        float best = float.PositiveInfinity;
        foreach (var h in hits)
        {
            var sc = h.GetComponent<RopeController.SpanCollider>();
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
        if (currentRope && ropeAnchorRB)
            ropeAnchorRB.MovePosition(currentRope.GetNodePos(currentSpan));
        if (currentRope && footAnchorJoint)
        {
            Vector2 F = ((Vector2)footJoint.transform.position - wishPos).normalized * swingSpeed;

            float playerMass = 3f*(footRB.mass + calfRB.mass + thighRB.mass);
            F -= playerMass * Physics2D.gravity;

            const float MaxExt = 4000f;
            if (F.sqrMagnitude > MaxExt * MaxExt)
                F = F.normalized * MaxExt;

            currentRope.AddExternalForce(currentSpan, -F);
        }

        ApplyPD(thighRB, desiredThighDeg);
        ApplyPD(calfRB, desiredThighDeg + desiredCalfDeg);

        if (foot.collision != null && foot.collision.contacts.Length > 0)
        {
            Vector2 n = foot.collision.contacts[0].normal;
            ApplyPD(footRB, Mathf.Atan2(n.y, n.x) * Mathf.Rad2Deg);

            if (foot.isGrounded)
            {
                Vector2 normal = n.normalized;
                Vector2 mouseDelta = wishPos - prevWishPos;
                float press = Vector2.Dot(mouseDelta, normal);

                if (press > pressDeadZone) { isCharging = true; chargeDepth += press; }
                bool released = isCharging && press < -pressDeadZone;
                if (released)
                {
                    Jump(chargeDepth, -mouseDelta);
                    chargeDepth = 0f; isCharging = false;
                }
            }
        }
        else { chargeDepth = 0f; isCharging = false; }

        prevWishPos = wishPos;
    }

    void Jump(float depth, Vector2 direction)
    {
        Vector2 impulse = direction * Mathf.Clamp(depth, 0f, maxJumpForce);
        thighRB.AddForce(impulse, ForceMode2D.Impulse);
        calfRB.AddForce(impulse, ForceMode2D.Impulse);
        StartCoroutine(BoostPD(0.12f));
    }

    IEnumerator BoostPD(float t)
    {
        float kp0 = Kp; Kp *= 2f;
        yield return new WaitForSeconds(t);
        Kp = kp0;
    }

    void ApplyPD(Rigidbody2D rb, float desiredDeg)
    {
        float errorRad = Mathf.DeltaAngle(rb.rotation, desiredDeg) * Mathf.Deg2Rad;
        float velRad = rb.angularVelocity * Mathf.Deg2Rad;

        float torque = Kp * errorRad - Kd * velRad;
        torque = Mathf.Clamp(torque, -torqueCap, torqueCap);
        rb.AddTorque(torque, ForceMode2D.Force);
    }
}