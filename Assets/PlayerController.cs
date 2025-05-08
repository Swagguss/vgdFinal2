using System.Collections;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public Camera cam;
    [Header("Bones/Rigidbodies")]
    public GameObject thighJoint;
    public GameObject calfJoint;
    public GameObject footJoint;
    private Rigidbody2D thighRB;
    private Rigidbody2D calfRB;
    private Rigidbody2D footRB;
    private Foot foot;

    [Header("IK helpers")]
    public Transform targetPos;
    public Transform mouseTest; // debug sphere
    public Transform thighWish; // Keeps thigh upright

    [Header("PD gains")]
    public float Kp = 600f;
    public float Kd = 80f;
    public float torqueCap = 2000f;
    public float maxJumpForce = 200f;

    private float lenThigh, lenCalf;
    private float thighBaseRotation, calfBaseRotation;

    private float desiredThighDeg, desiredCalfDeg;
    private Vector2 wishPos = Vector2.zero;

    void Start()
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

    void Update()
    {
        if (Input.GetMouseButton(0))
            wishPos = cam.ScreenToWorldPoint(Input.mousePosition);
        mouseTest.position = wishPos;
        targetPos.position = wishPos;

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

            if (calfA <= 0f)
            {
                joint0 = thighA;
                joint1 = calfA;
            }
            else
            {
                joint0 = thighB;
                joint1 = calfB;
            }
        }

        desiredThighDeg = thighBaseRotation + joint0;
        desiredCalfDeg = calfBaseRotation + joint1;
    }

    private Vector2 prevWishPos = Vector2.zero;
    private bool isCharging = false;
    private float chargeDepth = 0f;
    public float pressDeadZone = 0.1f;

    void FixedUpdate()
    {
        ApplyPD(thighRB, desiredThighDeg);
        ApplyPD(calfRB, desiredThighDeg + desiredCalfDeg);
        if (foot.collision != null)
        {
            if (foot.collision.contacts.Length > 0)
            {
                Vector2 n = foot.collision.contacts[0].normal;
                ApplyPD(footRB, Mathf.Atan2(n.y, n.x) * Mathf.Rad2Deg);
                if (foot.isGrounded)
                {
                    Vector2 normal = foot.collision.contacts[0].normal.normalized;

                    Vector2 mouseDelta = wishPos - prevWishPos;
                    float press = Vector2.Dot(mouseDelta, normal);

                    if (press > pressDeadZone)
                    {
                        isCharging = true;
                        chargeDepth += press;
                    }

                    bool released = isCharging && press < -pressDeadZone;

                    if (released)
                    {
                        Jump(chargeDepth, -mouseDelta);
                        chargeDepth = 0f;
                        isCharging = false;
                    }
                }
            }
        }
        else
        {
            chargeDepth = 0f;
            isCharging = false;
        }

        prevWishPos = wishPos;
    }

    void Jump(float depth, Vector2 direction)
    {
        Vector2 impulse = direction*Mathf.Clamp(depth, 0f, maxJumpForce);

        thighRB.AddForce(impulse, ForceMode2D.Impulse);
        calfRB.AddForce(impulse, ForceMode2D.Impulse);

        StartCoroutine(BoostPD(0.12f));
    }

    IEnumerator BoostPD(float t)
    {
        float kp0 = Kp;
        Kp *= 2f;
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
