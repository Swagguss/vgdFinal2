using System.Collections;
using System.Collections.Generic;
using UnityEngine;
[DefaultExecutionOrder(1)]
public class RopeController : MonoBehaviour
{
    public Transform whatTheRopeIsConnectedTo;
    public Transform whatIsHangingFromTheRope;
    [HideInInspector]
    public Rigidbody2D whatIsHangingFromTheRopeRb;
    internal bool footIsKinematic = false;
    public int hangingSectionIndex = -1;

    public LineRenderer lineRenderer;

    public List<RopeSection> allRopeSections = new List<RopeSection>();

    //Rope data
    private float ropeSectionLength;

    public int spans = 5;
    public float ropeLength = 2f;
    //Spring constant
    public float ropeSpringiness = 40f;
    //Damping from rope friction constant
    public float ropeFriction = 2f;
    //Damping from air resistance constant
    public float ropeAirResistance = 0.05f;
    //Mass of one rope section
    public float ropeSegmentMass = 0.2f;
    [Header("Collider generation")]
    [Tooltip("Capsule radius in meters for per-span grab colliders")]
    public float colliderRadius = 0.05f;

    [Tooltip("Extra slack so colliders do not lag behind fast rope motion")]
    public float colliderLengthPadding = 0.02f;
    private readonly List<SpanCollider> spanColliders = new();

    public Vector2 GetNodePos(int index) => allRopeSections[index].pos;

    private Vector3[] extForces;

    private void ClearExternalForces()
    {
        if (extForces == null || extForces.Length != allRopeSections.Count)
            extForces = new Vector3[allRopeSections.Count];

        for (int i = 0; i < extForces.Length; i++)
            extForces[i] = Vector3.zero;
    }

    public void AddExternalForce(int index, Vector3 force)
    {
        if (extForces == null || index < 0 || index >= extForces.Length) return;
        extForces[index] += force;
    }

    private void Start()
    {
        ropeSectionLength = ropeLength / spans;
        whatIsHangingFromTheRopeRb = whatIsHangingFromTheRope.GetComponent<Rigidbody2D>();
        Vector3 p = whatTheRopeIsConnectedTo.position;
        for (int i = 0; i <= spans; i++)
        {
            allRopeSections.Add(new RopeSection(p));
            p.y -= ropeSectionLength;
        }

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            GameObject g = new GameObject($"RopeSpanCollider_{i}");
            g.transform.parent = transform;
            CapsuleCollider2D cc = g.AddComponent<CapsuleCollider2D>();
            cc.gameObject.layer = 7;
            cc.isTrigger = true;
            cc.direction = CapsuleDirection2D.Vertical;

            SpanCollider sc = g.AddComponent<SpanCollider>();
            sc.Setup(this, i, cc);
            spanColliders.Add(sc);
        }
        ClearExternalForces();
    }

    private void Update()
    {
        DisplayRope();

        if (hangingSectionIndex >= 0 && hangingSectionIndex < allRopeSections.Count - 1)
        {
            if (whatIsHangingFromTheRopeRb != null && footIsKinematic)
                whatIsHangingFromTheRopeRb.MovePosition(allRopeSections[hangingSectionIndex].pos);
        }
    }

    private void FixedUpdate()
    {
        if (allRopeSections.Count == 0) return;

        const int iterations = 1;
        float dt = Time.fixedDeltaTime / iterations;

        for (int k = 0; k < iterations; k++)
            UpdateRopeSimulation(allRopeSections, dt);

        for (int i = 0; i < spanColliders.Count; i++)
            spanColliders[i].MatchSpan(allRopeSections[i].pos,
                                       allRopeSections[i + 1].pos,
                                       colliderRadius,
                                       colliderLengthPadding);
        ClearExternalForces();
    }

    internal void GrabSpan(int spanIndex)
    {
        hangingSectionIndex = spanIndex;
    }

    internal void ReleaseSpan(int spanIndex)
    {
        if (hangingSectionIndex == spanIndex)
            hangingSectionIndex = -1;
    }

    private void DisplayRope()
    {
        float ropeWidth = 0.2f;

        lineRenderer.startWidth = ropeWidth;
        lineRenderer.endWidth = ropeWidth;

        Vector3[] positions = new Vector3[allRopeSections.Count];

        for (int i = 0; i < allRopeSections.Count; i++)
        {
            positions[i] = allRopeSections[i].pos;
        }

        lineRenderer.positionCount = positions.Length;

        lineRenderer.SetPositions(positions);
    }

    private void UpdateRopeSimulation(List<RopeSection> allRopeSections, float timeStep)
    {
        RopeSection lastRopeSection = allRopeSections[allRopeSections.Count - 1];

        lastRopeSection.pos = whatTheRopeIsConnectedTo.position;

        allRopeSections[allRopeSections.Count - 1] = lastRopeSection;

        List<Vector3> accelerations = CalculateAccelerations(allRopeSections, timeStep);

        List<RopeSection> nextPosVelForwardEuler = new List<RopeSection>();

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            RopeSection thisRopeSection = RopeSection.zero;

            //Forward Euler
            //vel = vel + acc * t
            thisRopeSection.vel = allRopeSections[i].vel + accelerations[i] * timeStep;

            //pos = pos + vel * t
            thisRopeSection.pos = allRopeSections[i].pos + allRopeSections[i].vel * timeStep;
            nextPosVelForwardEuler.Add(thisRopeSection);
        }

        nextPosVelForwardEuler.Add(allRopeSections[allRopeSections.Count - 1]);

        List<Vector3> accelerationFromEuler = CalculateAccelerations(nextPosVelForwardEuler, timeStep);

        List<RopeSection> nextPosVelHeunsMethod = new List<RopeSection>();

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            RopeSection thisRopeSection = RopeSection.zero;

            //Heuns method
            //vel = vel + (acc + accFromForwardEuler) * 0.5 * t
            thisRopeSection.vel = allRopeSections[i].vel + (accelerations[i] + accelerationFromEuler[i]) * 0.5f * timeStep;

            //pos = pos + (vel + velFromForwardEuler) * 0.5f * t
            thisRopeSection.pos = allRopeSections[i].pos + (allRopeSections[i].vel + nextPosVelForwardEuler[i].vel) * 0.5f * timeStep;

            nextPosVelHeunsMethod.Add(thisRopeSection);
        }

        nextPosVelHeunsMethod.Add(allRopeSections[allRopeSections.Count - 1]);


        for (int i = 0; i < allRopeSections.Count; i++)
        {
            allRopeSections[i] = nextPosVelHeunsMethod[i];
        }

        int maximumStretchIterations = 2;

        for (int i = 0; i < maximumStretchIterations; i++)
        {
            ImplementMaximumStretch(allRopeSections);
        }
    }

    private List<Vector3> CalculateAccelerations(List<RopeSection> allRopeSections, float timeStep)
    {
        List<Vector3> accelerations = new List<Vector3>();

        //Spring constant
        float k = ropeSpringiness;
        //Damping constant
        float d = ropeFriction;
        //Damping constant from air resistance
        float a = ropeAirResistance;
        //Mass of one rope section
        float m = ropeSegmentMass;
        //How long should the rope section be
        float wantedLength = ropeSectionLength;


        List<Vector3> allForces = new List<Vector3>();

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            Vector3 vectorBetween = allRopeSections[i + 1].pos - allRopeSections[i].pos;

            float distanceBetween = vectorBetween.magnitude;
            if (distanceBetween < 1e-6f) distanceBetween = 1e-6f;

            Vector3 dir = vectorBetween / distanceBetween;

            float springForce = k * (distanceBetween - wantedLength);

            float frictionForce = d * ((Vector3.Dot(allRopeSections[i + 1].vel - allRopeSections[i].vel, vectorBetween)) / distanceBetween);

            Vector3 springForceVec = -(springForce + frictionForce) * dir;

            springForceVec = -springForceVec;

            allForces.Add(springForceVec);
        }

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            Vector3 springForce = Vector3.zero;

            springForce += allForces[i];

            if (i != 0)
            {
                springForce -= allForces[i - 1];
            }

            float vel = allRopeSections[i].vel.magnitude;

            Vector3 dampingForce = a * vel * vel * allRopeSections[i].vel.normalized;

            float springMass = m;

            if (i == hangingSectionIndex)
            {
                springMass += whatIsHangingFromTheRope.GetComponent<Rigidbody2D>().mass;
            }

            Vector3 gravityForce = springMass * new Vector3(0f, -9.81f, 0f);
            Vector3 totalForce = springForce + gravityForce - dampingForce + extForces[i];

            Vector3 acceleration = totalForce / springMass;
            accelerations.Add(acceleration);
        }

        return accelerations;
    }

    private void ImplementMaximumStretch(List<RopeSection> allRopeSections)
    {
        float maxStretch = 1.1f;
        float minStretch = 0.9f;

        for (int i = allRopeSections.Count - 1; i > 0; i--)
        {
            RopeSection topSection = allRopeSections[i];

            RopeSection bottomSection = allRopeSections[i - 1];

            float dist = (topSection.pos - bottomSection.pos).magnitude;

            float stretch = dist / ropeSectionLength;

            if (stretch > maxStretch)
            {
                float overshoot = dist - ropeSectionLength * maxStretch;
                Vector3 dir = SafeNormal(topSection.pos - bottomSection.pos);
                Vector3 change = dir * overshoot;
                MoveSection(change, i - 1);
            }
            else if (stretch < minStretch)
            {
                float deficit = ropeSectionLength * minStretch - dist;
                Vector3 dir = SafeNormal(bottomSection.pos - topSection.pos);
                Vector3 change = dir * deficit;
                MoveSection(change, i - 1);
            }
        }
    }

    private void MoveSection(Vector3 finalChange, int listPos)
    {
        RopeSection bottomSection = allRopeSections[listPos];

        Vector3 pos = bottomSection.pos;

        pos += finalChange;

        bottomSection.pos = pos;

        allRopeSections[listPos] = bottomSection;
    }

    private static Vector3 SafeNormal(Vector3 v)
    {
        float m = v.magnitude;
        return m > 1e-6 ? v / m : Vector3.zero;
    }

    internal sealed class SpanCollider : MonoBehaviour
    {
        private RopeController rope;
        private int spanIndex;
        private CapsuleCollider2D col;

        public int SpanIndex => spanIndex;

        internal void Setup(RopeController owner, int index, CapsuleCollider2D capsule)
        {
            rope = owner;
            spanIndex = index;
            col = capsule;
        }

        internal void MatchSpan(Vector3 a, Vector3 b, float radius, float lengthPad)
        {
            Vector3 mid = (a + b) * 0.5f;
            Vector3 diff = b - a;
            float len = diff.magnitude + lengthPad;
            if (len < 1e-4f) len = 1e-4f;

            // Position
            transform.position = mid;

            if (diff.sqrMagnitude > 1e-12f)
            transform.rotation = Quaternion.FromToRotation(Vector3.forward,
            SafeNormal(diff));

            // Scale collider
            col.size = new Vector2(radius, len);
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.transform == rope.whatIsHangingFromTheRope) return;

            rope.GrabSpan(spanIndex);
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.transform == rope.whatIsHangingFromTheRope) return;

            rope.ReleaseSpan(spanIndex);
        }
    }
}