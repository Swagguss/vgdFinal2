using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RopeController : MonoBehaviour
{
    //Objects that will interact with the rope
    public Transform whatTheRopeIsConnectedTo;
    public Transform whatIsHangingFromTheRope;
    public int hangingSectionIndex = -1;

    //Line renderer used to display the rope
    public LineRenderer lineRenderer;

    //A list with all rope section
    public List<RopeSection> allRopeSections = new List<RopeSection>();

    //Rope data
    private float ropeSectionLength = 1f;

    //Data we can change to change the properties of the rope
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
    private readonly List<SpanCollider> spanColliders = new();   // one per spring

    private void Start()
    {
        // Build the rope's node list (simple vertical start shape)
        Vector3 p = whatTheRopeIsConnectedTo.position;
        for (int i = 0; i < 7; i++)
        {
            allRopeSections.Add(new RopeSection(p));
            p.y -= ropeSectionLength;
        }

        // Generate one capsule trigger for every span except the fixed top node
        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            GameObject g = new GameObject($"RopeSpanCollider_{i}");
            g.transform.parent = transform;               // keep hierarchy tidy
            CapsuleCollider2D cc = g.AddComponent<CapsuleCollider2D>();
            cc.gameObject.layer = 7;
            cc.isTrigger = true;
            cc.direction = CapsuleDirection2D.Vertical;

            SpanCollider sc = g.AddComponent<SpanCollider>();
            sc.Setup(this, i, cc);
            spanColliders.Add(sc);
        }
    }

    private void Update()
    {
        DisplayRope();

        if (hangingSectionIndex >= 0 && hangingSectionIndex < allRopeSections.Count - 1)
        {
            whatIsHangingFromTheRope.position =
                allRopeSections[hangingSectionIndex].pos;

            int lookAt = hangingSectionIndex == 0 ? 1 : hangingSectionIndex - 1;
            whatIsHangingFromTheRope.LookAt(allRopeSections[lookAt].pos);
        }
    }

    private void FixedUpdate()
    {
        if (allRopeSections.Count == 0) return;

        const int iterations = 1;
        float dt = Time.fixedDeltaTime / iterations;

        for (int k = 0; k < iterations; k++)
            UpdateRopeSimulation(allRopeSections, dt);

        // After simulation, update all span colliders to follow the rope
        for (int i = 0; i < spanColliders.Count; i++)
            spanColliders[i].MatchSpan(allRopeSections[i].pos,
                                       allRopeSections[i + 1].pos,
                                       colliderRadius,
                                       colliderLengthPadding);
    }

    /* ---------------------------------------------------------------------  COLLIDER CALLBACK  */

    internal void GrabSpan(int spanIndex)
    {
        hangingSectionIndex = spanIndex;
    }

    internal void ReleaseSpan(int spanIndex)
    {
        if (hangingSectionIndex == spanIndex)
            hangingSectionIndex = -1;
    }

    //Display the rope with a line renderer
    private void DisplayRope()
    {
        float ropeWidth = 0.2f;

        lineRenderer.startWidth = ropeWidth;
        lineRenderer.endWidth = ropeWidth;

        //An array with all rope section positions
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
        //Move the last position, which is the top position, to what the rope is attached to
        RopeSection lastRopeSection = allRopeSections[allRopeSections.Count - 1];

        lastRopeSection.pos = whatTheRopeIsConnectedTo.position;

        allRopeSections[allRopeSections.Count - 1] = lastRopeSection;


        //
        //Calculate the next pos and vel with Forward Euler
        //
        //Calculate acceleration in each rope section which is what is needed to get the next pos and vel
        List<Vector3> accelerations = CalculateAccelerations(allRopeSections);

        List<RopeSection> nextPosVelForwardEuler = new List<RopeSection>();

        //Loop through all line segments (except the last because it's always connected to something)
        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            RopeSection thisRopeSection = RopeSection.zero;

            //Forward Euler
            //vel = vel + acc * t
            thisRopeSection.vel = allRopeSections[i].vel + accelerations[i] * timeStep;

            //pos = pos + vel * t
            thisRopeSection.pos = allRopeSections[i].pos + allRopeSections[i].vel * timeStep;

            //Save the new data in a temporarily list
            nextPosVelForwardEuler.Add(thisRopeSection);
        }

        //Add the last which is always the same because it's attached to something
        nextPosVelForwardEuler.Add(allRopeSections[allRopeSections.Count - 1]);


        //
        //Calculate the next pos with Heun's method (Improved Euler)
        //
        //Calculate acceleration in each rope section which is what is needed to get the next pos and vel
        List<Vector3> accelerationFromEuler = CalculateAccelerations(nextPosVelForwardEuler);

        List<RopeSection> nextPosVelHeunsMethod = new List<RopeSection>();

        //Loop through all line segments (except the last because it's always connected to something)
        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            RopeSection thisRopeSection = RopeSection.zero;

            //Heuns method
            //vel = vel + (acc + accFromForwardEuler) * 0.5 * t
            thisRopeSection.vel = allRopeSections[i].vel + (accelerations[i] + accelerationFromEuler[i]) * 0.5f * timeStep;

            //pos = pos + (vel + velFromForwardEuler) * 0.5f * t
            thisRopeSection.pos = allRopeSections[i].pos + (allRopeSections[i].vel + nextPosVelForwardEuler[i].vel) * 0.5f * timeStep;

            //Save the new data in a temporarily list
            nextPosVelHeunsMethod.Add(thisRopeSection);
        }

        //Add the last which is always the same because it's attached to something
        nextPosVelHeunsMethod.Add(allRopeSections[allRopeSections.Count - 1]);



        //From the temp list to the main list
        for (int i = 0; i < allRopeSections.Count; i++)
        {
            allRopeSections[i] = nextPosVelHeunsMethod[i];

            //allRopeSections[i] = nextPosVelForwardEuler[i];
        }


        //Implement maximum stretch to avoid numerical instabilities
        //May need to run the algorithm several times
        int maximumStretchIterations = 2;

        for (int i = 0; i < maximumStretchIterations; i++)
        {
            ImplementMaximumStretch(allRopeSections);
        }
    }

    //Calculate accelerations in each rope section which is what is needed to get the next pos and vel
    private List<Vector3> CalculateAccelerations(List<RopeSection> allRopeSections)
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


        //Calculate all forces once because some sections are using the same force but negative
        List<Vector3> allForces = new List<Vector3>();

        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            //From Physics for game developers book
            //The force exerted on body 1
            //pos1 (above) - pos2
            Vector3 vectorBetween = allRopeSections[i + 1].pos - allRopeSections[i].pos;

            float distanceBetween = vectorBetween.magnitude;

            Vector3 dir = vectorBetween.normalized;

            float springForce = k * (distanceBetween - wantedLength);


            //Damping from rope friction 
            //vel1 (above) - vel2
            float frictionForce = d * ((Vector3.Dot(allRopeSections[i + 1].vel - allRopeSections[i].vel, vectorBetween)) / distanceBetween);


            //The total force on the spring
            Vector3 springForceVec = -(springForce + frictionForce) * dir;

            //This is body 2 if we follow the book because we are looping from below, so negative
            springForceVec = -springForceVec;

            allForces.Add(springForceVec);
        }


        //Loop through all line segments (except the last because it's always connected to something)
        //and calculate the acceleration
        for (int i = 0; i < allRopeSections.Count - 1; i++)
        {
            Vector3 springForce = Vector3.zero;

            //Spring 1 - above
            springForce += allForces[i];

            //Spring 2 - below
            //The first spring is at the bottom so it doesnt have a section below it
            if (i != 0)
            {
                springForce -= allForces[i - 1];
            }

            //Damping from air resistance, which depends on the square of the velocity
            float vel = allRopeSections[i].vel.magnitude;

            Vector3 dampingForce = a * vel * vel * allRopeSections[i].vel.normalized;

            //The mass attached to this spring
            float springMass = m;

            //end of the rope is attached to a box with a mass
            if (i == hangingSectionIndex)
            {
                springMass += whatIsHangingFromTheRope.GetComponent<Rigidbody>().mass;
            }

            //Force from gravity
            Vector3 gravityForce = springMass * new Vector3(0f, -9.81f, 0f);

            //The total force on this spring
            Vector3 totalForce = springForce + gravityForce - dampingForce;

            //Calculate the acceleration a = F / m
            Vector3 acceleration = totalForce / springMass;

            accelerations.Add(acceleration);
        }

        //The last line segment's acc is always 0 because it's attached to something
        accelerations.Add(Vector3.zero);


        return accelerations;
    }

    //Implement maximum stretch to avoid numerical instabilities
    private void ImplementMaximumStretch(List<RopeSection> allRopeSections)
    {
        //Make sure each spring are not less compressed than 90% nor more stretched than 110%
        float maxStretch = 1.1f;
        float minStretch = 0.9f;

        //Loop from the end because it's better to adjust the top section of the rope before the bottom
        //And the top of the rope is at the end of the list
        for (int i = allRopeSections.Count - 1; i > 0; i--)
        {
            RopeSection topSection = allRopeSections[i];

            RopeSection bottomSection = allRopeSections[i - 1];

            //The distance between the sections
            float dist = (topSection.pos - bottomSection.pos).magnitude;

            //What's the stretch/compression
            float stretch = dist / ropeSectionLength;

            if (stretch > maxStretch)
            {
                //How far do we need to compress the spring?
                float compressLength = dist - (ropeSectionLength * maxStretch);

                //In what direction should we compress the spring?
                Vector3 compressDir = (topSection.pos - bottomSection.pos).normalized;

                Vector3 change = compressDir * compressLength;

                MoveSection(change, i - 1);
            }
            else if (stretch < minStretch)
            {
                //How far do we need to stretch the spring?
                float stretchLength = (ropeSectionLength * minStretch) - dist;

                //In what direction should we compress the spring?
                Vector3 stretchDir = (bottomSection.pos - topSection.pos).normalized;

                Vector3 change = stretchDir * stretchLength;

                MoveSection(change, i - 1);
            }
        }
    }

    //Move a rope section based on stretch/compression
    private void MoveSection(Vector3 finalChange, int listPos)
    {
        RopeSection bottomSection = allRopeSections[listPos];

        //Move the bottom section
        Vector3 pos = bottomSection.pos;

        pos += finalChange;

        bottomSection.pos = pos;

        allRopeSections[listPos] = bottomSection;
    }

    //Compare the current length of the rope with the wanted length
    private void DebugRopeLength()
    {
        float currentLength = 0f;

        for (int i = 1; i < allRopeSections.Count; i++)
        {
            float thisLength = (allRopeSections[i].pos - allRopeSections[i - 1].pos).magnitude;

            currentLength += thisLength;
        }

        float wantedLength = ropeSectionLength * (float)(allRopeSections.Count - 1);

        print("Wanted: " + wantedLength + " Actual: " + currentLength);
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

            // Position
            transform.position = mid;

            // Orientation - rotate Z?axis to align with the span
            if (diff != Vector3.zero)
                transform.rotation = Quaternion.FromToRotation(Vector3.forward, diff.normalized);

            // Scale collider
            col.size = new Vector2(radius, len);
        }

        private void OnTriggerEnter(Collider other)
        {
            // Ignore rope parts touching themselves
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