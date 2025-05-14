using UnityEngine;
using System.Collections.Generic;

public class RopeController : MonoBehaviour
{
    [SerializeField] GameObject topAnchor;
    [SerializeField] GameObject linkPrefab;
    [SerializeField] int links = 15;
    [SerializeField] float linkLength = 0.6f;

    public readonly List<Rigidbody2D> bodies = new();

    void Awake()
    {
        Vector3 dir = Vector3.down;

        GameObject prev = topAnchor;

        for (int i = 0; i < links; i++)
        {
            Vector3 pos = topAnchor.transform.position + dir * linkLength * (i + 1);
            var link = Instantiate(linkPrefab, pos, linkPrefab.transform.rotation, transform);

            var rb = link.GetComponent<Rigidbody2D>();
            var hj = link.GetComponent<HingeJoint2D>();
            link.GetComponent<RopeLink>().index = i;

            hj.connectedBody = prev.GetComponent<Rigidbody2D>();

            bodies.Add(rb);
            prev = link;
        }
    }
}
