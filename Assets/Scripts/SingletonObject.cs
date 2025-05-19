using System.Linq;
using UnityEngine;

public class SingletonObject : MonoBehaviour
{
    void Awake()
    {
        var all = FindObjectsByType<SingletonObject>(FindObjectsSortMode.None);

        var sameName = all.Where(o => o.gameObject.name == gameObject.name);

        if (sameName.Count() > 1)
        {
            Destroy(gameObject);
            return;
        }

        DontDestroyOnLoad(gameObject);
    }
}
