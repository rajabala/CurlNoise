using UnityEngine;
using System.Collections;

// Move gameobject around
public class RandomMotion : MonoBehaviour {
    public Vector3 moveDirection = new Vector3(-1f, 0f, 0f);
    public float moveSpeed = 0.6f;
    public float switchDirectionTimer = 2f;

    float startTime;
    bool bReverse = false;

	// Use this for initialization
	void Start () {
        startTime = Time.time;
	}
	
	// Update is called once per frame
	void Update () {
        float reverseDir = 1f;

        if (Time.time - startTime >= switchDirectionTimer)
        {
            startTime = Time.time;
            bReverse = !bReverse;
        }

        Vector3 translate = moveDirection * moveSpeed * Time.deltaTime;

        if (bReverse)
            translate *= -1f;

        transform.Translate(translate);
    }
}
