using UnityEngine;
using System.Collections;

public class FlyCamera : MonoBehaviour 
{
    public float forwardSpeed = 5.0f;
    public float upSpeed = 4.0f;
    public float strafeSpeed = 4.0f;
    public float xSpeed = 100.0f;
    public float ySpeed = 50.0f;
    public float yMinLimit = -20;
    public float yMaxLimit = 80;

    private float x = 0.0f;
    private float y = 0.0f;
    private bool inFocus = false;

    private void Start () 
    {
        var angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        // Make the rigid body not change rotation
        if (GetComponent<Rigidbody>() != null)
            GetComponent<Rigidbody>().freezeRotation = true;
    }

    private void LateUpdate () 
    {
        if (Input.GetKeyDown(KeyCode.Mouse1))
        {
            inFocus = !inFocus;
        }

        if( !inFocus )
            return;

        x += Input.GetAxis("Mouse X") * xSpeed * 0.02f;
        y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;

        y = ClampAngle(y, yMinLimit, yMaxLimit);

        float fAxis = Time.deltaTime * forwardSpeed * (Input.GetKey(KeyCode.W)    ? 1 : (Input.GetKey(KeyCode.S) ? -1 : 0));
        float uAxis = Time.deltaTime * upSpeed * (Input.GetKey(KeyCode.E) ? 1 : (Input.GetKey(KeyCode.Q) ? -1 : 0));
        float sAxis = Time.deltaTime * strafeSpeed * (Input.GetKey(KeyCode.D) ? 1 : (Input.GetKey(KeyCode.A) ? -1 : 0));

        transform.rotation = Quaternion.Euler(y, x, 0);
        transform.position += transform.forward * fAxis + transform.right * sAxis + transform.up * uAxis;
    }

    static float ClampAngle (float angle, float min, float max) 
    {
        if (angle < -360)
            angle += 360;
        if (angle > 360)
            angle -= 360;
        return Mathf.Clamp(angle, min, max);
    }
}
