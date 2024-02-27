using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class imu : MonoBehaviour
{
    string data;
    public float w;
    public float x;
    public float y;
    public float z;
    public GameObject Cube;
    SerialPort mySerialPort = new SerialPort("COM5", 115200);
    // Start is called before the first frame update
    void Start()
    {
        mySerialPort.Open();
    }

    // Update is called once per frame
    void Update()
    {
        data = mySerialPort.ReadLine();
        //Debug.Log(data);
        string[] dataParts = data.Split(new string[] { "w:", " x:", " y:", " z:" }, System.StringSplitOptions.RemoveEmptyEntries);

        // Parse the parts into floats
        if (dataParts.Length == 4)
        {
            w = float.Parse(dataParts[0]);
            x = float.Parse(dataParts[1]);
            y = float.Parse(dataParts[2]);
            z = float.Parse(dataParts[3]);
            
            //ApplyRotation(0, 0, roll);
            ApplyQuaternion(w,-y,z,-x);
        }

    }

    void ApplyRotation(float pitch, float yaw, float roll)
    {
        // Create a Vector3 with the Euler angles in degrees
        Vector3 eulerRotation = new Vector3(pitch, -yaw, -roll);

        // Apply the rotation to the target object
        Cube.transform.eulerAngles = eulerRotation;
    }

    void ApplyQuaternion(float w, float x, float y, float z){
        Quaternion rotation = new Quaternion(x, y, z, w);
        Cube.transform.rotation = rotation;
    }
}
