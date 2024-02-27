using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class imu : MonoBehaviour
{
    string data;
    public float yaw;
    public float pitch;
    public float roll;
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
        string[] dataParts = data.Split(new string[] { "roll:", " pitch:", " yaw:" }, System.StringSplitOptions.RemoveEmptyEntries);

        // Parse the parts into floats
        if (dataParts.Length == 3)
        {
            roll = float.Parse(dataParts[0]);
            pitch = float.Parse(dataParts[1]);
            yaw = float.Parse(dataParts[2]);
            
            ApplyRotation(0, 0, roll);
        }

    }
    void ApplyRotation(float pitch, float yaw, float roll)
    {
        // Create a Vector3 with the Euler angles in degrees
        Vector3 eulerRotation = new Vector3(pitch, -yaw, -roll);

        // Apply the rotation to the target object
        Cube.transform.eulerAngles = eulerRotation;
    }
}
