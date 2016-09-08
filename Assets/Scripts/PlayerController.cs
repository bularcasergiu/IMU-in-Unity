using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.IO.Ports;
using System;
public class MadgwickAHRS
{
	/// <summary>
	/// Gets or sets the sample period.
	/// </summary>
	public float SamplePeriod { get; set; }
	
	/// <summary>
	/// Gets or sets the algorithm gain beta.
	/// </summary>
	public float Beta { get; set; }
	
	/// <summary>
	/// Gets or sets the Quaternion output.
	/// </summary>
	public float[] Quaternion { get; set; }
	
	/// <summary>
	/// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
	/// </summary>
	/// <param name="samplePeriod">
	/// Sample period.
	/// </param>
	public MadgwickAHRS(float samplePeriod)
		: this(samplePeriod, 1f)
	{
	}
	
	/// <summary>
	/// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
	/// </summary>
	/// <param name="samplePeriod">
	/// Sample period.
	/// </param>
	/// <param name="beta">
	/// Algorithm gain beta.
	/// </param>
	public MadgwickAHRS(float samplePeriod, float beta)
	{
		SamplePeriod = samplePeriod;
		Beta = beta;
		Quaternion = new float[] { 1f, 0f, 0f, 0f };
	}
	
	/// <summary>
	/// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
	/// </summary>
	/// <param name="gx">
	/// Gyroscope x axis measurement in radians/s.
	/// </param>
	/// <param name="gy">
	/// Gyroscope y axis measurement in radians/s.
	/// </param>
	/// <param name="gz">
	/// Gyroscope z axis measurement in radians/s.
	/// </param>
	/// <param name="ax">
	/// Accelerometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="ay">
	/// Accelerometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="az">
	/// Accelerometer z axis measurement in any calibrated units.
	/// </param>
	/// <param name="mx">
	/// Magnetometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="my">
	/// Magnetometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="mz">
	/// Magnetometer z axis measurement in any calibrated units.
	/// </param>
	/// <remarks>
	/// Optimised for minimal arithmetic.
	/// Total ±: 160
	/// Total *: 172
	/// Total /: 5
	/// Total sqrt: 5
	/// </remarks> 
	public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
	{
		float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;
		
		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2f * q1;
		float _2q2 = 2f * q2;
		float _2q3 = 2f * q3;
		float _2q4 = 2f * q4;
		float _2q1q3 = 2f * q1 * q3;
		float _2q3q4 = 2f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;
		
		// Normalise accelerometer measurement
		norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;
		
		// Normalise magnetometer measurement
		norm = (float)Math.Sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;
		
		// Reference direction of Earth's magnetic field
		_2q1mx = 2f * q1 * mx;
		_2q1my = 2f * q1 * my;
		_2q1mz = 2f * q1 * mz;
		_2q2mx = 2f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = (float)Math.Sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2f * _2bx;
		_4bz = 2f * _2bz;
		
		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;
		
		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;
		
		// Integrate to yield quaternion
		q1 += qDot1 * SamplePeriod;
		q2 += qDot2 * SamplePeriod;
		q3 += qDot3 * SamplePeriod;
		q4 += qDot4 * SamplePeriod;
		norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		Quaternion[0] = q1 * norm;
		Quaternion[1] = q2 * norm;
		Quaternion[2] = q3 * norm;
		Quaternion[3] = q4 * norm;
	}
	
	/// <summary>
	/// Algorithm IMU update method. Requires only gyroscope and accelerometer data.
	/// </summary>
	/// <param name="gx">
	/// Gyroscope x axis measurement in radians/s.
	/// </param>
	/// <param name="gy">
	/// Gyroscope y axis measurement in radians/s.
	/// </param>
	/// <param name="gz">
	/// Gyroscope z axis measurement in radians/s.
	/// </param>
	/// <param name="ax">
	/// Accelerometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="ay">
	/// Accelerometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="az">
	/// Accelerometer z axis measurement in any calibrated units.
	/// </param>
	/// <remarks>
	/// Optimised for minimal arithmetic.
	/// Total ±: 45
	/// Total *: 85
	/// Total /: 3
	/// Total sqrt: 3
	/// </remarks>
	public void Update(float gx, float gy, float gz, float ax, float ay, float az)
	{
		float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
		float norm;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;
		
		// Auxiliary variables to avoid repeated arithmetic
		float _2q1 = 2f * q1;
		float _2q2 = 2f * q2;
		float _2q3 = 2f * q3;
		float _2q4 = 2f * q4;
		float _4q1 = 4f * q1;
		float _4q2 = 4f * q2;
		float _4q3 = 4f * q3;
		float _8q2 = 8f * q2;
		float _8q3 = 8f * q3;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;
		float q4q4 = q4 * q4;
		
		// Normalise accelerometer measurement
		norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;
		
		// Gradient decent algorithm corrective step
		s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
		s2 = _4q2 * q4q4 - _2q4 * ax + 4f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		s3 = 4f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		s4 = 4f * q2q2 * q4 - _2q2 * ax + 4f * q3q3 * q4 - _2q3 * ay;
		norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;
		
		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;
		
		// Integrate to yield quaternion
		q1 += qDot1 ;//* SamplePeriod;
		q2 += qDot2;//* SamplePeriod;
		q3 += qDot3;//* SamplePeriod;
		q4 += qDot4;//* SamplePeriod;
		norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		Quaternion[0] = q1; //* norm;
		Quaternion [1] = q2;//* norm;
		Quaternion [2] = q3;//* norm;
		Quaternion [3] = q4; //* norm;
	}
	public float[] QuaternToEuler(){
		float[] euler=new float[3]{0,0,0};
		//yaw
		euler [0] = Mathf.Atan2 (2 * (Quaternion [0] * Quaternion [3] + Quaternion [1] * Quaternion [2]), 1 - 2 * (Quaternion [2]*Quaternion [2]+ Quaternion [3]*Quaternion [3]));
		//roll
		euler [1] = Mathf.Atan2 (2 * (Quaternion [0] * Quaternion [1] + Quaternion [2] * Quaternion [3]), 1 - 2 * (Quaternion [1]*Quaternion [1]+ Quaternion [2]*Quaternion [2]));
		//pitch
		euler [2]=Mathf.Asin(2 * (Quaternion [0] * Quaternion [2] - Quaternion [3] * Quaternion [1]));
		return euler;
	}

}


public class Euler_angles
{
	public float[] getEuler(float[][] matGyr,float[][] matAcc) {
		//float[] matOut={0,0,0};
		float[] euler = new float[]{0,0,0};
		//float seamplePeriod = 1 / 256f;
		float[,][] R = new float[3, 3][];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				R [i, j] = new float[matGyr.Length];
				for (int k = 0; k < matGyr.Length; k++) {
					R [i, j] [k] = 0f;
				}
			} 
		}
		
		MadgwickAHRS ahrs = new MadgwickAHRS (1 / 256f);
		for (int i = 0; i < matGyr.Length; i++) {
			float imp = (float)3.14 / 180; 
			//gyro in grade
			float gx = (matGyr [i] [0] * imp);
			float gy = matGyr [i] [1] * imp;
			float gz = matGyr [i] [2] * imp;
			
			float ax = matAcc [i] [0];
			float ay = matAcc [i] [1];
			float az = matAcc [i] [2];
			
			ahrs.Update (gx, gy, gz, ax, ay, az);
			
			
			euler = ahrs.QuaternToEuler ();
			
			
		}
		return euler;
	}
}

public class PlayerController : MonoBehaviour {

	SerialPort stream = new SerialPort("COM3", 115200); //Set the port (com4) and the baud rate (9600, is standard on most devices)
	float[] lastRot = {0,0,0}; //Need the last rotation to tell how far to spin the camera
	//public static float[] accArray={0,0,0};
	Vector3 rot = Vector3.zero;
	Vector3 rot1 = Vector3.zero;
	Vector3 offset;
	static int count;

	void Start () {
		stream.Open();
		//transform.position = new Vector3 (0.0f,0.5f,0.0f);
		count = 0;
	}

	public static float[][] initMat(){
		float[][] mat= new float[2][];
		for (int i = 0; i < 2; i++)
		{
			mat[i] = new float[3];
			for (int j = 0; j < 3; j++)
			{
				mat[i][j] = 0.0f;
			}
		}
		return mat;
	}
	
	public static float[][] matAcc = initMat();
	public static float[][] matGyr = initMat();

	public float[] getAccArry(float ax,float ay,float az){
		float[] accArray={0,0,0};
		accArray [0] = ax;
		accArray [1] = ay;
		accArray [2] = az;
		return accArray;
	}

	public float[] getGyrArry(float gx,float gy,float gz){
		float[] gyrArray={0,0,0};
		gyrArray [0] = gx;
		gyrArray[1] = gy;
		gyrArray [2] = gz;
		return gyrArray;
	}
	
	public static float[][] finalizeMatAcc(float[] accArray){
		for (int j=0; j<3; j++) {
			if (count > 0) {
				matAcc[0][j]=matAcc[1][j];
			}
			matAcc[1][j]=accArray[j];
		}
		return matAcc;
	}

	public static float[][] finalizeMatGyr(float[] gyrArray){
		for (int j=0; j<3; j++) {
			if (count > 0) {
				matGyr[0][j]=matGyr[1][j];
			}
			matGyr[1][j]=gyrArray[j];
		}
		return matGyr;
	}

	static float deg2rad(float degrees)
	{
		return (float)(Math.PI / 180) * degrees;
	}

	// Update is called once per frame
	void Update () {

	}
	void FixedUpdate(){
		float[] arrayAcc = {0,0,0};
		float[] arrayGyr = {0,0,0};
		float[] euler = {0.0f,0.0f,0.0f};
		string value = stream.ReadLine(); //Read the information
		string[] vec3 = value.Split(','); //My arduino script returns a 3 part value (IE: 12,30,18)
		if(vec3[0] != "" && vec3[1] != "" && vec3[2] != "" )
			//&& vec3[3] != "" && vec3[4] != "" && vec3[5] != "") //Check if all values are recieved
		{ 
			//			getMatGyr(count,float.Parse(vec3[0]),float.Parse(vec3[1]),float.Parse(vec3[2]));
			//arrayAcc=getAccArry(float.Parse(vec3[3]),float.Parse(vec3[4]),float.Parse(vec3[5]));
			//arrayGyr=getGyrArry(deg2rad(float.Parse(vec3[0])),deg2rad(float.Parse(vec3[1])),deg2rad(float.Parse(vec3[2])));
			rot = new Vector3(float.Parse(vec3[0]),float.Parse(vec3[1]),float.Parse(vec3[2]));//gyro
			//rot1 = new Vector3(float.Parse(vec3[3]),float.Parse(vec3[4]),float.Parse(vec3[5]));//acc
		}

		
		//matAcc = finalizeMatAcc (arrayAcc);
		//matGyr = finalizeMatGyr (arrayGyr);
		//Euler_angles eul = new Euler_angles ();
		//euler=eul.getEuler (matGyr, matAcc);

		//EULER TO DEGREES
		float roll_degrees = 0.0000f;
		roll_degrees = -float.Parse(vec3[0])*(180.0f / 3.14f);
		if (roll_degrees < 0) {
			roll_degrees+=360.0f;
		}
		float pitch_degrees = 0.0000f;
		pitch_degrees = float.Parse(vec3[1])*(180.0f / 3.14f);
		if (pitch_degrees < 0) {
			pitch_degrees+=360.0f;
		}

		float yaw_degrees = 0.0000f;
		yaw_degrees = -float.Parse(vec3[2])*(180.0f / 3.14f);
		if (yaw_degrees < 0) {
			yaw_degrees+=360.0f;
		}

		//Vector3 eulV3 = new Vector3 (0,0,yaw_degrees);
		Vector3 eulV3 = new Vector3 (pitch_degrees,yaw_degrees,roll_degrees);
		transform.eulerAngles = eulV3;
		//var rotation=Quaternion.Euler(eulV3);
		count++;
	}

	void OnGUI()
	{
		GUI.Label(new Rect(10,30,300,100), "\t" + rot);
		GUI.Label(new Rect(10,40,400,200), "\t" + rot1);
	}
}

