using UnityEngine;
using System.Collections;

public class FirstTwoArmsController : MonoBehaviour {
		public float xRotation = 5.0F;
		void Update() {
				xRotation += Input.GetAxis("Vertical");
				transform.eulerAngles = new Vector3(10, xRotation, 0);
		}
		void Example() {
				print(transform.eulerAngles.x);
				print(transform.eulerAngles.y);
				print(transform.eulerAngles.z);
		}
}
