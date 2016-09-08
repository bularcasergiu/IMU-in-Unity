using UnityEngine;
using System.Collections;

public class LowerArmControll : MonoBehaviour {

	// Use this for initialization
	void Start () {
	
	}

		public Transform A, B;
		private float startDistance = 0;
		private bool rotateAtoB = false;

		public void Update() {
//				if (startDistance == 0) {
//						startDistance = Vector3.Distance(A.position, B.position);
//						rotateAtoB = true;
//				}
//
//				if (rotateAtoB == true) {
//						float delta = 1 - (Vector3.Distance(A.position, B.position) / startDisatnce);
//						A.rotation = Quaternion.Slerp(A.rotation, B.rotation, delta);
//
//						if (Mathf.Approximately(delta, 1)) // stop rotating when A is rotated approximately equal to B's orientation.
//								rotateAtoB = false;
//				}
		}

}
