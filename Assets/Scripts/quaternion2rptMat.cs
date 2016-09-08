using UnityEngine;
using System.Collections;

public class quaternion2rptMat
{

		public float R(float[] q,int i,int j,int k,int length) {
			float[,][] R = new float[3, 3][];
			for (int ii = 0; ii < 3; ii++)
			{
				for (int jj = 0; jj < 3; jj++)
				{
					R[ii, jj] = new float[length];
					for (int kk = 0; kk < length; kk++)
					{
						R[ii, jj][kk] = 0f;
					}
				}
			}
			R[i, j][k] = (2 * (q[0] * q[0])) - 1 + (2 * (q[1] * q[1]));
			R[i, j][k] = ((2 * q[1]) * (q[2])) + ((q[0]) * (q[3]));
			R[i, j][k] = ((2 * q[1]) * (q[3])) - ((q[0]) * (q[2]));
			R[i, j][k] = ((2 * q[1]) * (q[2])) - ((q[0]) * (q[3]));
			R[i, j][k] = (2 * (q[0] * q[0])) - 1 + (2 * (q[2] * q[2]));
			R[i, j][k] = ((2 * q[2]) * (q[3])) + ((q[0]) * (q[1]));
			R[i, j][k] = ((2 * q[1]) * (q[3])) + ((q[0]) * (q[2]));
			R[i, j][k] = ((2 * q[2]) * (q[3])) - ((q[0]) * (q[1]));
			R[i, j][k] = (2 * (q[0] * q[0])) - 1 + (2 * (q[3] * q[0]));
			
			return R[i,j][k];
	}

}

