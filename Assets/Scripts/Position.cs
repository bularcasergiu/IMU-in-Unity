
using System;
namespace AssemblyCSharp
{
	public class Position
	{
		public float[][] getPosition(float[][] matGyr,float[][] matAcc) {
			//float[] matOut={0,0,0};
			float seamplePeriod = 1 / 256f;
			float[,][] R = new float[3, 3][];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++)
				{
					R[i, j] = new float[matGyr.Length];
					for (int k = 0; k < matGyr.Length; k++)
					{
						R[i, j][k] = 0f;
					}
				} 
			}
			
			MahonyAHRS ahrs = new MahonyAHRS(1/256f);
			for (int i = 0; i < matGyr.Length; i++) {
				float imp = (float)3.14 / 180; 
				//gyro in grade
				float gx = (matGyr[i][0] * imp);
				float gy = matGyr[i][1] * imp;
				float gz = matGyr[i][2] * imp;
				
				float ax = matAcc[i][0];
				float ay = matAcc[i][1];
				float az = matAcc[i][2];
				
				ahrs.Update(gx, gy, gz, ax, ay, az);
				
				float[] euler=new float[]{0,0,0};
				euler=ahrs.QuaternToEuler();
				quaternion2rptMat quatern2rot = new quaternion2rptMat();
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++)
					{
						ahrs.Update(gx, gy, gz, ax, ay, az);
						
						R[j, k][i] = quatern2rot.R(ahrs.Quaternion, j, k, i, matGyr.Length);
					}
				}
			}
			
			// Calculate 'tilt-compensated' accelerometer
			float[][] tcAcc= new float[matAcc.Length][];
			for (int i = 0; i < matAcc.Length; i++)
			{
				tcAcc[i] = new float[matAcc.Length];
				for (int j = 0; j < 3; j++)
				{
					tcAcc[i][j] = 0f;
				}
			}
			
			for (int i = 0; i < matAcc.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					for (int k = 0; k < 3; k++)
					{
						tcAcc[i][j] = tcAcc[i][j] + (R[j, k][i] * matAcc[i][j]);
						
					}
					
				}
			}
			
			float[][] linAcc = new float[matAcc.Length][];
			for (int i = 0; i < matAcc.Length; i++)
			{
				linAcc[i] = new float[matAcc.Length];
				for (int j = 0; j < 3; j++)
				{
					linAcc[i][j] = 0f;
				}
			}
			
			float[][] zeros = new float[matAcc.Length][];
			for (int i = 0; i < matAcc.Length; i++)
			{
				zeros[i] = new float[matAcc.Length];
				for (int j = 0; j < 2; j++)
				{
					zeros[i][j] = 0f;
				}
				for (int j = 2; j < 3; j++)
				{
					zeros[i][j] = 1f;
				}
			}
			
			for (int i = 0; i < matAcc.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					linAcc[i][j] = (tcAcc[i][j]-zeros[i][j])*9.81f;
				}
			}
			
			// %% Calculate linear velocity (integrate acceleartion)
			float[][] linVel = new float[linAcc.Length][];
			for (int i = 0; i < linAcc.Length; i++)
			{
				linVel[i] = new float[linAcc.Length];
				for (int j = 0; j < 3; j++)
				{
					linVel[i][j] = 0f;
				}
			}
			
			for (int i = 1; i < linAcc.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					linVel[i][j] = linVel[i-1][j]+linAcc[i][j]*seamplePeriod;
				}
			}
			
			// %% High-pass filter linear velocity to remove drift
			float[][] linVelHP = new float[linVel.Length][];
			for (int i = 0; i < linVel.Length; i++)
			{
				linVelHP[i] = new float[linVel.Length];
				for (int j = 0; j < 3; j++)
				{
					linVelHP[i][j] = 0f;
				}
			}
			
			butterworth butt = new butterworth();
			butt.FilterButterworth(0.1f,1,2,1/256f);
			for (int i = 0; i < linVel.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					butt.Update(linVel[i][j]);
					linVelHP[i][j] = butt.Value; 
				}
			}
			
			//%% Calculate linear position (integrate velocity)
			float[][] linPos = new float[linVelHP.Length][];
			for (int i = 0; i < linVelHP.Length; i++)
			{
				linPos[i] = new float[linVelHP.Length];
				for (int j = 0; j < 3; j++)
				{
					linPos[i][j] = 0f;
				}
			}
			
			for (int i = 1; i < linVelHP.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					linPos[i][j] = linPos[i - 1][j] + linVelHP[i][j] * seamplePeriod;
				}
			}
			
			//%% High-pass filter linear position to remove drift
			float[][] linPosHP = new float[linPos.Length][];
			for (int i = 0; i < linPos.Length; i++)
			{
				linPosHP[i] = new float[linPos.Length];
				for (int j = 0; j < 3; j++)
				{
					linPosHP[i][j] = 0f;
				}
			}
			
			butt.FilterButterworth(0.1f, 1, 2, 1 / 256f);
			for (int i = 0; i < linPos.Length; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					butt.Update(linPos[i][j]);
					linPosHP[i][j] = butt.Value;
				}
			}
			return linPosHP;
		}
	}
}

