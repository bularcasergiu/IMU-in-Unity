using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.IO.Ports;
using System;

public class butterworth
{
	/// <summary>
	/// rez amount, from sqrt(2) to ~ 0.1
	/// </summary>
	private float resonance;
	
	private  float frequency;
	private  int sampleRate;
	private  PassType passType;
	
	private  float c, a1, a2, a3, b1, b2;
	
	/// <summary>
	/// Array of input values, latest are in front
	/// </summary>
	private float[] inputHistory = new float[2];
	
	/// <summary>
	/// Array of output values, latest are in front
	/// </summary>
	private float[] outputHistory = new float[3];
	
	public int FilterButterworth(float frequency, int sampleRate, int passType, float resonance)
	{
		this.resonance = resonance;
		this.frequency = frequency;
		this.sampleRate = sampleRate;
		
		
		switch (passType)
		{
		case 1://lowpass
			c = 1.0f / (float)Math.Tan(Math.PI * frequency / sampleRate);
			a1 = 1.0f / (1.0f + resonance * c + c * c);
			a2 = 2f * a1;
			a3 = a1;
			b1 = 2.0f * (1.0f - c * c) * a1;
			b2 = (1.0f - resonance * c + c * c) * a1;
			break;
		case 2://highpass
			c = (float)Math.Tan(Math.PI * frequency / sampleRate);
			a1 = 1.0f / (1.0f + resonance * c + c * c);
			a2 = -2f * a1;
			a3 = a1;
			b1 = 2.0f * (c * c - 1.0f) * a1;
			b2 = (1.0f - resonance * c + c * c) * a1;
			break;
		}
		return 0;
	}
	
	public enum PassType
	{
		Highpass,
		Lowpass,
	}
	
	public void Update(float newInput)
	{
		float newOutput = a1 * newInput + a2 * this.inputHistory[0] + a3 * this.inputHistory[1] - b1 * this.outputHistory[0] - b2 * this.outputHistory[1];
		
		this.inputHistory[1] = this.inputHistory[0];
		this.inputHistory[0] = newInput;
		
		this.outputHistory[2] = this.outputHistory[1];
		this.outputHistory[1] = this.outputHistory[0];
		this.outputHistory[0] = newOutput;
	}
	
	public float Value
	{
		get { return this.outputHistory[0]; }
	}
}

