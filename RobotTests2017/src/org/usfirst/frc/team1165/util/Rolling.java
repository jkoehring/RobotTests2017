package org.usfirst.frc.team1165.util;

/**
 * A class for computing an average over a rolling window of values.
 */
public class Rolling
{
	private int size;
	private double total = 0d;
	private int index = 0;
	private double samples[];

	/**
	 * Construct an instance given the sample size to average over.
	 */
	public Rolling(int size)
	{
		this.size = size;
		samples = new double[size];
		for (int i = 0; i < size; i++)
			samples[i] = 0d;
	}

	/**
	 * Add a new value to the window, updating current total appropriately.
	 */
	public void add(double x)
	{
		total -= samples[index];
		samples[index] = x;
		total += x;
		if (++index == size)
			index = 0; // cheaper than modulus
	}

	/**
	 * Get current
	 * @return
	 */
	public double getAverage()
	{
		return total / size;
	}
}
