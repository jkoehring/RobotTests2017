package org.usfirst.frc.team1165.util;

/**
 * A class for computing sample rates based upon a rolling average.
 */
public class SampleRate
{
	private Rolling rolling;
	private long startTime;
	private int rollingWindowSize;
	
	/**
	 * Construct an instance using a default sample window size.
	 */
	public SampleRate()
	{
		this(100);
	}
	
	/**
	 * Construct an instance using a given sample window size.
	 */
	public SampleRate(int rollingWindowSize)
	{
		this.rollingWindowSize = rollingWindowSize;
	}
	
	/**
	 * Adds a sample.
	 */
	public void addSample()
	{
		synchronized(this)
		{
			long now = System.nanoTime();
			rolling.add(now - startTime);
			startTime = now;
		}
	}
	
	/**
	 * Starts computing sample rates now.
	 */
	public void start()
	{
		synchronized(this)
		{
			startTime = System.nanoTime();
			rolling = new Rolling(rollingWindowSize);
		}
	}
	
	/**
	 * Returns sample rate in samples per second (Hz).
	 */
	public double getSampleRate()
	{
		synchronized(this)
		{
			return 1e9 / rolling.getAverage();
		}
	}
}
