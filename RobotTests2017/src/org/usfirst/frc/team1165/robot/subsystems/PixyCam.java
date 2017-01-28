package org.usfirst.frc.team1165.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team1165.robot.commands.Reporter;
import org.usfirst.frc.team1165.util.SampleRate;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 *
 * Data received from the Pixy Cam iver the serial interface has the following format.
 * All data is received as 16-bit words with the low-order byte sent first.
 * 
 * Each frame starts with 2 sync words, with a single sync word separating the
 * data for each object within a frame.
 * 
 * Bytes    16-bit word    Description
 * ----------------------------------------------------------------
 * 0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
 * 2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
 * 4, 5     y              signature number
 * 6, 7     y              x center of object
 * 8, 9     y              y center of object
 * 10, 11   y              width of object
 * 12, 13   y              height of object
 *
 */
public class PixyCam extends ReportableSubsystem implements Runnable
{
	private SerialPort serialPort;
	
	private SampleRate serialSampleRate;
	
	private byte[] serialData;
	int serialIndex;
	
	private final short syncWordLower = 0x55;
	private final short syncWordUpper = 0xaa;
	private final int syncWord = (syncWordUpper << 8) | syncWordLower;
	
	private final Object frameLock = new Object();
	private List<PixyCamObject> currentFrame;
	
	private int frameErrorCount = 0;
	private int checksumErrorCount = 0;
	
	public PixyCam(SerialPort serialPort)
	{
		this.serialPort = serialPort;
		
		serialSampleRate = new SampleRate();
		serialSampleRate.start();
		new Thread(this).start();
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	/**
	 * Returns the current frame of object data.
	 * @return
	 */
	public List<PixyCamObject> getFrame()
	{
		synchronized(frameLock)
		{
			return currentFrame;
		}
	}

	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		setDefaultCommand(new Reporter(this));
	}
	
	/**
	 * Reads an 8-bit unsigned byte from the serial port.
	 */
	private short readByte()
	{
		while (true)
		{
			// If all previously read bytes have been processed, read more bytes:
			if (serialIndex >= serialData.length)
			{
				serialData = serialPort.read(serialPort.getBytesReceived());
				if (0 == serialData.length)
				{
					continue;
				}
				serialIndex = 0;
				break;
			}
		}
		
		short b = (short)(serialData[serialIndex++] & 0xff);
		System.out.println("Pixy: " + b);
		return b;
	}
	
	/**
	 * Reads a frame of data from the camera.
	 * The two sync words at the end of the frame are read as well.
	 */
	private List<PixyCamObject> readFrame()
	{
		// Start with an empty list:
		List<PixyCamObject> frame = new ArrayList<PixyCamObject>();
		
		int word = readWord();
		
		// Loop through all objects in the block:
		while (true)
		{
			PixyCamObject obj = readObject(word);
			if (null != obj)
			{
				frame.add(obj);
			}
			
			word = readWord();
			if (syncWord == word)
			{
				word = readWord();
				if (syncWord == word)
				{
					return frame;
				}
			}
			else
			{
				// This is unexpected. There should be a sync word after the object data.
				System.out.println("Pixy Cam object data not followed by sync word");
				frameErrorCount++;
				readFrameStart();
				return frame;
			}
		}
	}
	
	/**
	 * Reads up to and including the two sync words that indicate a start of frame data.
	 */
	private void readFrameStart()
	{
		// Find the start of a block of data:
		while (true)
		{
			readSyncWord();
			if (readWord() == syncWord)
			{
				return;
			}
		}
	}
	
	/**
	 * Reads an object, exclusive of any sync words.
	 * @return null is returned if an object was not successfully read
	 */
	private PixyCamObject readObject(int firstWord)
	{
		int checksum = firstWord;
		int signatureNumber = readWord();
		int xCenter = readWord();
		int yCenter = readWord();
		int width = readWord();
		int height = readWord();
		if (checksum == ((signatureNumber + xCenter + yCenter + width + height) & 0xffff))
		{
			return new PixyCamObject(signatureNumber, xCenter, yCenter, width, height);
		}
		else
		{
			checksumErrorCount++;
			return null;
		}
	}
	
	/**
	 * Reads the next sync word.
	 */
	private void readSyncWord()
	{
		short b = readByte();
		while (true)
		{
			while (b != syncWordLower)
			{
				b = readByte();
			}
			b = readByte();
			if (b == syncWordUpper)
			{
				System.out.println("Found Pixy sync word");
				return;
			}
		}
	}
	
	/**
	 * Reads a 16-bit unsigned word from the serial port.
	 */
	private int readWord()
	{
		return (int)(readByte() | (readByte() << 8));
	}
	
	@Override
	public void report()
	{
		SmartDashboard.putNumber("Pixy Sample Rate", serialSampleRate.getSampleRate());
		SmartDashboard.putNumber("Pixy Checksum Errors", checksumErrorCount);
		SmartDashboard.putNumber("Pixy Frame Errors", frameErrorCount);
		
		List<PixyCamObject> frame = getFrame();
		
		for (int i = 0; i < frame.size(); i++)
		{
			PixyCamObject obj = frame.get(i);
			SmartDashboard.putNumber("Pixy Object" + (i + 1) + " sigNum", obj.signatureNumber);
			SmartDashboard.putNumber("Pixy Object" + (i + 1) + " xCenter", obj.xCenter);
			SmartDashboard.putNumber("Pixy Object" + (i + 1) + " yCenter", obj.yCenter);
			SmartDashboard.putNumber("Pixy Object" + (i + 1) + " width", obj.width);
			SmartDashboard.putNumber("Pixy Object" + (i + 1) + " height", obj.height);
		}
	}	

	@Override
	public void run()
	{
		// Reset serial port to empty buffers:
		serialPort.reset();
		
		// Start with an empty data buffer:
		serialData = new byte[0];
		serialIndex = 0;
		
		checksumErrorCount = 0;
		
		// Initialize frame that is returned:
		currentFrame = new ArrayList<PixyCamObject>();
		
		readFrameStart();
		
		// Every time through this loop we have already read the block start words
		// and are ready to read a block's worth of data
		while (true)
		{
			List<PixyCamObject> frame = readFrame();
			serialSampleRate.addSample();
			
			synchronized(frameLock)
			{
				currentFrame = frame;
			}
		}
	}

	public class PixyCamObject
	{
		public int signatureNumber;
		public int xCenter;
		public int yCenter;
		public int width;
		public int height;
		
		public PixyCamObject(int signatureNumber, int xCenter, int yCenter, int width, int height)
		{
			this.signatureNumber = signatureNumber;
			this.xCenter = xCenter;
			this.yCenter = yCenter;
			this.width = width;
			this.height = height;
		}
	}
}

