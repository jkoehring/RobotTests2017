package org.usfirst.frc.team1165.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An abstract definition of a Subsystem that can report information about itself.
 */
public abstract class ReportableSubsystem extends Subsystem
{
	public abstract void report();
}
