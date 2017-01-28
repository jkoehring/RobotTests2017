package org.usfirst.frc.team1165.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team1165.robot.subsystems.ReportableSubsystem;

/**
 * A Command that reports information from a ReportableSubsystem.
 */
public class Reporter extends Command
{
	private ReportableSubsystem reportable;
	
	public Reporter(ReportableSubsystem reportable)
	{
		// Use requires() here to declare subsystem dependencies
		requires(reportable);
		this.reportable = reportable;
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
		reportable.report();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished()
	{
		return false;
	}

	// Called once after isFinished returns true
	protected void end()
	{
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted()
	{
	}
}
