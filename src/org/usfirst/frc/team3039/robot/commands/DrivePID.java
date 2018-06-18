package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DrivePID extends Command {
	public double distance;
    public DrivePID(double distance) {
    	requires(Robot.drivetrain);
    	this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.setEncoder();
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.resetNavX();
    	Robot.drivetrain.motorSafety(false);
    	Robot.drivetrain.unBrake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.getAngle();
    	Robot.drivepid.drive(distance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.drivepid.onTarget());
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivepid.disablePID();
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.brake();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrain.resetEncoder();
    	Robot.drivepid.disablePID();
    	Robot.drivetrain.brake();
    }
}
