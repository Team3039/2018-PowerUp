package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {
	double power;
	double distance;
    public DriveStraight(double power, double distance) {
    	requires(Robot.drivetrain);
    	this.power = power;
    	this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.setEncoder();
    	Robot.drivetrain.resetNavX();
    	Robot.drivetrain.motorSafety(false);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.getAngle();
//    	System.out.println("Encoder : " + Robot.drivetrain.getDistance());
    	//System.out.println("Gyro : " + Robot.drivetrain.getAngle());
    	Robot.drivetrain.driveStraight(power);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.drivetrain.getDistance()) >= distance);
    }

    // Called once after isFinished returns true
    protected void end() {
//    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.brake();
    	Robot.drivetrain.brakeDrive(.1);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.brake();
    	Robot.drivetrain.resetEncoder();
    }
}
