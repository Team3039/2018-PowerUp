package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCurve extends Command {
	double power;
	double angle;
	boolean rotation;
    public DriveCurve(double power, double angle) {
    	requires(Robot.drivetrain);
    	this.power = power;
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.motorSafety(false);
    	Robot.drivetrain.setEncoder();
    	Robot.drivetrain.resetNavX();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println("Encoder : " + Robot.drivetrain.getDistance());
    	//System.out.println("Gyro : " + Robot.drivetrain.getAngle());
    	Robot.drivetrain.getAngle();
    	Robot.drivetrain.driveCurve(power, angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.drivetrain.getAngle()) >= Math.abs(angle));
    }

    // Called once after isFinished returns true
    protected void end() {
    	if(angle > 0) {
        	Robot.drivetrain.brakeTurn(power);
    	}
    	else {
    		Robot.drivetrain.brakeTurn(-power);
    	}
    	Robot.drivetrain.resetNavX();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	if(angle > 0) {
        	Robot.drivetrain.brakeTurn(power);
    	}
    	else {
    		Robot.drivetrain.brakeTurn(-power);
    	}
    	Robot.drivetrain.resetNavX();
    }
}
