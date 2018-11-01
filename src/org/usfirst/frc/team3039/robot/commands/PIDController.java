package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PIDController extends Command {
	public double distance;
	public double output;
	public double position = Robot.drivetrain.getDistance();
    public PIDController(double distance) {
    	requires(Robot.drivetrain);
    	requires(Robot.pidcontroller);
    	this.distance = distance;
    	Robot.pidcontroller.setpoint = this.distance;
    	this.output = Robot.pidcontroller.power;
    }

protected void initialize() {
	Robot.drivetrain.setEncoder();
	Robot.drivetrain.resetEncoder();
	Robot.drivetrain.resetNavX();
	Robot.drivetrain.motorSafety(false);
	Robot.drivetrain.unBrake();
}

protected void execute() {
//	Robot.drivetrain.getAngle();
//	Robot.pidcontroller.PID(distance);
//	Robot.drivetrain.driveStraight(output);
	
	System.out.println("Target: " + distance);
	System.out.println("Output: " + output);
	System.out.println("Position: " + position);
}

// Make this return true when this Command no longer needs to run execute()
protected boolean isFinished() {
    return (position + 5 == distance || position - 5 == distance);
}

// Called once after isFinished returns true
protected void end() {
	Robot.drivepid.disablePID();
	Robot.drivetrain.resetEncoder();
	Robot.drivetrain.brake();
	Robot.pidcontroller.resetController();
	System.out.println("Stop");
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
protected void interrupted() {
	Robot.drivetrain.resetEncoder();
	Robot.drivepid.disablePID();
	Robot.drivetrain.brake();
	Robot.pidcontroller.resetController();
	System.out.println("Stop");
	

}
}