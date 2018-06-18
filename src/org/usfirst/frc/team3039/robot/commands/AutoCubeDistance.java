package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoCubeDistance extends Command {

    	double power;
        public AutoCubeDistance(double power) {
        	requires(Robot.drivetrain);
        	this.power = power;
        }

        // Called just before this Command runs the first time
        protected void initialize() {
        	Robot.drivetrain.setEncoder();
        	Robot.drivetrain.resetNavX();
        	Robot.drivetrain.motorSafety(false);

        }

        // Called repeatedly when this Command is scheduled to run
        protected void execute() {
        	//Robot.drivetrain.getAngle();
        	//Robot.drivetrain.driveStraight(.4);
    		System.out.println("Distance to Cube B :" + Robot.cubeDistance);

        }
    protected boolean isFinished() {
        return (Robot.drivetrain.getDistance() >= Robot.cubeDistance);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.driveStop();
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.resetNavX();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrain.driveStop();
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.resetNavX();
    }
}
