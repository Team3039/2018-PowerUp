package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoPathfinder extends Command {

    public AutoPathfinder() {
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.drivetrain.leftEnc.reset();
		Robot.drivetrain.rightEnc.reset();
		Robot.drivetrain.navX.reset();
    	Robot.drivetrain.setupPF();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.runPF();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.frontleftMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.frontrightMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.rearleftMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.rearrightMotor.set(ControlMode.PercentOutput, 0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrain.frontleftMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.frontrightMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.rearleftMotor.set(ControlMode.PercentOutput, 0);
    	Robot.drivetrain.rearrightMotor.set(ControlMode.PercentOutput, 0);
    }
}
