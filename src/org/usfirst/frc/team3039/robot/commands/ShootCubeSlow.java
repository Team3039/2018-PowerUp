package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootCubeSlow extends Command {

    public ShootCubeSlow() {
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.shootCubeSlow();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.stopCube();
    	Robot.intake.stopIntakeNoBrake();
    	Robot.intake.cubeVaultFalse();
    	Robot.intake.cubeAngleFalse();
    	Robot.intake.cubeTurnFalse();
    	Robot.intake.cubeScored();
    	Robot.lights.cubeOut();
    	Robot.intake.cubeAuto = false;

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intake.stopCube();
    	Robot.intake.stopIntakeNoBrake();
    	Robot.intake.cubeVaultFalse();
    	Robot.intake.cubeAngleFalse();
    	Robot.intake.cubeTurnFalse();
    	Robot.intake.cubeScored();
    	Robot.lights.cubeOut();
    	Robot.intake.cubeAuto = false;
    }
}
