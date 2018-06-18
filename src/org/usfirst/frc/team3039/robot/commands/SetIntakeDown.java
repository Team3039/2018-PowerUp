package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakeDown extends Command {
	
	double targetRev = 650;
	
    public SetIntakeDown() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setNotUp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.cubeIn = false;
    	Robot.intake.climb = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.intakepid.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
//    	Robot.intake.stopIntakeBrake();
    	Robot.intakepid.stopIntake();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	Robot.intake.stopIntakeBrake();
    	Robot.intakepid.stopIntake();
    }
}
