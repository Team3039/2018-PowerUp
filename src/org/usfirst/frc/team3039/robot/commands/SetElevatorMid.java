package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorMid extends Command {

    public SetElevatorMid() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.intakeUp == false) {
        	//Robot.elevatorpid.scaleLevel();
        	Robot.elevator.liftElevatorRampMid();
        	Robot.intake.cubeOut();

    	}
    	else {
    		Robot.elevator.stopElevator();
    		Robot.intake.cubeOut();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeOut();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeOut();
    }
}
