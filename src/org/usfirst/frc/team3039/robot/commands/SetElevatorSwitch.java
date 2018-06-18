package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorSwitch extends Command {

    public SetElevatorSwitch () {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    	Robot.elevator.movingScale = false;
    	Robot.elevator.movingIntake = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.intakeUp == false) {
    		Robot.elevator.movingSwitch = true;
//        	Robot.elevatorpid.switchLevel();
        	if(Robot.elevator.risen) {
            	Robot.elevator.lowerElevator();
            	Robot.intake.cubeOut();

        	}
        	
        	else {
        		Robot.elevator.liftElevator();
            	Robot.intake.cubeOut();

        	}
    	}
    	else {
    		Robot.elevator.stopElevator();
        	Robot.intake.cubeOut();

    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//      return Robot.elevatorpid.onTarget() == true;
  	return (Robot.elevator.getMid() == false || Robot.elevator.stop == true || Robot.elevator.movingIntake == true || Robot.elevator.movingScale == true);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeOut();
    	Robot.elevator.movingSwitch = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeOut();
    	Robot.elevator.movingSwitch = false;

    }
}
