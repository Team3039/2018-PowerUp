package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorIntakePID extends Command {

    public SetElevatorIntakePID() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.torqueMode();
    	if(Robot.intake.intakeUp == false) {
        	//Robot.elevatorpid.setElevatorPosition(10);
        	//Robot.elevator.lowerElevator();
    		System.out.println("Going Down");
        	
    	}
    	else {
    		Robot.elevator.stopElevator();
    	}

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//      return Robot.elevatorpid.onTarget() == true;
  	return (Robot.elevator.getMin() == false || Robot.elevator.stop == true);
  	}

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setNotRisen();
    	Robot.elevator.torqueMode();
    	Robot.intake.cubeAngleFalse();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setNotRisen();
    	Robot.elevator.torqueMode();
    	Robot.intake.cubeAngleFalse();
    }
}
