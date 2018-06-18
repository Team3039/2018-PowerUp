package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorIntake extends Command {

    public SetElevatorIntake() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    	Robot.elevator.movingScale = false;
    	Robot.elevator.movingSwitch = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.torqueMode();
    	Robot.elevator.movingIntake = true;
    	if(Robot.intake.intakeUp == false) {
//        	Robot.elevatorpid.intakeLevel();
        	Robot.elevator.lowerElevator();
        	Robot.intake.cubeIntakeAngle();

    		//System.out.println("Going Down");
        	
    	}
    	else {
    		Robot.elevator.stopElevator();
    		Robot.intake.cubeIntakeAngle();
    	}

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//      return Robot.elevatorpid.onTarget() == true;
  	return (Robot.elevator.getMin() == false || Robot.elevator.stop == true || Robot.elevator.movingScale == true || Robot.elevator.movingSwitch == true);
  	}

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeAngleFalse();
    	Robot.intake.cubeOut();
    	Robot.elevator.setNotUp();
    	Robot.elevator.setNotRisen();
    	Robot.elevator.movingIntake = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.intake.cubeOut();
    	Robot.intake.cubeAngleFalse();
    	Robot.elevator.setNotRisen();
    	Robot.elevator.setNotUp();
    	Robot.elevator.movingIntake = false;


    }
}
