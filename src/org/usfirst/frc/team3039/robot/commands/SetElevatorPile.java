
package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorPile extends Command {

    public SetElevatorPile() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    	Robot.intake.cubeOut();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.intakeUp == false) {
        	Robot.elevatorpid.setElevatorPosition(550);
//        	if(Robot.elevator.risen) {
//        		//Robot.elevator.lowerElevator();
//        		System.out.print("Going Switch");
//        	}
//        	else {
//        		//Robot.elevator.liftElevator();
//        	}
    	}
    	else {
    		Robot.elevator.stopElevator();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//      return Robot.elevatorpid.onTarget() == true;
  	return (Robot.elevator.getMid() == false || Robot.elevator.stop == true || Robot.elevatorpid.onTarget());
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    }
}
