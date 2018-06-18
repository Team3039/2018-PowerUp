package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorScale extends Command {

    public SetElevatorScale() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    	Robot.elevator.movingIntake = false;
    	Robot.elevator.movingSwitch = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.torqueMode();
    	Robot.elevator.movingScale = true;
    	if(Robot.intake.intakeUp == false) {
        	//Robot.elevatorpid.scaleLevel();
        	Robot.elevator.liftElevator();
        	Robot.intake.cubeOut();

    	}
    	else {
    		Robot.elevator.stopElevator();
    		Robot.intake.cubeOut();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Robot.elevator.getMax() == false || Robot.elevator.stop == true || Robot.elevator.movingSwitch == true|| Robot.elevator.movingIntake == true);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setRisen();
    	Robot.intake.cubeOut();
//    	Robot.elevator.movingScale = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setRisen();
    	Robot.intake.cubeOut();
//    	Robot.elevator.movingScale = false;

    }
}
