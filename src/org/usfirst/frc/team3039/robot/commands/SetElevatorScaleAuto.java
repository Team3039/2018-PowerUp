package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorScaleAuto extends Command {

    public SetElevatorScaleAuto() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getElevatorEncoder();
    	Robot.elevator.movingElevator();
    	Robot.intake.cubeAuto = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.torqueMode();
    	if(Robot.intake.intakeUp == false) {
        	//Robot.elevatorpid.scaleLevel();
        	Robot.elevator.liftElevatorAuto();
        	Robot.intake.cubeAuto = true;

    	}
    	else {
    		Robot.elevator.stopElevator();
        	Robot.intake.cubeAuto = true;

    	}
    }
   

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Robot.elevator.getMax() == false);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setRisen();
    	Robot.intake.cubeAuto = true;

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.stopElevator();
    	Robot.elevator.setRisen();
    	Robot.intake.cubeAuto = true;

    }
}
