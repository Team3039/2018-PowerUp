package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoIntakePID extends Command {

    public AutoIntakePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.cubeAuto == true){
    		Robot.intakepid.setIntakePosition(285);
    		//System.out.println("Cube Up");
    		Robot.intake.intakeDown();
    	}

    	else{
    		Robot.intakepid.setIntakePosition(45);
    		//System.out.println("Neither");
    		Robot.intake.intakeDown();
    	}
   }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
      	Robot.intake.intakeUp = false;
    	Robot.intake.intakeDown = true;

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      	Robot.intake.intakeUp = false;
    	Robot.intake.intakeDown = true;

    }
}
