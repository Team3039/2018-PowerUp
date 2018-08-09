/*package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

*//**
 *
 *//*
public class SetIntakePosition extends Command {

    public SetIntakePosition() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.cubeIn == true && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false)){
    		Robot.intakepid.setIntakePosition(30);
    		System.out.println("Cube Up");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false)){
    		Robot.intakepid.setIntakePosition(315);
    		System.out.println("No Cube Straight");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == true) && (Robot.intake.cubeTurn == false)){
    		Robot.intakepid.setIntakePosition(30);
    		System.out.println("Cube Out Up");
    		Robot.intake.intakeDown();
    		
    	}
    	
    	else if(Robot.intake.cubeIn == true && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == true) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false)){
    		Robot.intakepid.setIntakePosition(290);
    		System.out.println("Cube Vault");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == true && (Robot.intake.cubeAngle == true) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false)){
    		Robot.intakepid.setIntakePosition(140);
    		System.out.println("Angled Shot");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == true)){
    		Robot.intakepid.setIntakePosition(230);
    		System.out.println("Flip Cube");
    		Robot.intake.intakeDown();
    	}
    	else{
    		Robot.intakepid.setIntakePosition(300);
    		System.out.println("Neither");
    		Robot.intake.intakeDown();
    	}
   }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intakepid.stopIntake();
    	Robot.intakepid.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intakepid.stopIntake();
    	Robot.intakepid.disable();
    }
}
*/

package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakePosition extends Command {

    public SetIntakePosition() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.intake.cubeIn == true && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false) && (Robot.intake.cubePush == false)){
    		Robot.intakepid.setIntakePosition(270);
    		//System.out.println("Cube In Straight");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeAngle == false) && (Robot.intake.cubeVault == false) && (Robot.intake.cubeScored == false) && (Robot.intake.cubeTurn == false) && (Robot.intake.cubePush == false)){
    		Robot.intakepid.setIntakePosition(295);
    		//System.out.println("No Cube Straight");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeScored == true)){
    		Robot.intakepid.setIntakePosition(45);
    		//System.out.println("Cube Out Up");
    		Robot.intake.intakeDown();
    		
    	}
    	
    	else if (Robot.intake.cubeIn == true && (Robot.intake.cubeAngle == true)){
    		Robot.intakepid.setIntakePosition(140);
    		//System.out.println("Angled Shot");
    		Robot.intake.intakeDown();
    	}
    	
    	else if (Robot.intake.cubeIn == false && (Robot.intake.cubeTurn == true)){
    		Robot.intakepid.setIntakePosition(230);
    		//System.out.println("Flip Cube");
    		Robot.intake.intakeDown();
    	}
    	
    	else if(Robot.intake.cubeIn == true && (Robot.intake.cubePush == true)) {
    		Robot.intakepid.setIntakePosition(190);
    		//System.out.println("Slight Angle");
    		Robot.intake.intakeDown();
    	}
    	
    	else if(Robot.intake.cubeIn  == false && (Robot.intake.climb == true)) {
    		Robot.intakepid.setIntakePosition(585);
    		//System.out.println("Slight Angle");
    		Robot.intake.intakeDown();
    	}
    	else{
    		Robot.intakepid.setIntakePosition(300);
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
    	Robot.intakepid.stopIntake();
    	Robot.intakepid.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intakepid.stopIntake();
    	Robot.intakepid.disable();
    }
}
