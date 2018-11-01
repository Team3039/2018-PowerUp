package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightScale extends CommandGroup {

    public AutoRightScale() { 
        String gameInfo = DriverStation.getInstance().getGameSpecificMessage();
        int count = 0;
        while(count < 1000) {
            count++;
            gameInfo = DriverStation.getInstance().getGameSpecificMessage();
        }

       	if(gameInfo.isEmpty()) {
    	    addSequential(new DrivePID(250));
       	}
       	else {
       		if(gameInfo.charAt(1) == 'R') {	//Right Scale is Prioritzed
            	addSequential(new Startup(), .00001);
        	    addParallel(new DrivePID(270));
        	    addSequential(new RunElevatorScaleAuto());
        	    addParallel(new TurnPID(-42.5), .5);
        	    addSequential(new GetCube(), 1);
        	    addSequential(new DriveShortPID(34),.1);
        	    addSequential(new ShootCube(), .2);
        	    addSequential(new AutoResetDriveEncoders(), .0001);
        	    addSequential(new DriveBackward(), .25);
        	    addParallel(new SetElevatorIntakeAuto());
        		addSequential(new TurnPIDLong(-90));

           	}
           	else {
           		if(gameInfo.charAt(0) == 'R') {	//If No Scale then go to Switch
	            	addSequential(new Startup(), .00001);
	                addSequential(new DrivePID(132));
	                addSequential(new RunElevatorSwitchAuto());
	               	addSequential(new CurveLeft(), .7); //TODO Fix the Negative Turning, it is very slow with PID
	                addSequential(new ShootCube(), .2);
           		}
           		else {	//TODO: Only going straight for now because of Turning error
	            	addSequential(new Startup(), .00001);
	                addSequential(new DrivePID(215));
/*	              	addSequential(new AutoResetDriveEncoders(), .025);
	                addSequential(new TurnPID(85)); 
	                addSequential(new AutoResetDriveEncoders(), .025);
	                addSequential(new DrivePID(190));
	                addSequential(new AutoResetDriveEncoders(), .025);
	               	addSequential(new CurveLeft(), .7);
	                addSequential(new RunElevatorScaleAuto());
	                addSequential(new GetCube(), 1);
	        	    addSequential(new DriveShortPID(5),.1);
	        	    addSequential(new ShootCube(), .2);*/
           		}
        	}
       	}
    }
}
