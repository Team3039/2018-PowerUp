package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftScale extends CommandGroup {

    public AutoLeftScale() {    
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
       		if(gameInfo.charAt(1) == 'L') {	//Left Scale is Prioritzed
            	addSequential(new Startup(), .00001);
        	    addParallel(new DrivePID(275));
        	    addSequential(new RunElevatorScaleAuto());
        	    addParallel(new TurnPID(42.5), .5);
        	    addSequential(new GetCube(), 1);
        	    addSequential(new DriveShortPID(34),.1);
        	    addSequential(new ShootCube(), .2);
        	    addSequential(new AutoResetDriveEncoders(), .0001);
        	    addSequential(new DriveBackward(), .25);
        	    addParallel(new SetElevatorIntakeAuto());
        		addSequential(new TurnPIDLong(90));

           	}
           	else {
           		if(gameInfo.charAt(0) == 'L') {	//If No Scale then go to Switch
	            	addSequential(new Startup(), .00001);
	                addSequential(new DrivePID(132));
	                addSequential(new RunElevatorSwitchAuto());
	                addSequential(new TurnPID(85));
	                addSequential(new ShootCube(), .2);
           		}
           		else {	//Last Option is Opposite Scale; Also the Riskiest
	            	addSequential(new Startup(), .00001);
	                addSequential(new DrivePID(215));
	              	addSequential(new AutoResetDriveEncoders(), .025);
	                addSequential(new TurnPID(85)); 
	                addSequential(new AutoResetDriveEncoders(), .025);
	                addSequential(new DrivePID(150)); //was 190
/*	                addSequential(new AutoResetDriveEncoders(), .025);
	               	addSequential(new CurveLeft(), .7);
	                addSequential(new RunElevatorScaleAuto());
	                addSequential(new GetCube(), 1);
	        	    addSequential(new DriveShortPID(5),.1);
	        	    addSequential(new ShootCube(), .2);*/
           		}
        	}
       		
       	}
       

/*	    addSequential(new AutoResetDriveEncoders(), .0001);
		addParallel(new DriveShortPID(40));
		addSequential(new GetCube(), 2);*/
/*	    addSequential(new AutoResetDriveEncoders(), .0001);
		addParallel(new RunElevatorScaleAuto());
		addSequential(new TurnPIDLong(150));
	    addSequential(new AutoResetDriveEncoders(), .0001);
		addSequential(new DriveShortPID(35));
	    addSequential(new ShootCube(), .2);
	    addSequential(new AutoResetDriveEncoders(), .0001);*/
//		addParallel(new TurnPIDLong(108));//Curve
//		addSequential(new SetElevatorIntakeAuto());
//	    addSequential(new AutoResetDriveEncoders(), .0001);
//		addParallel(new DriveShortPID(39));
//		addSequential(new GetCube(), 1.9);
//	    addSequential(new AutoResetDriveEncoders(), .0001);
//		addParallel(new RunElevatorScaleAuto());
//		addSequential(new TurnPIDLong(180));
//		addSequential(new DriveShortPID(35));

//	    addSequential(new ShootCubeSlow(), .2);


	
		
/*    	addSequential(new Startup(), .00001);
	    addParallel(new DrivePID(266));
	    addSequential(new RunElevatorScaleAuto());
	    addSequential(new TurnPID(25), .4);
	    addSequential(new ShootCube(), .2);
	    addSequential(new AutoResetDriveEncoders(), .0001);
	    addParallel(new SetElevatorIntakeAuto());
		addSequential(new TurnPIDLong(125));//Curve
	    addSequential(new AutoResetDriveEncoders(), .0001);
		addParallel(new DriveShortPID(41));									Working 2 Cube
		addSequential(new GetCube(), 1.6);
		addParallel(new RunElevatorScaleAuto());
		addSequential(new TurnPIDLong(210));
		addSequential(new DriveShortPID(35));
	    addSequential(new ShootCube(), .2);
	    addParallel(new SetElevatorIntakeAuto());*/
		




    }
}
