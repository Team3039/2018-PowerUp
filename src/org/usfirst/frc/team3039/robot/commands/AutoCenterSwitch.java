package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCenterSwitch extends CommandGroup {

    public AutoCenterSwitch() {
    	String gameInfo;
    	gameInfo = DriverStation.getInstance().getGameSpecificMessage();
    	int count = 0;
           	while(count < 1000) {
    		count++;
    		gameInfo = DriverStation.getInstance().getGameSpecificMessage();
       	}
       	
       	try {
       		if(!gameInfo.equals(null)) {

    	//In the Left Position on the Blue Alliance and you are scoring Outside the Zone with the gameInfo ""	    	
    	if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'L') {
    		//System.out.println("I am on the Blue Alliance In the Center");
    		addSequential(new Startup(), .025);
       		addSequential(new CurveLeft(), .45);
    		addSequential(new AutoResetDriveEncoders(), .1);
    		addParallel(new RunElevatorSwitchAuto());
    		addSequential(new DriveShortPID(85));
    		addSequential(new AutoResetDriveEncoders(), .1);
    		addSequential(new CurveRight(), .5);
    		addSequential(new ShootCube(), .2);
    		addSequential(new DriveBackward(), .3);
    		addParallel(new RunElevatorScaleAuto());
    		addSequential(new TurnPID(145));



    	}
    	
    	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/Switch Outside the Zone");
    		addSequential(new Startup(), .025);
       		addSequential(new CurveRight(), .45);
    		addSequential(new AutoResetDriveEncoders(), .025);
    		addParallel(new RunElevatorSwitchAuto());
    		addSequential(new DriveShortPID(85));
    		addSequential(new AutoResetDriveEncoders(), .025);
    		addSequential(new CurveLeft(), .45);
//    		addSequential(new ShootCube(), .2);
//    		addSequential(new DriveBackward(), .35);
//    		addParallel(new TurnPID(-130));
//    		addSequential(new SetElevatorIntakeAuto());


    	}
    	
    	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
    		addSequential(new Startup(), .025);
       		addSequential(new CurveLeft(), .45);
    		addSequential(new AutoResetDriveEncoders(), .1);
    		addParallel(new RunElevatorSwitchAuto());
    		addSequential(new DriveShortPID(85));
    		addSequential(new AutoResetDriveEncoders(), .1);
    		addSequential(new CurveRight(), .45);
//    		addSequential(new ShootCube(), .2);
//    		addSequential(new DriveBackward(), .3);
//    		addParallel(new SetElevatorIntakeAuto());
//    		addSequential(new TurnPID(145));

    	}
    	
    	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
    		addSequential(new Startup(), .025);
       		addSequential(new CurveRight(), .45);
    		addSequential(new AutoResetDriveEncoders(), .025);
    		addParallel(new RunElevatorSwitchAuto());
    		addSequential(new DriveShortPID(85));
    		addSequential(new AutoResetDriveEncoders(), .025);
    		addSequential(new CurveLeft(), .45);
    		addSequential(new ShootCube(), .2);
    		addSequential(new DriveBackward(), .35);
    		addParallel(new TurnPID(-130));
    		addSequential(new SetElevatorIntakeAuto());
    	}
    	
    	else if(gameInfo.equals(null)){
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(25));
    	}
    	else if(gameInfo.length() == 0) {
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(25));
    	}
    	
    	else if(gameInfo.length() < 0) {
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(25));
    	}
   	}
    	else {
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(25));
    	}
    }
     
       	catch(Exception e) {
       		System.out.println("Drive Forward");
       		addSequential(new Startup(), .2);
       		addSequential(new DrivePID(25));
       	}
    }
}
    


