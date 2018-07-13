package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightScale extends CommandGroup {

    public AutoRightScale() {
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
     		addSequential(new Startup(), .001);
    	 	addSequential(new DrivePID(223));
    		addSequential(new AutoResetDriveEncoders(), .001);
    	 	addSequential(new TurnPID(-85)); // Turn To Switch
    		addSequential(new AutoResetDriveEncoders(), .001);
    	 	addParallel(new DrivePID(172));
        	addSequential(new RunElevatorScaleAuto());
    		addSequential(new AutoResetDriveEncoders(), .001);
        	addSequential(new TurnPID(85)); // Turn To Switch
    		addSequential(new AutoResetDriveEncoders(), .001);        		
    	    addSequential(new DriveForward() ,.55);
        	addSequential(new ShootCube(), .5);
        	addSequential(new DriveBackward(), .5);
        	addParallel(new TurnPIDLong(150)); // Turn To Switch
            addSequential(new SetElevatorIntakeAuto());



    	}
    	
    	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/Switch Outside the Zone");
     		addSequential(new Startup(), .001);
    	 	addSequential(new DrivePID(223));
    		addSequential(new AutoResetDriveEncoders(), .001);
    	 	addSequential(new TurnPID(-85)); // Turn To Switch
    		addSequential(new AutoResetDriveEncoders(), .001);
    	 	addParallel(new DrivePID(172));
        	addSequential(new RunElevatorScaleAuto());
    		addSequential(new AutoResetDriveEncoders(), .001);
        	addSequential(new TurnPID(85)); // Turn To Switch
    		addSequential(new AutoResetDriveEncoders(), .001);        		
    	    addSequential(new DriveForward() ,.55);
        	addSequential(new ShootCube(), .5);
        	addSequential(new DriveBackward(), .5);
        	addParallel(new TurnPIDLong(150)); // Turn To Switch
            addSequential(new SetElevatorIntakeAuto());


    	}
    	
    	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
    		addSequential(new Startup(), .001);
    	    addParallel(new DrivePID(278));
    	    addSequential(new RunElevatorScaleAuto());
    	    addSequential(new TurnPID(-35));
    	    addSequential(new ShootCube(), .25);
    	    addSequential(new DriveBackward(), .2);
    	    addSequential(new AutoResetDriveEncoders(), .001);
    	    addParallel(new SetElevatorIntakeAuto());
    		addSequential(new TurnPID(-100));//Curve
//    		addSequential(new AutoResetDriveEncoders(), .001);
//    		addSequential(new AutoGetCube(), 1);
//    		addSequential(new GetCube(), .5);
//    		addSequential(new AutoResetDriveEncoders(), .001);
//    	    addSequential(new DriveBackward(), .85);
//    		addParallel(new TurnPID(-115));//Curve
//    	    addSequential(new RunElevatorScaleAuto());
//    	    addSequential(new DriveShortPID(47), 1.5);
//    	    addSequential(new ShootCube(), .2);
//    	    addSequential(new DriveBackward(), .35);
//    		addSequential(new AutoResetDriveEncoders(), .025);
//    	    addParallel(new SetElevatorIntakeAuto());
//    		addSequential(new TurnPIDLong(95));//Curve
//    		addSequential(new AutoResetDriveEncoders(), .025);
//    		addSequential(new DrivePID(54));
//    	  	addParallel(new GetCube(), 2.7);
//    		addSequential(new TurnPIDLong(200));//Curve 	//Three Cube
//    	    addParallel(new DrivePID(46));
//    	    addSequential(new RunElevatorScaleAuto());
//    	    addSequential(new ShootCubeSlow());

    	}
    	
    	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
    		addSequential(new Startup(), .001);
    	    addParallel(new DrivePID(278));
    	    addSequential(new RunElevatorScaleAuto());
    	    addSequential(new TurnPID(-35));
    	    addSequential(new ShootCube(), .25);
    	    addSequential(new DriveBackward(), .2);
    	    addSequential(new AutoResetDriveEncoders(), .001);
    	    addParallel(new SetElevatorIntakeAuto());
    		addSequential(new TurnPID(-100));//Curve
//    		addSequential(new AutoResetDriveEncoders(), .001);
//    		addSequential(new AutoGetCube(), 1);
//    		addSequential(new GetCube(), .5);
//    		addSequential(new AutoResetDriveEncoders(), .001);
//    	    addSequential(new DriveBackward(), .85);
//    		addParallel(new TurnPID(-115));//Curve
//    	    addSequential(new RunElevatorScaleAuto());
//    	    addSequential(new DriveShortPID(47), 1.5);
//    	    addSequential(new ShootCube(), .2);
//    	    addSequential(new DriveBackward(), .35);
//    		addSequential(new AutoResetDriveEncoders(), .025);
//    	    addParallel(new SetElevatorIntakeAuto());
//    		addSequential(new TurnPIDLong(95));//Curve
//    		addSequential(new AutoResetDriveEncoders(), .025);
//    		addSequential(new DrivePID(54));
//    	  	addParallel(new GetCube(), 2.7);
//    		addSequential(new TurnPIDLong(200));//Curve 	//Three Cube
//    	    addParallel(new DrivePID(46));
//    	    addSequential(new RunElevatorScaleAuto());
//    	    addSequential(new ShootCubeSlow());
    	}
    	
    	else if(gameInfo.equals(null)){
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(125));
    	}

    	else {
    		System.out.println("Drive Forward");
    		addSequential(new Startup(), .2);
    		addSequential(new DrivePID(125));
    	}
    }
}
       	catch(Exception e) {
       		System.out.println("Drive Forward");
       		addSequential(new Startup(), .2);
       		addSequential(new DrivePID(125));
       	}
    }
}
    


