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
    	        		System.out.println("I am on the Blue Alliance in the Right Position and should score Scale");
    	    	 		addSequential(new Startup(), .25);
    	    		 	addSequential(new AutoDrivePID(194));
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	    		 	addSequential(new AutoTurnPID(-80)); // Turn To Switch
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	    		 	addSequential(new AutoDrivePID(160));
    	        		addSequential(new AutoResetDriveEncoders(), .2);
    	    	    	addSequential(new AutoTurnPID(90)); // Turn To Switch
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	        		addSequential(new LowerIntake(), .5);
    	    		    addSequential(new SetElevatorScaleAuto());
    	    	    	addSequential(new ShootCube(), .5);
    	        	    addSequential(new DriveStraight(-.5, 10));
    	    	        addSequential(new SetElevatorIntakeAuto());
    	        		addSequential(new AutoResetDriveEncoders(), .1);
//    	    		 	addSequential(new AutoDrivePID(-15));
    	    	    	addSequential(new AutoTurnPID(-110)); // Turn To Switch

    	        	}
    	        	
    	        	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
    	        		//System.out.println("I am on the Blue Alliance in the Right Position and should score Scale");
            			addSequential(new Startup(), .001);
                	    addParallel(new AutoDrivePID(278));
                	    addSequential(new RunElevatorScaleAuto());
                	    addSequential(new AutoTurnPID(-35));
                	    addSequential(new ShootCube(), .25);
                	    addSequential(new DriveBackward(), .2);
                	    addSequential(new AutoResetDriveEncoders(), .001);
                	    addParallel(new SetElevatorIntakeAuto());
                		addSequential(new AutoTurnPID(-100));//Curve
//                		addSequential(new AutoResetDriveEncoders(), .001);
//                		addSequential(new AutoGetCube(), 1);
//                		addSequential(new GetCube(), .5);
//                		addSequential(new AutoResetDriveEncoders(), .001);
//                	    addSequential(new DriveBackward(), .85);
//                		addParallel(new AutoTurnPID(-115));//Curve
//                	    addSequential(new RunElevatorScaleAuto());
//                	    addSequential(new AutoDriveShortPID(47), 1.5);
//                	    addSequential(new ShootCube(), .2);
//                	    addSequential(new DriveBackward(), .35);
//                		addSequential(new AutoResetDriveEncoders(), .025);
//                	    addParallel(new SetElevatorIntakeAuto());
//                		addSequential(new AutoTurnPIDLong(95));//Curve
//                		addSequential(new AutoResetDriveEncoders(), .025);
//                		addSequential(new AutoDrivePID(54));
//                	  	addParallel(new GetCube(), 2.7);
//                		addSequential(new AutoTurnPIDLong(200));//Curve 	//Three Cube
//                	    addParallel(new AutoDrivePID(46));
//                	    addSequential(new RunElevatorScaleAuto());
//                	    addSequential(new ShootCubeSlow());
    	        		
    	        	}
    	        	
    	        	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
    	        		//System.out.println("I am on the Blue Alliance in the Right Position and should score Scale");
    	    	 		addSequential(new Startup(), .25);
    	    		 	addSequential(new AutoDrivePID(194));
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	    		 	addSequential(new AutoTurnPID(-80)); // Turn To Switch
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	    		 	addSequential(new AutoDrivePID(160));
    	        		addSequential(new AutoResetDriveEncoders(), .2);
    	    	    	addSequential(new AutoTurnPID(90)); // Turn To Switch
    	        		addSequential(new AutoResetDriveEncoders(), .1);
    	        		addSequential(new LowerIntake(), .5);
    	    		    addSequential(new SetElevatorScaleAuto());
    	    	    	addSequential(new ShootCube(), .5);
    	        	    addSequential(new DriveStraight(-.5, 10));
    	    	        addSequential(new SetElevatorIntakeAuto());
    	        		addSequential(new AutoResetDriveEncoders(), .1);
//    	    		 	addSequential(new AutoDrivePID(-15));
    	    	    	addSequential(new AutoTurnPID(-110)); // Turn To Switch
    	    	    	}
    	        	
    	        	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
    	        		//System.out.println("I am on the Blue Alliance in the Right Position and should score Scale");
            			addSequential(new Startup(), .25);
                	    addSequential(new AutoDrivePID(204.5));
                		addSequential(new AutoResetDriveEncoders(), .1);
                		addSequential(new AutoTurnPID(-45));//Curve
                		addSequential(new AutoResetDriveEncoders(), .1);
                		addSequential(new LowerIntake(), .5);
                	    addSequential(new SetElevatorScaleAuto());
                	    addSequential(new ShootCube(), .2);
                	    addSequential(new DriveStraight(-.5, 10));
                	    addSequential(new SetElevatorIntakeAuto());
                		addSequential(new AutoResetDriveEncoders(), .25);
                		addSequential(new AutoTurnPID(-90));//Curve
                	    addParallel(new GetCube(), 2);
                	    addParallel(new AutoDrivePID(45));


    	        	}
    	       	}
    	       	else {
    	    		System.out.println("Drive Forward");
    	    		addSequential(new Startup(), .2);
    	    		addSequential(new AutoDrivePID(25));
    	       	}
       }
       catch(Exception e) {
      		System.out.println("Drive Forward");
      		addSequential(new Startup(), .2);
      		addSequential(new AutoDrivePID(25));
       }
   	}
}

