package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftScale extends CommandGroup {

    public AutoLeftScale() {
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
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/SwitchX2 Outside the Zone");
                    	addSequential(new Startup(), .00001);
                	    addParallel(new AutoDrivePID(264));
                	    addSequential(new RunElevatorScaleAuto());
                	    addSequential(new AutoTurnPID(25), .45);
                	    addSequential(new ShootCube(), .2);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                	    addParallel(new SetElevatorIntakeAuto());
                		addSequential(new AutoTurnPIDLong(125));//Curve
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addParallel(new AutoDriveShortPID(36.5));
                		addSequential(new GetCube(), 1.55);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addParallel(new RunElevatorScaleAuto());
                		addSequential(new AutoTurnPIDLong(218));
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addSequential(new AutoDriveShortPID(35));
                	    addSequential(new ShootCube(), .2);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new TurnPIDLong(108));//Curve
//                		addSequential(new SetElevatorIntakeAuto());
//                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new DriveShortPID(39));
//                		addSequential(new GetCube(), 1.9);
//                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new RunElevatorScaleAuto());
//                		addSequential(new TurnPIDLong(180));
//                		addSequential(new DriveShortPID(35));

//                	    addSequential(new ShootCubeSlow(), .2);






                	}
                	
                	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX2 Outside the Zone");
            	 		addSequential(new Startup(), .001);
            		 	addSequential(new AutoDrivePID(223));
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addSequential(new AutoTurnPID(85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addParallel(new AutoDrivePID(172));
            	    	addSequential(new RunElevatorScaleAuto());
                		addSequential(new AutoResetDriveEncoders(), .025);
            	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);        		
        			    addSequential(new DriveForward() ,.55);
        		    	addSequential(new ShootCube(), .5);
        		    	addSequential(new DriveBackward(), .5);
            	        addParallel(new SetElevatorIntakeAuto());
            	    	addSequential(new AutoTurnPID(180)); // Turn To Switch

            			 	


                		
                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/Switch Outside the Zone");
                    	addSequential(new Startup(), .00001);
                	    addParallel(new AutoDrivePID(264));
                	    addSequential(new RunElevatorScaleAuto());
                	    addSequential(new AutoTurnPID(25), .45);
                	    addSequential(new ShootCube(), .2);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                	    addParallel(new SetElevatorIntakeAuto());
                		addSequential(new AutoTurnPIDLong(125));//Curve
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addParallel(new AutoDriveShortPID(36.5));
                		addSequential(new GetCube(), 1.55);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addParallel(new RunElevatorScaleAuto());
                		addSequential(new AutoTurnPIDLong(218));
                	    addSequential(new AutoResetDriveEncoders(), .0001);
                		addSequential(new AutoDriveShortPID(35));
                	    addSequential(new ShootCube(), .2);
                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new TurnPIDLong(108));//Curve
//                		addSequential(new SetElevatorIntakeAuto());
//                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new DriveShortPID(39));
//                		addSequential(new GetCube(), 1.9);
//                	    addSequential(new AutoResetDriveEncoders(), .0001);
//                		addParallel(new RunElevatorScaleAuto());
//                		addSequential(new TurnPIDLong(180));
//                		addSequential(new DriveShortPID(35));

//                	    addSequential(new ShootCubeSlow(), .2);





                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
            	 		addSequential(new Startup(), .1);
            		 	addSequential(new AutoDrivePID(223));
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addSequential(new AutoTurnPID(85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addParallel(new AutoDrivePID(172));
            	    	addSequential(new RunElevatorScaleAuto());
                		addSequential(new AutoResetDriveEncoders(), .025);
            	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);        		
        			    addSequential(new DriveForward() ,.55);
        		    	addSequential(new ShootCube(), .5);
        		    	addSequential(new DriveBackward(), .5);
            	        addParallel(new SetElevatorIntakeAuto());
            	    	addSequential(new AutoTurnPID(180)); // Turn To Switch
                	}
               	}
               	else {
            		//System.out.println("Drive Forward");
            		addSequential(new Startup(), .2);
            		addSequential(new AutoDrivePID(25));
               	}
       	}
       	catch(Exception e) {
       		//System.out.println("Drive Forward");
       		addSequential(new Startup(), .2);
       		addSequential(new AutoDrivePID(25));
       	}
	}
}

