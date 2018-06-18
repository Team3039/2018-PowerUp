package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeft extends CommandGroup {

    public AutoLeft() {
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
       	    	    		addSequential(new Startup(), .25);
       	    			 	addSequential(new AutoDrivePID(132));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new LowerIntake(), .5);
       	    	    		addSequential(new SetElevatorSwitchAuto());
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new DriveStraight(.5, 10));
       	    			 	addParallel(new DriveForward(), .5);
       	    	    		addParallel(new ShootCube(), .5);
       	    		    	addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new DriveStraight(.5, -15));
       	    		    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
       	        		
       	        		
       	        		/*	addSequential(new Startup(), .1);
       	    				addSequential(new SetElevatorIntake());
       	    				addSequential(new AutoDrivePID(187.5));
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoDrivePID(31));
       	    				addSequential(new LowerIntake(), .5);
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new AutoElevatorBrake(), .25);
       	    				addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoDrivePID(9.5));
       	    				addSequential(new DriveForward(), .75);
       	    				addSequential(new ShootCube(), 1);
       	    				
       	    				//Hope
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());
       	    				addSequential(new AutoTurnPID(7));
       	    				addSequential(new AutoGetCube());				
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new ShootCube(), 1);
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());*/
       	        	}
       	        	
       	        	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
       	        		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX2 Outside the Zone");
       	      		 		addSequential(new Startup(), .25);
       	       		 		addSequential(new AutoDrivePID(132));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
                    		addSequential(new SetAutoIntakePID(), .2);
       	    	    		addSequential(new SetElevatorSwitchAuto());
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new ShootCube(), 1);					//This is the Working Auto not going behind the switch
       	    		    	addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new DriveStraight(.5, -15));
       	    		    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
       	        		
       	    				/*addSequential(new SetElevatorIntake());
       	    				addSequential(new AutoDrivePID(187.5));
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoDrivePID(31));
       	    				addSequential(new LowerIntake(), .5);
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new AutoElevatorBrake(), .25);
       	    				addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    				addSequential(new AutoResetDriveEncoders(), .1);
       	    				addSequential(new AutoDrivePID(9.5));
       	    				addSequential(new DriveForward(), .75);
       	    				addSequential(new ShootCube(), 1);
       	    				
       	    				//Hope
       	    				addSequential(new DriveStraight(-.5, 25));			To Close to the Switch
       	    				addSequential(new SetElevatorIntake());
       	    				addSequential(new AutoTurnPID(7));
       	    				addSequential(new AutoGetCube());				
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new ShootCube(), 1);
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());*/

       	        	}
       	        	
       	        	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
       	        		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/Switch Outside the Zone");
       	    	 		addSequential(new Startup(), .25);
       	    			addSequential(new SetElevatorIntake());
       	    		 	addSequential(new AutoDrivePID(200));
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	    		 	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	    		 	addSequential(new AutoDrivePID(210));
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	    		 	addSequential(new AutoDrivePID(35));
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	        		addSequential(new LowerIntake(), .5);
       	    		 	addSequential(new SetElevatorSwitch());
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	       		 	addParallel(new DriveForward(), .75);
       	       		 	addParallel(new ShootCube(), 1);
       	        		addSequential(new AutoResetDriveEncoders(), .1);
       	        		addSequential(new DriveStraight(.5, -15));
       	    	    	addSequential(new AutoTurnPID(85)); // Turn To Switch



       	    		    	

       	    			 	;
       	    		    	/*addSequential(new LowerIntake(), .5);
       	    		    	addSequential(new SetElevatorSwitch());
       	    		    	addSequential(new AutoElevatorBrake(), .25);
       	    		    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	   		 	addSequential(new AutoDrivePID(15));
       	    	   		 	addSequential(new DriveForward(), .75);
       	    		    	addSequential(new ShootCube(), 1);
       	    		    	
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());			To close to Switch
       	    				addSequential(new AutoTurnPID(7));
       	    				addSequential(new AutoGetCube());				
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new ShootCube(), 1);
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());*/
       	    	    	
       	        	}
       	        	
       	        	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
       	        		//System.out.println("I am on the Blue Alliance in the Left Position and should go Switch/Scale Outside the Zone");
       	    		 		addSequential(new Startup(), .25);
       	    				addSequential(new SetElevatorIntake());
       	    			 	addSequential(new AutoDrivePID(200));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new AutoDrivePID(210));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    		    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new AutoDrivePID(35));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new LowerIntake(), .5);
       	    			 	addSequential(new SetElevatorSwitch());
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    		    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	   		 	addParallel(new DriveForward(), .75);
       	    	   		 	addParallel(new ShootCube(), 1);
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	    		addSequential(new DriveStraight(.5, -15));
       	    		    	addSequential(new AutoTurnPID(85)); // Turn To Switch

       	        		
       	        		
       	        		
       	        		
       	        		
       	        		
       	        		/*	addSequential(new Startup(), .1);
       	    				addSequential(new SetElevatorIntake());
       	    			 	addSequential(new AutoDrivePID(190));
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    			 	addSequential(new AutoDrivePID(134));
       	    		    	addSequential(new LowerIntake(), .5);
       	    		    	addSequential(new SetElevatorSwitch());
       	    		    	addSequential(new AutoElevatorBrake(), .25);
       	    		    	addSequential(new AutoTurnPID(80)); // Turn To Switch
       	    	    		addSequential(new AutoResetDriveEncoders(), .1);
       	    	   		 	addSequential(new AutoDrivePID(9.5));
       	    		    	addSequential(new ShootCube(), 1);
       	    	   		 	addSequential(new DriveStraight(-.5, 10));
       	    	   		 	addSequential(new DriveForward(), .75);
       	    		    	addSequential(new ShootCube(), 1);

       	    		    	//Hope
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());
       	    				addSequential(new AutoTurnPID(7));
       	    				addSequential(new AutoGetCube());				
       	    				addSequential(new SetElevatorSwitch());
       	    				addSequential(new ShootCube(), 1);
       	    				addSequential(new DriveStraight(-.5, 25));
       	    				addSequential(new SetElevatorIntake());*/
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

