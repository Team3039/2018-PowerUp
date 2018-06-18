package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRight extends CommandGroup {

    public AutoRight() {
    	String gameInfo;
    	gameInfo = DriverStation.getInstance().getGameSpecificMessage();
    	int count = 0;
       	while(count < 1000) {
    		count++;
    		gameInfo = DriverStation.getInstance().getGameSpecificMessage();
       }
       try {
    	   if(!gameInfo.equals(null)) {
           	//In the Right Position on the Blue Alliance and you are scoring Outside the Zone with the gameInfo ""
       	    if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'L') {
       	      	//System.out.println("I am on the Blue Alliance in the Right Position and should go Switch/Scale Outside the Zone");
       	 		addSequential(new Startup(), .25);
       			addSequential(new SetElevatorIntake());
       		 	addSequential(new AutoDrivePID(200));
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoDrivePID(210));
           		addSequential(new AutoResetDriveEncoders(), .1);
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoDrivePID(35));
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new LowerIntake(), .5);
       		 	addSequential(new SetElevatorSwitch());
           		addSequential(new AutoResetDriveEncoders(), .1);
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
          	 	addParallel(new DriveForward(), .75);
          		addParallel(new ShootCube(), 1);
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new DriveStraight(.5, -15));
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch

       	
       		 	
       		 	/*addSequential(new LowerIntake(), .5);
       	    	addSequential(new SetElevatorSwitch());
       	    	addSequential(new AutoElevatorBrake(), .25);
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
          		 	addSequential(new AutoDrivePID(15));
          		 	addSequential(new DriveForward(), 1.25);
       	    	addSequential(new ShootCube(), 1);
       	    	
       	    	//Hope
       			addSequential(new DriveStraight(-.5, 20));
       			addSequential(new SetElevatorIntake());				/To Close to the Switch
       			addSequential(new AutoTurnPID(7));
       			addSequential(new AutoGetCube());				
       			addSequential(new SetElevatorSwitch());
       			addSequential(new ShootCube(), 1);
       			addSequential(new DriveStraight(-.5, 20));
       			addSequential(new SetElevatorIntake());*/
       	   }
       	      	    	
       	    else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
       	        //System.out.println("I am on the Blue Alliance in the Right Position and should go Scale/Switch Outside the Zone");
       	 		addSequential(new Startup(), .25);
       			addSequential(new SetElevatorIntake());
       		 	addSequential(new AutoDrivePID(200));
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoDrivePID(210));
           		addSequential(new AutoResetDriveEncoders(), .1);
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new AutoDrivePID(35));
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new LowerIntake(), .5);
           		addSequential(new SetElevatorSwitch());
           		addSequential(new AutoResetDriveEncoders(), .1);
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
          	 	addParallel(new DriveForward(), .75);
          	 	addParallel(new ShootCube(), 1);
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new DriveStraight(.5, -15));
       	    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
       	
       		        
       	        
       	        
       	        
       	        
       	        /*		 		addSequential(new Startup(), .1);
       				addSequential(new SetElevatorIntake());
       			 	addSequential(new AutoDrivePID(184));
       	    		addSequential(new AutoResetDriveEncoders(), .1);
       			 	addSequential(new AutoTurnPID(-85)); // Turn To Switch
       	    		addSequential(new AutoResetDriveEncoders(), .1);
       			 	addSequential(new AutoDrivePID(115));
       		    	addSequential(new LowerIntake(), .5);
       		    	addSequential(new SetElevatorSwitch());
       		    	addSequential(new AutoElevatorBrake(), .25);
       		    	addSequential(new AutoTurnPID(-85)); // Turn To Switch
       	    		addSequential(new AutoResetDriveEncoders(), .1);
       	   		 	addSequential(new AutoDrivePID(15));
       	   		 	addSequential(new DriveForward(), 1.25);
       		    	addSequential(new ShootCube(), 1);
       		    	
       		    	//Hope
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());
       				addSequential(new AutoTurnPID(7));
       				addSequential(new AutoGetCube());				
       				addSequential(new SetElevatorSwitch());
       				addSequential(new ShootCube(), 1);
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());*/
       	   }
       	      	    	
       	      	    	
       	    else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
       	      	//System.out.println("I am on the Blue Alliance in the Right Position and should go Swicth/Scale Outside the Zone");
           		addSequential(new Startup(), .25);
       		 	addSequential(new AutoDrivePID(132));
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new LowerIntake(), .5);
           		addSequential(new SetElevatorSwitch());
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new DriveStraight(.5, 10));
       		 	addSequential(new DriveForward(), 1);
           		addSequential(new ShootCube(), 1);
       	    	addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new DriveStraight(.5, -15));
       	    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	      	
       	      	
       	      	
       	      	/*	addSequential(new Startup(), .1);
       				addSequential(new SetElevatorIntake());
       				addSequential(new AutoDrivePID(187.5));
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoTurnPID(-80)); // Turn To Switch
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoDrivePID(31));
       				addSequential(new LowerIntake(), .5);
       				addSequential(new SetElevatorSwitch());
       				addSequential(new AutoElevatorBrake(), .25);
       				addSequential(new AutoTurnPID(-80)); // Turn To Switch
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoDrivePID(9.5));
       				addSequential(new DriveForward(), 1.25);
       				addSequential(new ShootCube(), 1);
       				
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());
       				addSequential(new AutoTurnPID(7));
       				addSequential(new AutoGetCube());			
       				addSequential(new SetElevatorSwitch());
       				addSequential(new ShootCube(), 1);
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());*/
       	   }
       	      	    	
       	    else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
       	      	//System.out.println("I am on the Blue Alliance in the Right Position and should go Scale/SwitchX2 Outside the Zone");
           		addSequential(new Startup(), .25);
       		 	addSequential(new AutoDrivePID(132));
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new AutoTurnPID(-85)); // Turn To Switch
           		addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new LowerIntake(), .5);
           		addSequential(new SetElevatorSwitch());
           		addSequential(new AutoResetDriveEncoders(), .1);
       		 	addSequential(new DriveStraight(.5, 10));
       		 	addSequential(new DriveForward(), 1);
           		addSequential(new ShootCube(), 1);
       	    	addSequential(new AutoResetDriveEncoders(), .1);
           		addSequential(new DriveStraight(.5, -15));
       	    	addSequential(new AutoTurnPID(85)); // Turn To Switch
       	      	
       	      	
       	      	/*	addSequential(new Startup(), .1);
       				addSequential(new SetElevatorIntake());
       				addSequential(new AutoDrivePID(187.5));
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoTurnPID(-80)); // Turn To Switch
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoDrivePID(31));
       				addSequential(new LowerIntake(), .5);
       				addSequential(new SetElevatorSwitch());
       				addSequential(new AutoElevatorBrake(), .25);
       				addSequential(new AutoTurnPID(-80)); // Turn To Switch
       				addSequential(new AutoResetDriveEncoders(), .1);
       				addSequential(new AutoDrivePID(9.5));
       				addSequential(new DriveForward(), 1.25);
       				addSequential(new ShootCube(), 1);
       				
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());
       				addSequential(new AutoTurnPID(7));
       				addSequential(new AutoGetCube());			
       				addSequential(new SetElevatorSwitch());
       				addSequential(new ShootCube(), 1);
       				addSequential(new DriveStraight(-.5, 20));
       				addSequential(new SetElevatorIntake());*/
       	   }
          	}
          
          else {
   	   		addSequential(new Startup(), .2);
   	   		addSequential(new AutoDrivePID(25));
          }
       }
       catch(Exception e) {
  	   		addSequential(new Startup(), .2);
  	   		addSequential(new AutoDrivePID(25));
       }       
    }    				  		
}
