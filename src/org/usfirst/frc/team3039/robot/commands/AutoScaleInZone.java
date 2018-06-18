package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoScaleInZone extends CommandGroup {

    public AutoScaleInZone() {
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
            			addSequential(new Startup(), .001);
                	    addSequential(new AutoDrivePID(280));
                	    addSequential(new RunElevatorScaleAuto());
                	    addSequential(new AutoTurnPID(60));
                	    addSequential(new ShootCubeSlow(), .3);

                	}
                	
                	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX2 Outside the Zone");
            	 		addSequential(new Startup(), .001);
            		 	addSequential(new AutoDrivePID(223));
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addSequential(new AutoTurnPID(85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);
                		addSequential(new RunElevatorSwitchAuto());
            		 	addSequential(new AutoDrivePID(182));


            			 	


                		
                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
            			addSequential(new Startup(), .001);
                	    addSequential(new AutoDrivePID(280));
                	    addSequential(new RunElevatorScaleAuto());
                	    addSequential(new AutoTurnPID(60));
                	    addSequential(new ShootCubeSlow(), .3);






                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
            	 		addSequential(new Startup(), .001);
            		 	addSequential(new AutoDrivePID(223));
                		addSequential(new AutoResetDriveEncoders(), .025);
            		 	addSequential(new AutoTurnPID(85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .025);
                		addSequential(new RunElevatorSwitchAuto());
            		 	addSequential(new AutoDrivePID(182));                	}
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

