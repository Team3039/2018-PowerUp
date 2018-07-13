package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSideScaleCompatible extends CommandGroup {

    public AutoLeftSideScaleCompatible() {
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
                	    addParallel(new DrivePID(278));
                	    addSequential(new RunElevatorScaleAuto());
                		addSequential(new AutoResetDriveEncoders(), .0001);
                	    addSequential(new TurnPID(25), .58);
                	    addSequential(new ShootCube(), .2);
                	    addParallel(new DrivePID(-150));
                	    addSequential(new SetElevatorIntake());

                	}
                	
                	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX3 On the Back-side");
            	 		addSequential(new Startup(), .00001);
            	 		addParallel(new DrivePID(196));
            	 		addSequential(new RunElevatorSwitchAuto());
                	    addSequential(new AutoResetDriveEncoders(), .001);
            	 		addSequential(new TurnPIDLong(135));
            	 		addSequential(new ShootCube(), .2);
 
                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'L') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/Switch Outside the Zone");
            			addSequential(new Startup(), .001);
                	    addParallel(new DrivePID(278));
                	    addSequential(new RunElevatorScaleAuto());
                		addSequential(new AutoResetDriveEncoders(), .0001);
                	    addSequential(new TurnPID(25), .58);
                	    addSequential(new ShootCube(), .2);
                	    addParallel(new DrivePID(-150));
                	    addSequential(new SetElevatorIntake());

                	}
                	
                	else if (gameInfo.charAt(0) == 'R' && gameInfo.charAt(1) == 'R') {
                		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX3 On the Back-side");
                 		addSequential(new Startup(), .001);
                	 	addSequential(new DrivePID(223));
                		addSequential(new AutoResetDriveEncoders(), .0001);
                	 	addSequential(new TurnPID(85)); // Turn To Switch
                		addSequential(new AutoResetDriveEncoders(), .0001);
                	 	addSequential(new DrivePID(100));
                	}
               	}
               	else {
            		//System.out.println("Drive Forward");
            		addSequential(new Startup(), .2);
            		addSequential(new DrivePID(140));
               	}
       	}
       	catch(Exception e) {
       		//System.out.println("Drive Forward");
       		addSequential(new Startup(), .2);
       		addSequential(new DrivePID(140));
       	}
	}
}

