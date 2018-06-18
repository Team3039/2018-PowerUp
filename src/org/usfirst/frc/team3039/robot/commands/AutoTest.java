package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoTest extends CommandGroup {
    public AutoTest() {
       	String gameInfo;
    	gameInfo = DriverStation.getInstance().getGameSpecificMessage();
//    	int count = 0;
//       	while(count < 1000) {
//    		count++;
//    		gameInfo = DriverStation.getInstance().getGameSpecificMessage();
//       	}
       	if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'L') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go Scale/SwitchX2 Outside the Zone");
//    		addSequential(new Startup(), .1);
//       	addSequential(new AutoCurveLeft(), .45);
//    		addSequential(new AutoResetDrsxiveEncoders(), .1);
//    		addParallel(new RunElevatorSwitchAuto());
//    		addSequential(new AutoDriveShortPID(80));
//    		addSequential(new AutoResetDriveEncoders(), .1);
//    		addSequential(new AutoCurveRight(), .5);
//    		addSequential(new ShootCube(), .2);
//    		addSequential(new DriveBackward(), .85);
//    		addParallel(new AutoTurnPID(90));
//    		addSequential(new SetElevatorIntake());
//    		addParallel(new AutoDriveShortPID(30));
//    		addSequential(new GetCube(), 1.5);
    		addSequential(new Startup(), .1);
    		addSequential(new AutoTurnVision());
    		


  	}
    	
    	else if (gameInfo.charAt(0) == 'L' && gameInfo.charAt(1) == 'R') {
    		//System.out.println("I am on the Blue Alliance in the Left Position and should go SwitchX2 Outside the Zone");
//    		addSequential(new AutoCurveLeft(), .35);
//    		addSequential(new AutoResetDriveEncoders(), .1);
//    		addParallel(new DriveStraight(.6, 70));
//    		addParallel(new SetAutoIntakePID());
//	 		addSequential(new SetElevatorSwitchAuto());
//    		addSequential(new AutoResetDriveEncoders(), .1);
//    		addSequential(new ShootCubeSlow(), .2);
    		addSequential(new Startup(), .1);
//    		addSequential(new AutoDriveShortPID(45));
    		addSequential(new AutoDriveShortPID(48));
    		


    	}       	}
    }

    






