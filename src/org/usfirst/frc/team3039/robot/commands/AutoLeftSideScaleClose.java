package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSideScaleClose extends CommandGroup {

    public AutoLeftSideScaleClose() {    	
    	addSequential(new Startup(), .00001);
	    addParallel(new DrivePID(264));
	    addSequential(new RunElevatorScaleAuto());
	    addSequential(new TurnPID(25), .45);
	    addSequential(new ShootCube(), .2);
	    addSequential(new AutoResetDriveEncoders(), .0001);
	    addParallel(new SetElevatorIntakeAuto());
		addSequential(new TurnPIDLong(125));//Curve
	    addSequential(new AutoResetDriveEncoders(), .0001);
		addParallel(new DriveShortPID(36.5));
		addSequential(new GetCube(), 1.55);
	    addSequential(new AutoResetDriveEncoders(), .0001);
		addParallel(new RunElevatorScaleAuto());
		addSequential(new TurnPIDLong(218));
	    addSequential(new AutoResetDriveEncoders(), .0001);
		addSequential(new DriveShortPID(35));
	    addSequential(new ShootCube(), .2);
	    addSequential(new AutoResetDriveEncoders(), .0001);
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
