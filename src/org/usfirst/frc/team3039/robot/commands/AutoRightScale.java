package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightScale extends CommandGroup {

    public AutoRightScale() {
		addSequential(new Startup(), .001);
	    addParallel(new DrivePID(278));
	    addSequential(new RunElevatorScaleAuto());
	    addSequential(new TurnPID(-35));
	    addSequential(new ShootCube(), .25);
	    addSequential(new DriveBackward(), .2);
	    addSequential(new AutoResetDriveEncoders(), .001);
	    addParallel(new SetElevatorIntakeAuto());
		addSequential(new TurnPID(-100));//Curve
//		addSequential(new AutoResetDriveEncoders(), .001);
//		addSequential(new AutoGetCube(), 1);
//		addSequential(new GetCube(), .5);
//		addSequential(new AutoResetDriveEncoders(), .001);
//	    addSequential(new DriveBackward(), .85);
//		addParallel(new TurnPID(-115));//Curve
//	    addSequential(new RunElevatorScaleAuto());
//	    addSequential(new DriveShortPID(47), 1.5);
//	    addSequential(new ShootCube(), .2);
//	    addSequential(new DriveBackward(), .35);
//		addSequential(new AutoResetDriveEncoders(), .025);
//	    addParallel(new SetElevatorIntakeAuto());
//		addSequential(new TurnPIDLong(95));//Curve
//		addSequential(new AutoResetDriveEncoders(), .025);
//		addSequential(new DrivePID(54));
//	  	addParallel(new GetCube(), 2.7);
//		addSequential(new TurnPIDLong(200));//Curve 	//Three Cube
//	    addParallel(new DrivePID(46));
//	    addSequential(new RunElevatorScaleAuto());
//	    addSequential(new ShootCubeSlow());
    }
}
