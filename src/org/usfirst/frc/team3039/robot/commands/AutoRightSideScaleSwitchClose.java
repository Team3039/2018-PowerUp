package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightSideScaleSwitchClose extends CommandGroup {

    public AutoRightSideScaleSwitchClose() {
		addSequential(new Startup(), .001);
	    addParallel(new DrivePID(278));
	    addSequential(new RunElevatorScaleAuto());
	    addSequential(new TurnPID(-25), .58);
	    addSequential(new ShootCube(), .2);
	    addSequential(new AutoResetDriveEncoders(), .001);
	    addParallel(new SetElevatorIntakeAuto());
		addSequential(new TurnPIDLong(-125.5));//Curve
	    addSequential(new AutoResetDriveEncoders(), .001);
	    addParallel(new DriveShortPID(44));
		addSequential(new GetCube(), 2);
		addSequential(new AutoResetDriveEncoders(), .001);
		addSequential(new RunElevatorSwitchAuto());
		addParallel(new DriveForward(), .2);
		addSequential(new ShootCube(), .2);
    }
}
