package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSideScaleFar extends CommandGroup {

    public AutoLeftSideScaleFar() {
 		addSequential(new Startup(), .001);
	 	addSequential(new DrivePID(232));
		addSequential(new AutoResetDriveEncoders(), .001);
	 	addSequential(new TurnPID(85)); // Turn To Switch
		addSequential(new AutoResetDriveEncoders(), .001);
	 	addParallel(new DrivePID(178));
    	addSequential(new RunElevatorScaleAuto());
		addSequential(new AutoResetDriveEncoders(), .001);
    	addSequential(new TurnPID(-95)); // Turn To Switch
		addSequential(new AutoResetDriveEncoders(), .001);        		
	    addSequential(new DriveForward() ,.85);
    	addSequential(new ShootCube(), .5);
    	addSequential(new DriveBackward(), .65);
        addParallel(new SetElevatorIntakeAuto());
    	addSequential(new TurnPID(155)); // Turn To Switch
    }
}
