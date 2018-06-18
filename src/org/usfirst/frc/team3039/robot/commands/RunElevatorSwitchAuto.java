package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunElevatorSwitchAuto extends CommandGroup {

    public RunElevatorSwitchAuto() {
		addParallel(new AutoIntakePID(), .1);
    	addSequential(new SetElevatorSwitchAuto());
    }
}
