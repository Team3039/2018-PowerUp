package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunElevatorScaleAuto extends CommandGroup {

    public RunElevatorScaleAuto() {
		addParallel(new SetAutoIntakePID(),.1);
    	addSequential(new SetElevatorScaleAuto());
		
    }
}
