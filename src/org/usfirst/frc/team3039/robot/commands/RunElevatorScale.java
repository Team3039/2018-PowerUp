package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunElevatorScale extends CommandGroup {

    public RunElevatorScale() {
    	addSequential(new SetIntakeStraight(), .1);
    	addSequential(new SetElevatorSlow(), .6);
        addSequential(new SetElevatorScale());
    }
}
