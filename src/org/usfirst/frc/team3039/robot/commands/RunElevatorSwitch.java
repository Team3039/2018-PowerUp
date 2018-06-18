package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunElevatorSwitch extends CommandGroup {

    public RunElevatorSwitch() {
    	addSequential(new SetIntakeStraight(), .1);
    	addSequential(new SetElevatorSlow(), .35);
        addSequential(new SetElevatorSwitch());
    }
}
