package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunRampElevator extends CommandGroup {

    public RunRampElevator() {
    	addSequential(new SetIntakeStraight(), .1);
    	addSequential(new SetElevatorSlow(), .5);
    }
}
