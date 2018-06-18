package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveRobotSwitchAuto extends CommandGroup {

    public MoveRobotSwitchAuto() {
		addParallel(new RunElevatorSwitchAuto());
    	addSequential(new DriveStraight(.6, 70));
    }
}
