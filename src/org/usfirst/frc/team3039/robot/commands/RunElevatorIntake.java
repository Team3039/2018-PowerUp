package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunElevatorIntake extends CommandGroup {

    public RunElevatorIntake() {
        addSequential(new SetElevatorIntake());
       
    }
}
