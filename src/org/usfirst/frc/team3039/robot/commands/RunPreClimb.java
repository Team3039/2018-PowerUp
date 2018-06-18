package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RunPreClimb extends CommandGroup {

    public RunPreClimb() {
        addSequential(new RunElevatorSwitch());    	
        addSequential(new SetIntakeDown());

    }
}
