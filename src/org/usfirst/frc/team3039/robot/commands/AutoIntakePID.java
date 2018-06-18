package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoIntakePID extends CommandGroup {

    public AutoIntakePID() {
    	addSequential(new SetAutoIntakePID());
    }
}
