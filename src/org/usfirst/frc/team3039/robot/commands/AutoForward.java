package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoForward extends CommandGroup {

    public AutoForward() {
    	//addSequential(new AutoIntakePID(), .2);
    	addSequential(new Startup(), .1);
    	addSequential(new DrivePID(132));


    	

   		

 }
}
    
    
	    
    

