package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoTest extends CommandGroup {
    public AutoTest() {
       	String gameInfo;
    	gameInfo = DriverStation.getInstance().getGameSpecificMessage();
    	int count = 0;
       	while(count < 1000) {
    		count++;
    		gameInfo = DriverStation.getInstance().getGameSpecificMessage();
       	}
       	
    
       	addSequential(new CurveLeft(), .5);
    }
   }

    






