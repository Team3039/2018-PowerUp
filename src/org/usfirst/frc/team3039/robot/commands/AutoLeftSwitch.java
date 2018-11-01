package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSwitch extends CommandGroup {

    public AutoLeftSwitch() {
        String gameInfo = DriverStation.getInstance().getGameSpecificMessage();
        int count = 0;
        while(count < 1000) {
            count++;
            gameInfo = DriverStation.getInstance().getGameSpecificMessage();
        }

       	if(gameInfo.isEmpty()) {
    	    addSequential(new DrivePID(250));
       	}
       	else {
       		if(gameInfo.charAt(0) == 'L') {	//Left Switch
            	addSequential(new Startup(), .00001);
                addSequential(new DrivePID(132));
                addSequential(new RunElevatorSwitchAuto());
                addSequential(new TurnPID(85));
                addSequential(new ShootCube(), .2);
       		}
       		else {
        	    addSequential(new DrivePID(250));
       		}
       	}
    }
}
