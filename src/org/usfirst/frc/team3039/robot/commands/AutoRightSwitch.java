package org.usfirst.frc.team3039.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightSwitch extends CommandGroup {

    public AutoRightSwitch() {
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
       		if(gameInfo.charAt(0) == 'R') {	//Left Switch
            	addSequential(new Startup(), .00001);
                addSequential(new DrivePID(132));
                addSequential(new RunElevatorSwitchAuto());
               	addSequential(new CurveLeft(), .7); //TODO Fix the Negative Turning, it is very slow with PID
                addSequential(new ShootCube(), .2);
       		}
       		else {
        	    addSequential(new DrivePID(250));
       		}
       	}
    }
}