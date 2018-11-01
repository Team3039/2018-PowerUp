package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lights extends Subsystem {

    public DigitalOutput cubeLight = new DigitalOutput(RobotMap.cubeLight);
    public DigitalOutput allianceLight = new DigitalOutput(RobotMap.allianceLight);

    public void cubeIn() {
    	cubeLight.set(true);
    }
    
    public void cubeOut() {
    	cubeLight.set(false);
    }
    
    public void swapAlliance(boolean alliance) {
    	allianceLight.set(alliance);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

