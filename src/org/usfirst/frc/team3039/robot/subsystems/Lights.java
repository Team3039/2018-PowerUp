package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lights extends Subsystem {

    public DigitalOutput cubeLight = new DigitalOutput(RobotMap.cubeLight);
    public DigitalOutput ratchetLight = new DigitalOutput(RobotMap.ratchetLight);

    public void cubeIn() {
    	cubeLight.set(true);
    }
    
    public void cubeOut() {
    	cubeLight.set(false);
    }
    
    public void ratchetEngaged() {
    	ratchetLight.set(true);
    }
    
    public void ratchetDisengaged() {
    	ratchetLight.set(false);
    }
    
    public void checkLights() {
    	//System.out.println("CUBE" + cubeLight.get());
    	//System.out.println("RATCHET" + ratchetLight.get());
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

