package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;
import org.usfirst.frc.team3039.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    public static Solenoid ratchetCylinder = new Solenoid(RobotMap.ratchetCylinder);
    
    public void engageRatchet() {
    	ratchetCylinder.set(true);
    	Robot.rachetMode = true;
    	Robot.lights.ratchetEngaged();
    }
    
    public void disengageRatchet() {
    	ratchetCylinder.set(false);
    	Robot.rachetMode = false;
    	Robot.lights.ratchetDisengaged();
    }
    
    public void initDefaultCommand() {
        
    }
}

