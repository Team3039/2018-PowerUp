package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    public static Solenoid liftoffA = new Solenoid(RobotMap.liftoffA);
    public static Solenoid liftoffB = new Solenoid(RobotMap.liftoffB);
    public static Solenoid release = new Solenoid(RobotMap.release);
    
    public void liftoff() {
    	liftoffA.set(true);
    	liftoffB.set(true);
    }
    
    public void setdown() {
    	liftoffA.set(false);
    	liftoffB.set(false);
    }
    
    public void relaseFork() {
    	release.set(true);
    }
    
    public void detractFork() {
    	release.set(false);
    }
    public void initDefaultCommand() {
        
    }
}

