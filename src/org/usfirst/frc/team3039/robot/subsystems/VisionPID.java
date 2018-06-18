package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class VisionPID extends PIDSubsystem {
	double targetAngle = 320;
	
	public static double kP = .0043;
	public static double kI = .0001;
	public static double kD = .0;
	
    public VisionPID() {
    	super("PIDControl", kP,kI, kD);
    	setInputRange(0, 640);
		setOutputRange(-.6, .6);//voltage
		setSetpoint(targetAngle);
		getPIDController().setContinuous(false);
		setAbsoluteTolerance(2.5);

    }
    
	public void enablePID() {
		enable();
		Robot.drivetrain.unBrake();
		setSetpoint(targetAngle);
		
	}
	
	public void disablePID() {
		disable();
		Robot.drivetrain.driveStop();
		Robot.drivetrain.brake();
	}
	
    public void initDefaultCommand() {
    	Robot.drivetrain.getAngle();
    }

    protected double returnPIDInput() {
        return Robot.cubeArrayX[0]; 
    }

    protected void usePIDOutput(double output) { 	
        	Robot.drivetrain.rotate(-output);
        	
    	System.out.println("The Target's Position is " + Robot.cubeArrayX[0]);    
    	System.out.println("The Output is " + output);
    	System.out.println("On Target " + onTarget());
    }
}
