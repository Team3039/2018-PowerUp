package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class TurnPID extends PIDSubsystem {
	
	public static double kP = .0058;
	public static double kI = .0002;
	public static double kD = .1;


    // Initialize your subsystem here
    public TurnPID() {
        super("Rotation PID", kP, kI, kD); 
        setInputRange(-360, 360);
        setOutputRange(-.65, .65);//voltage
        getPIDController().setContinuous(false);
        setPercentTolerance(.2);
    }

    public void initDefaultCommand() {
        Robot.drivetrain.getAngle();
    }

    public void rotate(double targetAngle) {
        setSetpoint(targetAngle);
        enable();
    }
    
    public void disablePID() {
        disable();
        Robot.drivetrain.driveStop();
    }
    protected double returnPIDInput() {
        return Robot.drivetrain.getAngle();
    }

    protected void usePIDOutput(double output) {
    	
    		Robot.drivetrain.frontleftMotor.pidWrite(output);
            Robot.drivetrain.frontrightMotor.pidWrite(output);
            Robot.drivetrain.rearleftMotor.pidWrite(output);
            Robot.drivetrain.rearrightMotor.pidWrite(output);
    	
        System.out.println("On Target:" + onTarget());
        System.out.println("Robot Angle" + Robot.drivetrain.getAngle());
    }
}
