package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class IntakePID extends PIDSubsystem {
    
	private double targetAngleDown = 585;
	
	public static double kP = .0610;
	public static double kI = 0.000001;
	public static double kD = .001;
    
    public IntakePID() {
        super("Intake PID", kP, kD, kI);
    	setInputRange(0, 1000);
		setOutputRange(-.6, .6);//Voltage Range
		getPIDController().setContinuous(false);
		setPercentTolerance(.0001);
    }
    
    public void setIntakePosition(double targetAngle) {
    	setSetpoint(targetAngle);
    	enable();
    
    }
    
    public void intakeDown() {
    	setSetpoint(targetAngleDown);
    	enable();
    }
    
    public void stopIntake() {
    	disable();
    	Robot.intake.stopIntakeNoBrake();
    }
    
    protected double returnPIDInput() {
        return Robot.intake.getIntakeEncoder();
    }
    
    public void initDefaultCommand() {
    	Robot.intake.getIntakeEncoder();
            }

    protected void usePIDOutput(double output) {
        Robot.intake.moveIntake(-output);
//        System.out.println("On Target :" + onTarget());
//        System.out.println("Intake Angle" + Robot.intake.getIntakeEncoder());
    }
}
