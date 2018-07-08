package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**How PID works:
 * TA = targetAngle- The Angle in which we want to reach
 * RA = Robot Angle- The Current Angle of the Robot
 * 
 * If Both of these Angles are equal to each other then the PID loop is satisfied,
 * however if there is a difference in the two angles there is an error which we need to correct.
 * 
 * Error -is the Distance the Robot is from its targetAngle
 * 
 *P=1/error
 *I= Change Over Time  
 *D= Rate of Change
 *
 *If you are ONLY using P in your PID loop, subtract RA - TA to get your Error 
 *Then multiply you Error by your kP which will get you your Controller output.
 *
 *If you are ONLY using PI in your PID loop, subtract RA - TA to get your Error 
 *Then multiply you Error by your kP and kI. Finally add the two numbers together to get your Controller Output
 *
 *If you are ONLY using PID in your PID loop, subtract RA - TA to get your Error 
 *Then multiply you Error by your kP and kI and kD. Finally add the three numbers together to get your Controller Output 
 *
 *Start By Tuning P until is begins to oscillate slightly. 
 *Then Add I until it reaches its targetAngle quicker with less oscillation. 
 *Finally, If needed add D to smooth out the process 
 */
public class RotatePID extends PIDSubsystem {
	
	public static double kP = .007;
	public static double kI = .0002;
	public static double kD = .052;
	public static double setpoint;


    // Initialize your subsystem here
    public RotatePID() {
        super("Rotation PID", kP, kI, kD); 
        setInputRange(-360, 360);
        setOutputRange(-.8, .6);//voltage
        getPIDController().setContinuous(false);
        setPercentTolerance(.2);
    }

    public void initDefaultCommand() {
        Robot.drivetrain.getAngle();
    }

    public void rotate(double targetAngle) {
        setSetpoint(targetAngle);
        enable();
        setpoint = targetAngle;
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
    	
    	
//        System.out.println("On Target:" + onTarget());
//        System.out.println("Robot Angle" + Robot.drivetrain.getAngle());
    }
}
