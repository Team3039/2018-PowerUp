package org.usfirst.frc.team3039.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static final int pilot = 0;
	public static final int coPilot = 1;
	
//CAN
	//Drivetrain
	public static int frontleftMotor = 1;
	public static int frontrightMotor = 8;
	public static int rearrightMotor = 9;
	public static int rearleftMotor = 0;
	
	//Elevator
	public static int elevatorMotorA = 3;
	public static int elevatorMotorB = 7;
	
	
	//Intake
	public static int leftIntakeMotor = 4;
	public static int rightIntakeMotor = 5;
	public static int rotateIntakeMotor = 6;
	
//DIO	
	//Encoder
	public static int driveEncoderA = 0;
	public static int driveEncoderB = 1;
	public static int elevatorEncoderA = 2;
	public static int elevatorEncoderB = 3;
	public static int intakeEncoderA = 7;
	public static int intakeEncoderB = 8;
	//Limit Switches
	public static int maxLevel = 4;
	public static int midLevel = 5;
	public static int minLevel = 6;
	public static int cubeSwitch = 9;
	//Light Output
	public static int cubeLight = 10;
	public static int ratchetLight = 11;
//Solenoids
	//EvoShifter
	public static int evoshiftCylinder = 0;
	public static int ratchetCylinder = 1;
	//public static int elevatorBrake = 2;
}
