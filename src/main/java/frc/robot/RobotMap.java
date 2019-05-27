/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


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
	public static int leftEncoderA = 0;
	public static int leftEncoderB = 1;
	public static int elevatorEncoderA = 2;
	public static int elevatorEncoderB = 3;
	public static int intakeEncoderA = 7;
	public static int intakeEncoderB = 8;
	public static int rightEncoderA = 12;
	public static int rightEncoderB = 13;
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
}