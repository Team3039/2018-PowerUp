package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;
import org.usfirst.frc.team3039.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {
	//Evo Shifter Weight Without Motors 3.2
	//Weight of CIM Motor 2.8 Pounds Each
	//Total Weight = 8.8
	//TODO Decrease Weight
	
	public boolean risen;
	public boolean up;
	public boolean movingIntake;
	public boolean movingSwitch;
	public boolean movingScale;
	
	
	public boolean moving;
	public boolean stop;
	
	public WPI_TalonSRX elevatorMotorA = new WPI_TalonSRX(RobotMap.elevatorMotorA);
	public WPI_TalonSRX elevatorMotorB = new WPI_TalonSRX(RobotMap.elevatorMotorB);
	
	public Encoder elevatorEncoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB, false);
	
	public DigitalInput maxLimit = new DigitalInput(RobotMap.maxLevel);
	public DigitalInput midLimit = new DigitalInput(RobotMap.midLevel);
	public DigitalInput minLimit = new DigitalInput(RobotMap.minLevel);
	
	public Solenoid evoshiftCylinder = new Solenoid(RobotMap.evoshiftCylinder);
	
	//public Solenoid elevatorBrake = new Solenoid(RobotMap.elevatorBrake);
	
	//Elevator Control
	public void motorSafety(boolean enabled) {
		elevatorMotorA.setSafetyEnabled(enabled);
		elevatorMotorB.setSafetyEnabled(enabled);
	}
	
	public void liftElevator() {
		if (Robot.elevator.getMax() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(.9);
			elevatorMotorB.set(.9);
		}

	}
	
	public void liftElevatorAuto() {
		if (Robot.elevator.getMax() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(.9);
			elevatorMotorB.set(.9);
		}

	}
	
	public void liftElevatorRampMid() {
		if (Robot.elevator.getMax() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(.8);
			elevatorMotorB.set(.8);
		}

	}
	
	
	public void liftElevatorRampSlow() {
		if (Robot.elevator.getMax() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(.6);
			elevatorMotorB.set(.6);
		}

	}
	
	public void climbSpeed() {
		elevatorMotorA.set(-.6);
		elevatorMotorB.set(-.6);	
	}
	
	public void lowerElevator() {
		if (Robot.elevator.getMin() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(-.96);
			elevatorMotorB.set(-.96);
		}

	}
	
	public void lowerElevatorAuto() {
		if (Robot.elevator.getMin() == false) {
			Robot.elevator.stopElevator();
		}
		else {
			elevatorMotorA.set(-.96);
			elevatorMotorB.set(-.96);
		}

	}
	
	public void stopElevator() {
		elevatorMotorA.set(0);
		elevatorMotorB.set(0);
	}
	
	//Braking0
	public void unBrake() {
		elevatorMotorA.setNeutralMode(NeutralMode.Coast);
		elevatorMotorB.setNeutralMode(NeutralMode.Coast);
		Robot.intake.rotateIntakeMotor.setNeutralMode(NeutralMode.Coast);
	}
	
	public void brake() {
		elevatorMotorA.setNeutralMode(NeutralMode.Brake);
		elevatorMotorB.setNeutralMode(NeutralMode.Brake);
		Robot.intake.rotateIntakeMotor.setNeutralMode(NeutralMode.Brake);
	}
	
	//Encoders
	public void setElevatorEncoder() {
		elevatorEncoder.setDistancePerPulse(0);
	}
	
	public void resetElevatorEncoder() {
		elevatorEncoder.reset();
	}
	
	public double getElevatorEncoder() {
		return Math.abs(elevatorEncoder.getDistance());
	}

	//Limit Switches
	public boolean getMax() {
		return maxLimit.get();
	}
	
	public boolean getMin() {
		return minLimit.get();
	}
	
	public boolean getMid() {
		return midLimit.get();
	}
	
	public void torqueMode() {
		evoshiftCylinder.set(false);
	}
	
	public void speedMode() {
		evoshiftCylinder.set(true);
		
	}
	
	/*public void brakeElevator() {
		elevatorBrake.set(true);
	}
	
	public void unBrakeElevator() {
		elevatorBrake.set(false);
	}
	*/
	//Command Booleans
	public void setRisen() {
		risen = true;
	}
	
	public void setNotRisen() {
		risen = false;
	}
	
	public void setUp() {
		up = true;
	}
	
	public void setNotUp() {
		up = false;
	}
	
	public void stopMovingElevator() {
		stop = true;
	}
	
	public void movingElevator() {
		stop = false;
	}
	
	public void movingSwitch() {
		movingSwitch = true;
		movingScale = false;
		movingIntake = false;
	}
	
	
	
    public void initDefaultCommand() {

    }
}

