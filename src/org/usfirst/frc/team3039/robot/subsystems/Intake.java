package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;
import org.usfirst.frc.team3039.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
	public boolean cubeIn;
	public boolean cubeScored;
	public boolean cubeVault;
	public boolean cubeAngle;
	public boolean cubeAuto;
	public boolean climb;
	public boolean cubeTurn;
	public boolean cubePush;
	public boolean intakeUp;
	public boolean intakeDown;

	public WPI_TalonSRX leftIntakeMotor = new WPI_TalonSRX(RobotMap.leftIntakeMotor);
	public WPI_TalonSRX rightIntakeMotor = new WPI_TalonSRX(RobotMap.rightIntakeMotor);
	public WPI_TalonSRX rotateIntakeMotor = new WPI_TalonSRX(RobotMap.rotateIntakeMotor);

	public Encoder intakeEncoder = new Encoder(RobotMap.intakeEncoderA, RobotMap.intakeEncoderB, true);

	public DigitalInput cubeSwitch = new DigitalInput(RobotMap.cubeSwitch);
	
	

	//Intake
	public void motorSafety(boolean enabled) {
		leftIntakeMotor.setSafetyEnabled(enabled);
		rightIntakeMotor.setSafetyEnabled(enabled);
		rotateIntakeMotor.setSafetyEnabled(enabled);
	}
	
		public void getCube() {
			leftIntakeMotor.set(.6);
			rightIntakeMotor.set(-.6);
		}
		
		public void shootCube() {
			leftIntakeMotor.set(-.5);
			rightIntakeMotor.set(.5);
		}
		
		public void shootCubeSlow() {
			leftIntakeMotor.set(-.3);
			rightIntakeMotor.set(.3);
		}
		public void stopCube() {
			leftIntakeMotor.set(0);
			rightIntakeMotor.set(0);
		}
		
		public void moveIntake(double power) {
			rotateIntakeMotor.pidWrite(power);
		}
		
		public void liftIntake() {
			rotateIntakeMotor.set(.5);
		}
		
		public void lowerIntake() {
			rotateIntakeMotor.set(-.5);
		}
		
		public void stopIntake() {
			rotateIntakeMotor.set(.1);
		}
		
		public void stopIntakeNoBrake() {
			rotateIntakeMotor.set(0);
		}
		
		public void setIntakeEncoder() {
			//Not Really how this is used but it sets the distance interms of revolutions i.e. 7 pulses = 1 revolution
			//Normal Distance doesn't really work because it only rotates
			//0, 325, 650
			//Up Straight, Down
			intakeEncoder.setDistancePerPulse(7);
		}
		
		public void resetIntakeEncoder() {
			intakeEncoder.reset();
		}
		
		public double getIntakeEncoder() {
			return Math.abs(intakeEncoder.getDistance());
		}
		
		public boolean getSwitch() {
			return cubeSwitch.get();
		}		

		public void cubeIn() {
			cubeIn = true;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeIn();
		
		}
		
		public void cubeOut() {
			cubeIn = false;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeOut();
		}
		
		public void cubeScored() {
			cubeIn = false;
			cubeScored = true;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeOut();

		}
		
		public void cubeIntakeVault() {
			cubeIn = true;
			cubeScored = false;
			cubeVault = true;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeIn();
			
		}
		
		public void cubeVaultFalse() {
			cubeIn = false;
			cubeScored = true;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeOut();

		}
		
		public void cubeIntakeAngle() {
			cubeIn = true;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = true;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeIn();

		}
		
		public void cubeAngleFalse() {
			cubeIn = false;
			cubeScored = true;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
			Robot.lights.cubeOut();

		}
		
		public void cubeTurn() {
			cubeIn = false;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = true;
			cubePush = false;
			Robot.lights.cubeOut();
		}
		
		public void cubeTurnFalse() {
			cubeIn = false;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;
		}
		
		public void cubePush() {
			cubeIn = true;
			cubeScored = false;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = true;
			Robot.lights.cubeIn();		
			}
		
		public void cubePushFalse() {
			cubeIn = false;
			cubeScored = true;
			cubeVault = false;
			cubeAngle = false;
			cubeTurn = false;
			cubePush = false;		
			}
		
		
		public void intakeUp() {
			intakeUp = true;
			intakeDown = false;
		}
		
		public void intakeDown() {
			intakeUp = false;
			intakeDown = true;
		}
		
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

