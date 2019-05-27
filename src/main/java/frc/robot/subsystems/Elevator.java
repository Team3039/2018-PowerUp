package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 *
 */
public class Elevator extends Subsystem {	

	public TalonSRX elevatorMotorA = new TalonSRX(RobotMap.elevatorMotorA);
	public TalonSRX elevatorMotorB = new TalonSRX(RobotMap.elevatorMotorB);
	
	
	public DigitalInput maxLimit = new DigitalInput(RobotMap.maxLevel);
	public DigitalInput midLimit = new DigitalInput(RobotMap.midLevel);
	public DigitalInput minLimit = new DigitalInput(RobotMap.minLevel);

	public void liftElevator() {
			elevatorMotorA.set(ControlMode.PercentOutput,.85);
			elevatorMotorB.set(ControlMode.PercentOutput,.85);
	}
	
	public void lowerElevator() {
		elevatorMotorA.set(ControlMode.PercentOutput,-.5);
		elevatorMotorB.set(ControlMode.PercentOutput,-.5);
	}

	public void stopElevator() {
		elevatorMotorA.set(ControlMode.PercentOutput,0);
		elevatorMotorB.set(ControlMode.PercentOutput,0);
	}
	
	public void setup() {
		elevatorMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		elevatorMotorA.config_kP(0, 0);
		elevatorMotorA.config_kI(0, 0);
		elevatorMotorA.config_kD(0, 0); 
		elevatorMotorA.config_kF(0, 0);
	}

	public void setElevator(double target) {
		elevatorMotorA.set(ControlMode.MotionMagic, target);
		elevatorMotorB.follow(elevatorMotorA);
	}

	public double getPosition() {
		return elevatorMotorA.getSelectedSensorPosition();
	}

	public void unBrake() {
		elevatorMotorA.setNeutralMode(NeutralMode.Coast);
		elevatorMotorB.setNeutralMode(NeutralMode.Coast);
	}
	
	public void brake() {
		elevatorMotorA.setNeutralMode(NeutralMode.Brake);
		elevatorMotorB.setNeutralMode(NeutralMode.Brake);
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
	
		
    public void initDefaultCommand() {

    }
}

