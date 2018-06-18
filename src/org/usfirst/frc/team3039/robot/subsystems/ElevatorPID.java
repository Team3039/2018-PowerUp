package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class ElevatorPID extends PIDSubsystem {	
	public double targetLevelIntake = 10;
	public double targetLevelPile = 550;
	public double targetLevelSwitch = 965;
	public double targetLevelScale = 2570;
	
    // Initialize your subsystem here
    public ElevatorPID() {
        super("Elevator PID", .0075, 0, 0);
    	setInputRange(0, 2600);//Feedback Range
		setOutputRange(-.8, .8);//Voltage Range
		getPIDController().setContinuous(true);
		setPercentTolerance(.1);
    }
    
    public void setElevatorPosition(double targetLevel) {
    	enable();
    	setSetpoint(targetLevel);
    	
    }

	public void intakeLevel() {
		enable();
		setSetpoint(targetLevelIntake);
	}
	
	public void switchLevel() {
		enable();
		setSetpoint(targetLevelSwitch);
	}
	
	public void scaleLevel() {
		enable();
		setSetpoint(targetLevelScale);
	}
	
	
	
	public void disablePID() {
		disable();
		Robot.elevator.stopElevator();
	} 

    protected double returnPIDInput() {
        return Robot.elevator.getElevatorEncoder();
    }

    protected void usePIDOutput(double output) {
    		Robot.elevator.elevatorMotorA.set(-output);
            Robot.elevator.elevatorMotorB.set(-output);

        System.out.println("On Target:" + onTarget());
        System.out.println("Elevator Encoder:" + Robot.elevator.getElevatorEncoder());
    }
	
    public void initDefaultCommand() {
    	Robot.elevator.getElevatorEncoder();
    	
    }
}
