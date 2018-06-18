package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PIDSubsystem extends Subsystem {

    private double error;
    private double lastError;
    private double position = Robot.drivetrain.getDistance();

    private double integral;
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private final double DT = 0.02; // In seconds so 20ms
    
    public double power;
    public double setpoint;

    public void PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }

    public double PID(double target) {
    	setpoint = target;
        error = target - position;
        integral += error * DT;
        double output = (kP * error) + (kI * integral) + (kD * (error - lastError) / DT) + kF;

        lastError = error;
        power = output;

        return output;
  } 

    public void resetController() {
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }

    public void updateConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getError() {
        return lastError;
    }


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

