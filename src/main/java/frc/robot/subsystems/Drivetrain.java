package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controllers.PS4Controller;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;

/**
 *
 */
public class Drivetrain extends Subsystem {
//Setup    
    //Drive Motors
    public TalonSRX frontleftMotor = new TalonSRX(RobotMap.frontleftMotor); 
    public TalonSRX frontrightMotor = new TalonSRX(RobotMap.frontrightMotor);
    public TalonSRX rearleftMotor = new TalonSRX(RobotMap.rearleftMotor);
    public TalonSRX rearrightMotor = new TalonSRX(RobotMap.rearrightMotor);
    
    //Gyro
    public AHRS navX = new AHRS(SPI.Port.kMXP);
    
//Methods    
    public void driveTeleOp(PS4Controller gp) {
        unBrake();
        double y = gp.getLeftYAxis()*-.8;
        double rot = gp.getRightXAxis()*.4;

        if(Math.abs(y) < .05) {
            y = 0;
        }
    //Calculated Outputs (Limits Output to 12V)
        double leftOutput = y + rot;
        double rightOutput = rot - y;


    //Set Motor's Neutral/Idle Mode to Brake
        frontleftMotor.setNeutralMode(NeutralMode.Brake);
        frontrightMotor.setNeutralMode(NeutralMode.Brake);
        rearrightMotor.setNeutralMode(NeutralMode.Brake);
        rearleftMotor.setNeutralMode(NeutralMode.Brake);

    //Assigns Each Motor's Power
        frontleftMotor.set(ControlMode.PercentOutput, leftOutput);
        frontrightMotor.set(ControlMode.PercentOutput, rightOutput);
        rearleftMotor.follow(frontleftMotor);
        rearrightMotor.follow(frontrightMotor);               
    }
    
    public void driveStop() {
        frontleftMotor.set(ControlMode.PercentOutput, 0);
        frontrightMotor.set(ControlMode.PercentOutput, 0);
        rearleftMotor.follow(frontleftMotor);
        rearrightMotor.follow(frontrightMotor);  
    }

    public void unBrake() {
        frontleftMotor.setNeutralMode(NeutralMode.Coast);
        frontrightMotor.setNeutralMode(NeutralMode.Coast);
        rearrightMotor.setNeutralMode(NeutralMode.Coast);
        rearleftMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    public void brake() {
        frontleftMotor.setNeutralMode(NeutralMode.Brake);
        frontrightMotor.setNeutralMode(NeutralMode.Brake);
        rearrightMotor.setNeutralMode(NeutralMode.Brake);
        rearleftMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    //Encoder 
    public void setEncoder() {
        //360 Pulses per Revolution
        //6" Wheel has a Circumference of 18.85"
    	//18.5/360 = x/1
        frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }
    
    public double getLeftDistance() {
        return frontleftMotor.getSelectedSensorPosition();
    }

    public double getRightDistance() {
        return frontrightMotor.getSelectedSensorPosition();      
    }
    
    public double getLeftVelocity() {
    	return frontleftMotor.getSelectedSensorVelocity();
    }
    public double getRightVelocity() {
        return frontrightMotor.getSelectedSensorVelocity();
    }

    public void resetEncoder() {
        frontleftMotor.setSelectedSensorPosition(0);
        frontrightMotor.setSelectedSensorPosition(0);
    }
    
    //NavX
    public double getAngle() {
        return navX.getAngle();
    }
    
    public void resetNavX() {
        navX.reset();
    }
    
    //Default Command: TeleOp Driving
    public void initDefaultCommand() {
        setDefaultCommand(new TeleOpDrive());
    }
}
