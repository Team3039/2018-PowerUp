package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.RobotMap;
import org.usfirst.frc.team3039.robot.commands.DriveTeleOp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.team3039.util.PS4Gamepad;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class Drivetrain extends Subsystem {
//Setup    
    //Drive Motors
    public WPI_TalonSRX frontleftMotor = new WPI_TalonSRX(RobotMap.frontleftMotor); 
    public WPI_TalonSRX frontrightMotor = new WPI_TalonSRX(RobotMap.frontrightMotor);
    public WPI_TalonSRX rearleftMotor = new WPI_TalonSRX(RobotMap.rearleftMotor);
    public WPI_TalonSRX rearrightMotor = new WPI_TalonSRX(RobotMap.rearrightMotor);

    //Drivetrain Sides
    public SpeedControllerGroup leftDrivetrain = new SpeedControllerGroup(frontleftMotor, rearleftMotor);
    public SpeedControllerGroup rightDrivetrain = new SpeedControllerGroup(frontrightMotor, rearrightMotor);

    //Full Drivetrain
    public DifferentialDrive drivetrain = new DifferentialDrive(leftDrivetrain, rightDrivetrain);
    
    //Driving Encoder
    public Encoder leftEnc = new Encoder(RobotMap.leftEncoderA, RobotMap.leftEncoderB, false);
    public Encoder rightEnc = new Encoder(RobotMap.rightEncoderA, RobotMap.rightEncoderB, false);
    
    //Gyro
    public AHRS navX = new AHRS(SPI.Port.kMXP);
    
//Methods    
    public void motorSafety(boolean enabled) {
        frontleftMotor.setSafetyEnabled(enabled);
        frontrightMotor.setSafetyEnabled(enabled);
        rearleftMotor.setSafetyEnabled(enabled);
        rearrightMotor.setSafetyEnabled(enabled);
    }
    //Drive Control 
    public void driveTeleOp(PS4Gamepad gp) {
    //Tele-Op Driving
        drivetrain.arcadeDrive(gp.getLeftYAxis() * -1, gp.getRightXAxis() * .6);        
    }
    
    public void driveCurve(double power, double angle) {
        //Auto Turning    
        getAngle();
        drivetrain.curvatureDrive(power, angle, false);
        System.out.println("Angle : " + getAngle());
        }
//Pathfinder (PF)
    //Create Trajectory
    Waypoint[] points = new Waypoint[] {
    		//Distances are in Meters
    	    new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
    	    new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
    	    new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    	};

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);
    TankModifier modifier = new TankModifier(trajectory).modify(0.5588); //22" Drivetrain Width
    //Create EncoderFollower
	public EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
	public EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
	
    public void setupPF() {
    	left.configurePIDVA(1.0, 0.0, 0.0, 1 / .9, 0);

    	left.configureEncoder((int)leftEnc.get(), 1440, 0.1524);
    	right.configureEncoder((int)rightEnc.get(), 1440, 0.1524);
    	
    }
    
    public void runPF() {
    	double l = left.calculate((int)(leftEnc.get()));
    	double r = right.calculate((int)(rightEnc.get()));

    	double gyro_heading = getAngle();  
    	double desired_heading = Pathfinder.r2d(left.getHeading()); 

    	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    	double turn = 0.8 * (-1.0/80.0) * angleDifference;
    	
//    	frontleftMotor.set(ControlMode.PercentOutput, l + turn); //https://github.com/Team254/FRC-2017-Public.git
//    	rearleftMotor.set(ControlMode.PercentOutput, l + turn);
//    	frontrightMotor.set(ControlMode.PercentOutput, r - turn);
//    	frontrightMotor.set(ControlMode.PercentOutput, r - turn);



    }
    
    public void driveStraight(double power) {
    //Auto Driving
            if(power < 0) {
                drivetrain.arcadeDrive(power, 0);
            }            
            else {
                double p = 0.06;
                drivetrain.arcadeDrive(power, (0 - getAngle()) * p);
            }
    }
    
    public void rotate(double power) {
        frontleftMotor.set(power);
        frontrightMotor.set(power);
        rearleftMotor.set(power);
        rearrightMotor.set(power);
    }
    
    public void driveStop() {
        drivetrain.stopMotor();
    }
    
    public void brakeDrive(double power) {
        for(int i = 4750; i >= 0; i --) { //Changed from 5
            drivetrain.arcadeDrive(-power, 0);
        }
        driveStop();
    }
    
    public void brakeTurn(double power) {
        for(int i = 800; i >= 0; i--) {
            frontleftMotor.set(-power);
            frontrightMotor.set(-power);
            rearleftMotor.set(-power);
            rearrightMotor.set(power);
        }
        driveStop();
    }
    
    public void driveForward() {
        frontleftMotor.set(.3);
        frontrightMotor.set(-.3);
        rearleftMotor.set(.3);
        rearrightMotor.set(-.3);
    }
    
    public void driveBackward() {
        frontleftMotor.set(-.4);
        frontrightMotor.set(.4);
        rearleftMotor.set(-.4);
        rearrightMotor.set(.4);
    }
    
    public void curveLeft(){
        frontrightMotor.set(-.7);
        frontleftMotor.set(.0);
        rearrightMotor.set(-.7);
        rearleftMotor.set(.0);
        
    }
    
    public void curveRight() {
        frontrightMotor.set(.0);
        frontleftMotor.set(.7);
        rearrightMotor.set(.0);
        rearleftMotor.set(.7);
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
        leftEnc.setDistancePerPulse(0.05235987755983);
        rightEnc.setDistancePerPulse(0.05235987755983);
    }
    
    public double getDistance() {
        return leftEnc.getDistance();
    }
    
    public double getRate() {
    	return leftEnc.getRate();
    }
    
    public void resetEncoder() {
        leftEnc.reset();
        rightEnc.reset();
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
        setDefaultCommand(new DriveTeleOp());
    }
}
