/*package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class DrivePID extends PIDSubsystem {
	
	public static double targetPoint;
	public static double setpoint;
	public static int Sigma;
	public static double error = setpoint - Robot.drivetrain.getDistance();
	public static double preError = Robot.drivetrain.getRate();
	
	public static double P = 1; //Proportional Constant
	public static double I = 1; //Integral Constant
	public static double D = 1; //Derivative Constant 
	
	public static double kP; //P Formula = P/error
	public static double kI; //I Formula = I * (error * time)
	public static double kD; //D Formula = D * (error - preError) / .02 to find rate of change 	
	
	public static double tolerance = .005; //Window of Error for Driving Forward
{
	
	if(targetPoint > 100) {
		kP = P/error; //.0045		
		kI = 0;//.0004
		kD = D * (error - preError) / .02;
	}
	
	
	else {
		kP = .0045;
		kI = .0004;
		kD = 0;
	}
}

    // Initialize your subsystem here
    public DrivePID() {
        super("Distance PID", kP, kI, kD); 
        getPIDController().setContinuous(false);
        setInputRange(-1000, 1000);
        setOutputRange(-.7, .7);//voltage
        setPercentTolerance(tolerance);
        
        
    }

    public void summation(double target) {
		target = setpoint;
		int Sigma = 0;
    	int[] arr = new int[(int)target];
    	for(int i = 0; i < arr.length; i++) {
    		arr[i] = i;
    	}
    	for(int i = 0; i < arr.length; i++) {
    		Sigma += arr[i];
    		
    		DrivePID.Sigma = Sigma;

    	}
 
    }
    
    public void initDefaultCommand() {
        Robot.drivetrain.getDistance();
    }

    public void drive(double targetDistance) {
        setSetpoint(targetDistance);
        enable();
        setpoint = targetDistance;
        
    }
    
//    public void PID() {
//    	kI += (error*.02);
//    	kD = (error - preError) / .02;
//    	kP = P*error;
//    	outputv2 = kP + I*kI + D*kD;
//    }
    
    public void disablePID() {
        disable();
        Robot.drivetrain.driveStop();
    }
    protected double returnPIDInput() {
        return Robot.drivetrain.getDistance();
    }

    protected void usePIDOutput(double output) {
    	Robot.drivetrain.driveStraight(output);
//    	Robot.drivetrain.frontleftMotor.pidWrite(output);
//    	Robot.drivetrain.frontrightMotor.pidWrite(-output);
//    	Robot.drivetrain.rearleftMotor.pidWrite(output);
//    	Robot.drivetrain.rearrightMotor.pidWrite(-output);
    	    	
    	System.out.println("Rate of Change" + Robot.drivetrain.getRate());

    }
}*/

package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class DrivePID extends PIDSubsystem {
	public double targetDistance;
	
	public static double kP = .008; //.0045
	public static double kI = .00005; //.0001
	public static double kD = .1;
	
	
/*    public void setPIDValues(double target) {
    	if(targetDistance >= 200) {
        	targetDistance = target;
            setSetpoint(targetDistance);
    		kP = .008;
    		kI = 0;
    		kD = .038;            
    		enable();
    
    		
    		}
    	else if((targetDistance < 200) && (targetDistance > 100)) {
        	targetDistance = target;
            setSetpoint(targetDistance);
    		kP = .0045;
    		kI = .0001;
    		kD = .038;            
    		enable();
    	}
    	else if((targetDistance < 100) && (targetDistance > 50)) {
        	targetDistance = target;
            setSetpoint(targetDistance);
    		kP = 0;
    		kI = 0;
    		kD = 0;            
    		enable();
    	}
    	else {
        	targetDistance = target;
            setSetpoint(targetDistance);
    		kP = 0;
    		kI = 0;
    		kD = 0;            
    		enable();
    	}
    }*/


    // Initialize your subsystem here
    public DrivePID() {
        super("Distance PID", kP, kI, kD); 
        setInputRange(-1000, 1000);
        setOutputRange(-.99, .99);//voltage
        getPIDController().setContinuous(false);
        setPercentTolerance(.023);
        
      }


    public void initDefaultCommand() {
        Robot.drivetrain.getDistance();
        //setPIDValues(targetDistance);
    }
    
    public void drive(double target) {
    	targetDistance = target; 
    	setSetpoint(targetDistance);
    	enable();
    	
    }

        public void disablePID() {
        disable();
        Robot.drivetrain.driveStop();
    }
    protected double returnPIDInput() {
        return Robot.drivetrain.getDistance();
    }


    protected void usePIDOutput(double output) {
    	Robot.drivetrain.driveStraight(output);
//    	System.out.println("target: " + targetDistance);
    	//System.out.println("kP: " + kP);
    	//System.out.println("On Target: " + onTarget());
    	//System.out.println("Robot Distance" + Robot.drivetrain.getDistance());

    }
}

