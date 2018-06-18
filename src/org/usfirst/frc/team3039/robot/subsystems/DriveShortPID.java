
package org.usfirst.frc.team3039.robot.subsystems;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class DriveShortPID extends PIDSubsystem {
	public double targetDistance;
	
	public static double kP = .029; //.0045
	public static double kI = .00005; //.0001
	public static double kD = .044;
	
	
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
    public DriveShortPID() {
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
    	System.out.println("On Target: " + onTarget());
    	System.out.println("Robot Distance" + Robot.drivetrain.getDistance());

    }
}

