
package org.usfirst.frc.team3039.robot;

import org.usfirst.frc.team3039.robot.commands.AutoCenter;
import org.usfirst.frc.team3039.robot.commands.AutoForward;
import org.usfirst.frc.team3039.robot.commands.AutoLeft;
import org.usfirst.frc.team3039.robot.commands.AutoLeftScale;
import org.usfirst.frc.team3039.robot.commands.AutoPathfinder;
import org.usfirst.frc.team3039.robot.commands.AutoRight;
import org.usfirst.frc.team3039.robot.commands.AutoRightScale;
import org.usfirst.frc.team3039.robot.commands.AutoScaleInZone;
import org.usfirst.frc.team3039.robot.commands.AutoTest;
import org.usfirst.frc.team3039.robot.subsystems.Climber;
import org.usfirst.frc.team3039.robot.subsystems.DrivePID;
import org.usfirst.frc.team3039.robot.subsystems.DriveShortPID;
import org.usfirst.frc.team3039.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3039.robot.subsystems.Elevator;
import org.usfirst.frc.team3039.robot.subsystems.ElevatorPID;
import org.usfirst.frc.team3039.robot.subsystems.Intake;
import org.usfirst.frc.team3039.robot.subsystems.IntakePID;
import org.usfirst.frc.team3039.robot.subsystems.Lights;
import org.usfirst.frc.team3039.robot.subsystems.PIDSubsystem;
import org.usfirst.frc.team3039.robot.subsystems.RotatePID;
import org.usfirst.frc.team3039.robot.subsystems.TurnPID;
import org.usfirst.frc.team3039.robot.subsystems.VisionPID;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	public static OI oi;
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Elevator elevator = new Elevator();
	public static final Climber climber = new Climber();
	public static final Intake intake = new Intake();
	public static final Lights lights = new Lights();
	
	public static final ElevatorPID elevatorpid = new ElevatorPID();
	public static final IntakePID intakepid = new IntakePID();
	public static final RotatePID rotatepid = new RotatePID();
	public static final DrivePID drivepid = new DrivePID();
	public static final VisionPID visionpid = new VisionPID();
	public static final TurnPID turnpid = new TurnPID();
	public static final DriveShortPID movepid = new DriveShortPID();
	public static final PIDSubsystem pidcontroller = new PIDSubsystem();

	
	//Auto Setup
	Command autonomousCommand;
    SendableChooser<CommandGroup> chooser;
		
	//Robot Status
	public static boolean rachetMode; //False = Red, True = Green
	public static boolean intakeStraight;
	public static boolean intakeAngle;
	public static boolean intakeUp;
	public static double shootCube; //Speed we Shoot the Cube At
	public static double area;
	
	static String gameInfo = "   " ;
	
	//Vision Array Setup
	public static NetworkTable cubeTable = NetworkTableInstance.getDefault().getTable("GRIP/cubeReport"); //Pulling the Table From GRIP
	public static NetworkTableEntry cubeEntryX = cubeTable.getEntry("centerX"); //Pulling the Table Entry centerX for use 
    public static NetworkTableEntry cubeEntryY = cubeTable.getEntry("centerY"); //Pulling the Table Entry centerY for use
    public static NetworkTableEntry cubeEntryArea = cubeTable.getEntry("area"); //Pulling the Table Entry area for use
	public static double[] defaultValue = new double[0]; //Creating a Default Values for the Table Entry's
	public static double[] cubeArrayX; //Making centerX a Double Array
	public static double[] cubeArrayY; //Making centerY a Double Array
	public static double[] cubeArrayArea; //Making Area a Double Array
	
	
	
	//Utilizing Vision Array
	/*This is where we make everything necessary to use the vision system that we set up. 
	 * Later in the program we use these variable to control different parts of the Robot Vision to it's best ability
	 * cubeDistance determines the Distance from the Robot to the Cube which we made a double because that's how the Number Returns
	 * */
	
	public static double  cubeDistance; //How Far Away the Cube is
		
	@Override
	public void robotInit() {
		setData("");
		drivetrain.motorSafety(false);
		elevator.motorSafety(false);
		intake.motorSafety(false);		
		drivetrain.brake();
		elevator.torqueMode();
		oi = new OI();
		chooser = new SendableChooser<CommandGroup>();
		

		//Auto Chooser
		SmartDashboard.putData("Autonomous", chooser);
		chooser.addObject("Left", new AutoLeft());
		chooser.addObject("Left-Scale", new AutoLeftScale());
		chooser.addObject("Center", new AutoCenter());
		chooser.addObject("Right", new AutoRight());
		chooser.addObject("Right-Scale", new AutoRightScale());
		chooser.addDefault("Forward", new AutoForward());
		
		chooser.addObject("Test", new AutoTest());
		chooser.addObject("CAPITAL DE FREEZE", new AutoScaleInZone());

		//Camera
		//http://roborio-3039-frc.local:1181/?action=stream
		//Here we set up the camera that the robot will use to maneuver 
		UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
		UsbCamera logitechCam = CameraServer.getInstance().startAutomaticCapture();
		usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 360, 60);
		logitechCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 360, 60);
		usbCamera.setWhiteBalanceAuto();
		logitechCam.setWhiteBalanceAuto();



		//Robot Startup
		Robot.drivetrain.resetEncoder();
		Robot.drivetrain.setEncoder();
		Robot.drivetrain.resetNavX();
		Robot.climber.disengageRatchet();
		Robot.elevator.torqueMode();
		Robot.elevator.resetElevatorEncoder();
		Robot.elevator.setUp();
		Robot.intake.resetIntakeEncoder();
		intake.intakeUp();

	}
	
	@Override
	public void robotPeriodic() {
		setData(DriverStation.getInstance().getGameSpecificMessage());
		
//		System.out.println("Intake Encoder Rotation: " + Robot.intake.getIntakeEncoder());
		//System.out.println("Elevator Min" + Robot.elevator.getMin());
		//System.out.println("Elevator Max" + Robot.elevator.getMax());
		//System.out.println("Elevator Mid" + Robot.elevator.getMid());
		//System.out.println("Robot Angle" + Robot.drivetrain.getAngle());
		//System.out.println("Encoder Distance" + Robot.drivetrain.getDistance());
		

       // System.out.println("Intake Angle" + Robot.intake.getIntakeEncoder());

        


//		//Checks if Area Targets Exist
//    	double[] cubeArrayArea = cubeEntryArea.getDoubleArray(defaultValue);
//    	if(cubeArrayArea.length < 1) {
//    		area = 0;
//    	}
//    	else if(cubeArrayArea.length == 1) {
//    		area = cubeArrayArea[0];
//    	}
//    	else {
//    		area = (cubeArrayArea[0] + cubeArrayArea[1])/2;
//    	}
    	
	}
	
		
	  

	@Override
	public void disabledInit() {
		Robot.elevator.resetElevatorEncoder();
		Robot.drivetrain.resetNavX();
		Robot.drivetrain.resetEncoder();
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		Robot.drivetrain.resetNavX();
		Robot.drivetrain.resetEncoder();

	}

	@Override
	public void autonomousInit() {
		setData(DriverStation.getInstance().getGameSpecificMessage());
//		
//		if(gameInfo.charAt(0) == 'L') {
//			if
//			autonomousCommand = new AutoBlueLeft();
//
//		}
//		SmartDashboard.putBoolean("Test Bool", true);
		chooser.addObject("Left", new AutoLeft());
		chooser.addObject("Left-Scale", new AutoLeftScale());
		chooser.addObject("Center", new AutoCenter());
		chooser.addObject("Right", new AutoRight());
		chooser.addObject("Right-Scale", new AutoRightScale());
		chooser.addDefault("Forward", new AutoForward());
//		
		chooser.addObject("Test", new AutoTest());

		chooser.addObject("CAPITAL DE FREEZE", new AutoScaleInZone());



		autonomousCommand = chooser.getSelected();
		
		if (autonomousCommand != null)
			autonomousCommand.start();
		
		intake.cubeIn();
	}


	@Override
	public void autonomousPeriodic() {	
		Scheduler.getInstance().run();
		setData(DriverStation.getInstance().getGameSpecificMessage());


		autonomousCommand = chooser.getSelected();
		chooser.addObject("Left", new AutoLeft());
		chooser.addObject("Left-Scale", new AutoLeftScale());
		chooser.addObject("Center", new AutoCenter());
		chooser.addObject("Right", new AutoRight());
		chooser.addObject("Right-Scale", new AutoRightScale());
		chooser.addObject("Forward", new AutoForward());
//				
		chooser.addObject("Test", new AutoTest());
		chooser.addObject("CAPITAL DE FREEZE", new AutoScaleInZone());

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		Robot.drivetrain.resetEncoder();
		Robot.drivetrain.setEncoder();
		Robot.drivetrain.resetNavX();
		//System.out.println("Robot Switch" + Robot.intake.getSwitch());

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.getNumber("Intake Rotation", Robot.intake.getIntakeEncoder());
		//System.out.println("Elevator Encoder:" + Robot.elevator.getElevatorEncoder());
    	Robot.lights.checkLights();



//      System.out.println("Switch State" + Robot.intake.getSwitch());
		

		double cubeDistance = 57.7495 * Math.pow(Math.E, (-0.0000168466 * area));
		Robot.cubeDistance = cubeDistance;
//		System.out.println("Distance to Cube : " + cubeDistance);
//		double cubeDistanceA = 59 * Math.pow(Math.E, (-0.0000168466 * area));
//		System.out.println("Distance to Cube A : " + cubeDistanceA);
//		System.out.println("Distance to Cube B :" + cubeDistance);
//		System.out.println("ANGLE : " + Robot.drivetrain.getAngle());
//		System.out.println("Left : " + Robot.drivetrain.leftEnc.getDistance());
//		System.out.println("Right : " + Robot.drivetrain.rightEnc.getDistance());
	}


	@SuppressWarnings("deprecation")
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public static String getData() {
		return gameInfo;
	}
	
	public void setData(String newString) {
		gameInfo = newString;
	}
	    
	    }

