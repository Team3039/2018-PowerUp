
package org.usfirst.frc.team3039.robot;


import org.usfirst.frc.team3039.robot.commands.AutoForward;
import org.usfirst.frc.team3039.robot.commands.AutoLeftScale;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSwitch;
import org.usfirst.frc.team3039.robot.commands.AutoRightScale;
import org.usfirst.frc.team3039.robot.commands.AutoRightSwitch;
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
	Command autoCommand;
    SendableChooser<CommandGroup> chooser;
	
	//Robot Status
	public static boolean rachetMode; //False = Red, True = Green
	public static boolean intakeStraight;
	public static boolean intakeAngle;
	public static boolean intakeUp;
	public static boolean alliance;
	
	//Cube Status
	public static double shootCube; //Speed we Shoot the Cube At
	public static double area;
	
	//Field Status
	public static String gameData = "" ;
	
	//Vision Array Setup
	public static NetworkTable cubeTable = NetworkTableInstance.getDefault().getTable("GRIP/cubeReport"); //Pulling the Table From GRIP
	public static NetworkTableEntry cubeEntryX = cubeTable.getEntry("centerX"); //Pulling the Table Entry centerX for use 
    public static NetworkTableEntry cubeEntryY = cubeTable.getEntry("centerY"); //Pulling the Table Entry centerY for use
    public static NetworkTableEntry cubeEntryArea = cubeTable.getEntry("area"); //Pulling the Table Entry area for use
	public static double[] defaultValue = new double[0]; //Creating a Default Values for the Table Entry's
	public static double[] cubeArrayX; //Making centerX a Double Array
	public static double[] cubeArrayY; //Making centerY a Double Array
	public static double[] cubeArrayArea; //Making Area a Double Array
	public static double  cubeDistance; //How Far Away the Cube is
	

	
	@Override
	public void robotInit() {
	
		//Robot Setup
		oi = new OI();
		drivetrain.motorSafety(false);
		elevator.motorSafety(false);
		intake.motorSafety(false);		
		drivetrain.brake();
		drivetrain.resetEncoder();
		drivetrain.setEncoder();
		drivetrain.resetNavX();
		drivetrain.setEncoder();
		elevator.resetElevatorEncoder();
		elevator.setUp();
		intake.resetIntakeEncoder();
		intake.intakeUp();
		
		//Auto Chooser
		chooser = new SendableChooser<CommandGroup>();		
		chooser.addDefault("Forward", new AutoForward());
		chooser.addObject("RIGHT Scale", new AutoRightScale());
		chooser.addObject("RIGHT Switch", new AutoRightSwitch());
		chooser.addObject("LEFT Scale", new AutoLeftScale());
		chooser.addObject("LEFT Switch", new AutoLeftSwitch());
		chooser.addObject("TEST", new AutoTest());
		SmartDashboard.putData("Auto Selector", chooser);

		//Camera Setup
		//http://roborio-3039-frc.local:1181/?action=stream
		//Here we set up the camera that the robot will use to maneuver 
		UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
		UsbCamera logitechCam = CameraServer.getInstance().startAutomaticCapture();
		usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 360, 60);
		logitechCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 360, 60);
		usbCamera.setWhiteBalanceAuto();
		logitechCam.setWhiteBalanceAuto();

	}
	
	@Override
	public void robotPeriodic() {
		setData(DriverStation.getInstance().getGameSpecificMessage());
        autoCommand = (Command) chooser.getSelected();

		//Checks if Area Targets Exist
    	double[] cubeArrayArea = cubeEntryArea.getDoubleArray(defaultValue);
    	if(cubeArrayArea.length < 1) {
    		area = 0;
    	}
    	else if(cubeArrayArea.length == 1) {
    		area = cubeArrayArea[0];
    	}
    	else {
    		area = (cubeArrayArea[0] + cubeArrayArea[1])/2;
    	}
    	
    	alliance = SmartDashboard.getBoolean("Alliance", false);

	}
	
	
	
	@Override
	public void disabledInit() {
//		Robot.intake.resetIntakeEncoder(); //Messes up when Auto Switches to TeleOp
		Robot.elevator.resetElevatorEncoder();
		Robot.drivetrain.resetNavX();
		Robot.drivetrain.resetEncoder();
		Robot.lights.swapAlliance(alliance);

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		Robot.drivetrain.resetNavX();
		Robot.drivetrain.resetEncoder();

	}

	@Override
	public void autonomousInit() {
		//Auto Chooser
		chooser = new SendableChooser<CommandGroup>();		
		chooser.addDefault("Forward", new AutoForward());
		chooser.addObject("RIGHT Scale", new AutoRightScale());
		chooser.addObject("RIGHT Switch", new AutoRightSwitch());
		chooser.addObject("LEFT Scale", new AutoLeftScale());
		chooser.addObject("LEFT Switch", new AutoLeftSwitch());
		chooser.addObject("TEST", new AutoTest());
		SmartDashboard.putData("Auto Selector", chooser);
        autoCommand = (Command) chooser.getSelected();

		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if(gameData != null) {
			gameData = DriverStation.getInstance().getGameSpecificMessage().substring(0, 2); 
		}
		else {
			  System.out.println("No Game Data");
			  gameData = "";
		}
		

	
	autoCommand.start();
	SmartDashboard.putString("Chosen Auto", autoCommand.toString());	
	
	intake.cubeIn();
	}	


	@Override
	public void autonomousPeriodic() {
		//Auto Chooser
		chooser = new SendableChooser<CommandGroup>();		
		chooser.addDefault("Forward", new AutoForward());
		chooser.addObject("RIGHT Scale", new AutoRightScale());
		chooser.addObject("RIGHT Switch", new AutoRightSwitch());
		chooser.addObject("LEFT Scale", new AutoLeftScale());
		chooser.addObject("LEFT Switch", new AutoLeftSwitch());
		chooser.addObject("TEST", new AutoTest());
		SmartDashboard.putData("Auto Selector", chooser);
        autoCommand = (Command) chooser.getSelected();
		Scheduler.getInstance().run();
		setData(DriverStation.getInstance().getGameSpecificMessage());


	}

	@Override
	public void teleopInit() {
		if (autoCommand != null)
			autoCommand.cancel();
		Robot.drivetrain.resetEncoder();
		Robot.drivetrain.setEncoder();
		Robot.drivetrain.resetNavX();
		Robot.intakepid.disable();
		Robot.lights.swapAlliance(alliance);

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		System.out.println("Intake Encoder:" + Robot.intake.getIntakeEncoder());
//		System.out.println("Robot Angle: " + Robot.drivetrain.getAngle());
//		System.out.println("Robot Distance: " + Robot.drivetrain.getDistance());

		double cubeDistance = 57.7495 * Math.pow(Math.E, (-0.0000168466 * area));
		Robot.cubeDistance = cubeDistance;
//		System.out.println("Distance to Cube : " + cubeDistance);

//		double cubeDistanceA = 59 * Math.pow(Math.E, (-0.0000168466 * area));
//		System.out.println("Distance to Cube A : " + cubeDistanceA);

		System.out.println("Robot Angle" + Robot.drivetrain.getAngle());
	}


	@SuppressWarnings("deprecation")
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public static String getData() {
		return gameData;
	}
	
	public void setData(String newString) {
		gameData = newString;
	}
	    
	    }

