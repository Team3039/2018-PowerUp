
package org.usfirst.frc.team3039.robot;


import org.usfirst.frc.team3039.robot.commands.AutoCenterScale;
import org.usfirst.frc.team3039.robot.commands.AutoCenterSwitch;
import org.usfirst.frc.team3039.robot.commands.AutoForward;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSideScaleClose;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSideScaleCompatible;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSideScaleFar;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSideScaleSwitchClose;
import org.usfirst.frc.team3039.robot.commands.AutoLeftSideScaleSwitchFar;
import org.usfirst.frc.team3039.robot.commands.AutoRightSideScaleClose;
import org.usfirst.frc.team3039.robot.commands.AutoRightSideScaleCompatible;
import org.usfirst.frc.team3039.robot.commands.AutoRightSideScaleFar;
import org.usfirst.frc.team3039.robot.commands.AutoRightSideScaleSwitchClose;
import org.usfirst.frc.team3039.robot.commands.AutoRightSideScaleSwitchFar;
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
	
	//Cube Status
	public static double shootCube; //Speed we Shoot the Cube At
	public static double area;
	
	//Field Status	
	private enum robotSide{
		Left,
		Right,
		Center,
	}
	
	private enum robotAuto{
		Switch,
		Scale,
		ScaleToSwitch,
		
	}
	
	private enum autoCompatible{
		True,
		False,
		
	}
	
	static String gameInfo = "" ;

	
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

	
	//Utilizing Vision Array
	/*This is where we make everything necessary to use the vision system that we set up. 
	 * Later in the program we use these variable to control different parts of the Robot Vision to it's best ability
	 * cubeDistance determines the Distance from the Robot to the Cube which we made a double because that's how the Number Returns
	 * */
	
	//Autonomous Options
			
	SendableChooser sideSelector;
	SendableChooser autoSelector;
	SendableChooser adaptableSelector;
	
	Command AutoLeftScaleClose = new AutoLeftSideScaleClose();
	Command AutoLeftScaleFar = new AutoLeftSideScaleFar();
	Command AutoRightScaleClose = new AutoRightSideScaleClose();
	Command AutoRightScaleFar = new AutoRightSideScaleFar();
	Command AutoCenterSwitch = new AutoCenterSwitch();
	Command AutoCenterScale = new AutoCenterScale();
	Command AutoCompatibleLeft = new AutoLeftSideScaleCompatible();
	Command AutoCompatibleRight = new AutoRightSideScaleCompatible();
	Command AutoLeftScaleSwitchClose = new AutoLeftSideScaleSwitchClose();
	Command AutoLeftScaleSwitchFar = new AutoLeftSideScaleSwitchFar();
	Command AutoRightScaleSwitchClose = new AutoRightSideScaleSwitchClose();
	Command AutoRightScaleSwitchFar = new AutoRightSideScaleSwitchFar();
	Command AutoForward = new AutoForward();
	
	@SuppressWarnings({ "rawtypes", "unchecked" })
	@Override
	public void robotInit() {
	
		//Robot Setup
		oi = new OI();
		drivetrain.motorSafety(false);
		elevator.motorSafety(false);
		intake.motorSafety(false);		
		drivetrain.brake();
		elevator.torqueMode();
		drivetrain.resetEncoder();
		drivetrain.setEncoder();
		drivetrain.resetNavX();
		drivetrain.setEncoder();
		climber.disengageRatchet();
		elevator.torqueMode();
		elevator.resetElevatorEncoder();
		elevator.setUp();
		intake.resetIntakeEncoder();
		intake.intakeUp();
		
		//Auto Setup
		sideSelector = new SendableChooser();
		autoSelector = new SendableChooser();
		adaptableSelector = new SendableChooser();
		
		sideSelector.addDefault("Center", robotSide.Center);
		sideSelector.addObject("Left", robotSide.Left);
		sideSelector.addObject("Right", robotSide.Right);
		
		autoSelector.addDefault("Switch", robotAuto.Switch);
		autoSelector.addObject("Scale", robotAuto.Scale);
		autoSelector.addObject("Scale-Switch", robotAuto.ScaleToSwitch);
		
		adaptableSelector.addDefault("No", autoCompatible.False);
		adaptableSelector.addObject("Yes", autoCompatible.True);
		
		SmartDashboard.putData("Side Slecetor", sideSelector);
		SmartDashboard.putData("Auto Selector", autoSelector);
		SmartDashboard.putData("Compatible", adaptableSelector);

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
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if(gameData != null) {
			gameData = DriverStation.getInstance().getGameSpecificMessage().substring(0, 2); //From 
		}
			else {
			  System.out.println("No Game Data");
			  gameData = "";
		}
		
	switch (gameData) {
		case "LR":
			if(sideSelector.getSelected().equals(robotSide.Left)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
						autoCommand = AutoLeftScaleFar;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
						autoCommand = AutoCompatibleLeft;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
						autoCommand = AutoLeftScaleSwitchFar;
				}
			}
			else if(sideSelector.getSelected().equals(robotSide.Right)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
					autoCommand = AutoRightScaleClose;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
					autoCommand = AutoCompatibleRight;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
					autoCommand = AutoRightScaleSwitchClose;
				}
			}
			
			else if(sideSelector.getSelected().equals(robotSide.Center)) {
				if(autoSelector.getSelected().equals(robotAuto.Switch)) {
					autoCommand = AutoCenterSwitch;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale)) {
					autoCommand = AutoCenterScale;

				}
			}
			break;
			
		case "RL":
			if(sideSelector.getSelected().equals(robotSide.Left)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
						autoCommand = AutoLeftScaleClose;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
						autoCommand = AutoCompatibleLeft;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
						autoCommand = AutoLeftScaleSwitchClose;
				}
			}
			else if(sideSelector.getSelected().equals(robotSide.Right)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
					autoCommand = AutoRightScaleFar;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
					autoCommand = AutoCompatibleRight;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
					autoCommand = AutoRightScaleSwitchFar;
				}
			}
			
			else if(sideSelector.getSelected().equals(robotSide.Center)) {
				if(autoSelector.getSelected().equals(robotAuto.Switch)) {
					autoCommand = AutoCenterSwitch;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale)) {
					autoCommand = AutoCenterScale;

				}
			}
			break;
			
		case "LL":
			if(sideSelector.getSelected().equals(robotSide.Left)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
						autoCommand = AutoLeftScaleClose;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
						autoCommand = AutoCompatibleLeft;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
						autoCommand = AutoLeftScaleSwitchClose;
				}
			}
			else if(sideSelector.getSelected().equals(robotSide.Right)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
					autoCommand = AutoRightScaleFar;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
					autoCommand = AutoCompatibleRight;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
					autoCommand = AutoRightScaleSwitchFar;
				}
			}
			
			else if(sideSelector.getSelected().equals(robotSide.Center)) {
				if(autoSelector.getSelected().equals(robotAuto.Switch)) {
					autoCommand = AutoCenterSwitch;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale)) {
					autoCommand = AutoCenterScale;

				}
			}
			break;
			
		case "RR":
			if(sideSelector.getSelected().equals(robotSide.Left)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
						autoCommand = AutoLeftScaleFar;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
						autoCommand = AutoCompatibleLeft;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
						autoCommand = AutoLeftScaleSwitchFar;
				}
			}
			else if(sideSelector.getSelected().equals(robotSide.Right)) {
				if (autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.False)) {
					autoCommand = AutoRightScaleClose;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale) && adaptableSelector.getSelected().equals(autoCompatible.True)) {
					autoCommand = AutoCompatibleRight;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.ScaleToSwitch)) {
					autoCommand = AutoRightScaleSwitchClose;
				}
			}
			
			else if(sideSelector.getSelected().equals(robotSide.Center)) {
				if(autoSelector.getSelected().equals(robotAuto.Switch)) {
					autoCommand = AutoCenterSwitch;
				}
				
				if(autoSelector.getSelected().equals(robotAuto.Scale)) {
					autoCommand = AutoCenterScale;

				}
			}
			break;
			
		default:
			autoCommand = new AutoForward();
			break;
	}
	
	autoCommand.start();
	SmartDashboard.putString("Chosen Auto", autoCommand.toString());	
	
	intake.cubeIn();
}


	@Override
	public void autonomousPeriodic() {	
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

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
//		System.out.println("Elevator Encoder:" + Robot.elevator.getElevatorEncoder());
		System.out.println("Robot Angle: " + Robot.drivetrain.getAngle());
		System.out.println("Robot Distance: " + Robot.drivetrain.getDistance());
    	Robot.lights.checkLights();		

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
		return gameInfo;
	}
	
	public void setData(String newString) {
		gameInfo = newString;
	}
	    
	    }

