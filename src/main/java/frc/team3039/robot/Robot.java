
package frc.team3039.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.auto.commands.CharacterizeStraight;
import frc.team3039.auto.commands.LazyLoadCommandGroup;
import frc.team3039.auto.routes.AutoTest;
import frc.team3039.robot.loops.Looper;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.paths.TrajectoryGenerator.RightLeftAutonSide;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Elevator;
import frc.team3039.robot.subsystems.RobotStateEstimator;
import frc.team3039.robot.subsystems.Drive.DriveControlMode;
import frc.team3039.utility.lib.control.RobotStatus;

public class Robot extends TimedRobot {
  public static OI oi;
  public static Drive drive = Drive.getInstance();
  public static Elevator elevator = new Elevator();
  public static final RobotStateEstimator estimator = RobotStateEstimator.getInstance();
	public static final TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();

  //Control Loop
  public static final Looper ctrlLoop = new Looper();

  // Choosers
  private SendableChooser<OperationMode> operationModeChooser;
  private SendableChooser<Command> autonTaskChooser;
  private SendableChooser<RightLeftAutonSide> autonRightLeftChooser;
  private Command autonomousCommand;
  private Command previousAutonomousCommand;

  private RobotStatus robotstatus = RobotStatus.getInstance();

  public static enum OperationMode {
		TEST, PRACTICE, COMPETITION
	};

  public static OperationMode operationMode = OperationMode.COMPETITION;
  
  public static RightLeftAutonSide rightLeftSide = RightLeftAutonSide.RIGHT;

  public Robot(){
    super(Constants.kLooperDt* 2);
  }

  @Override
  public void robotInit() {
    oi = OI.getInstance();
    
    ctrlLoop.register(drive);
    RobotStateEstimator.getInstance().registerEnabledLoops(ctrlLoop);
    trajectoryGenerator.generateTrajectories();

    operationModeChooser = new SendableChooser<OperationMode>();
    operationModeChooser.addOption("Practice", OperationMode.PRACTICE);
    operationModeChooser.setDefaultOption("Competition", OperationMode.COMPETITION);
    operationModeChooser.addOption("Test", OperationMode.TEST);
    SmartDashboard.putData("Operation Mode", operationModeChooser);

    autonTaskChooser = new SendableChooser<Command>();

    autonTaskChooser.setDefaultOption("None", null);

    autonTaskChooser.addOption("Test", new AutoTest());

    autonTaskChooser.addOption("V/A Test", new CharacterizeStraight());

    SmartDashboard.putData("Autonomous", autonTaskChooser);

    autonRightLeftChooser = new SendableChooser<RightLeftAutonSide>();
    autonRightLeftChooser.addOption("Left", RightLeftAutonSide.LEFT);
    autonRightLeftChooser.setDefaultOption("Right", RightLeftAutonSide.RIGHT);
    SmartDashboard.putData("Auton Side", autonRightLeftChooser);

    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();

  }


  @Override
  public void robotPeriodic() {
    updateStatus();
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

    autonomousCommand = autonTaskChooser.getSelected();
    if (autonomousCommand != previousAutonomousCommand) {
      if (autonomousCommand != null && autonomousCommand instanceof LazyLoadCommandGroup) {
        LazyLoadCommandGroup lazyLoad = (LazyLoadCommandGroup) autonomousCommand;
        System.out.println("Activate auton");
        double startTime = Timer.getFPGATimestamp();
        lazyLoad.activate();
        System.out.println("Activate auton complete t = " + (Timer.getFPGATimestamp()
        - startTime) + " sec");
        previousAutonomousCommand = autonomousCommand;
      }
    }
  }

  @Override
  public void autonomousInit() {
    ctrlLoop.start();

    rightLeftSide = autonRightLeftChooser.getSelected();
    trajectoryGenerator.setRightLeftAutonSide(rightLeftSide);

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    operationMode = operationModeChooser.getSelected();
    drive.setControlMode(DriveControlMode.JOYSTICK);
    ctrlLoop.start();

    if (operationMode == OperationMode.COMPETITION) {

    }

    if (operationMode != OperationMode.COMPETITION) {
      // Scheduler.getInstance().add(new ElevatorAutoZeroSensor());
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }

  public Alliance getAlliance() {
    return m_ds.getAlliance();
  }

  public double getMatchTime() {
    return m_ds.getMatchTime();
  }

  public void updateStatus() {
    drive.updateStatus(operationMode);
    robotstatus.updateStatus(operationMode);
  }
}
