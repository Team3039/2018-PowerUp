
package frc.team3039.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.loops.Looper;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Elevator;
import frc.team3039.robot.subsystems.RobotStateEstimator;
import frc.team3039.utility.lib.control.RobotStatus;

public class Robot extends TimedRobot {
  public static OI oi;
  public static Drive drive = Drive.getInstance();
  public static Elevator elevator = new Elevator();
  public static final RobotStateEstimator estimator = RobotStateEstimator.getInstance();
	public static final TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();

  Command selectedCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
	private SendableChooser<OperationMode> operationModeChooser;

  private RobotStatus robotstatus = RobotStatus.getInstance();
  public static final Looper ctrlLoop = new Looper();

  public static enum OperationMode {
		TEST, PRACTICE, COMPETITION
	};

	public static OperationMode operationMode = OperationMode.COMPETITION;

  @Override
  public void robotInit() {
    oi = new OI();
    SmartDashboard.putData("Auto mode", autoChooser);
    ctrlLoop.register(drive);
    RobotStateEstimator.getInstance().registerEnabledLoops(ctrlLoop);
  }


  @Override
  public void robotPeriodic() {
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    selectedCommand = autoChooser.getSelected();
    if (selectedCommand != null) {
      selectedCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (selectedCommand != null) {
      selectedCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }

  public void updateStatus() {
    drive.updateStatus(operationMode);
    robotstatus.updateStatus(operationMode);
  }
}
