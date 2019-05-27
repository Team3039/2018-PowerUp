
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
  public static OI oi;
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();

  Command selectedCommand;
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    oi = new OI();
    SmartDashboard.putData("Auto mode", autoChooser);
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
}
