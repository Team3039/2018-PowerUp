/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.auto.routes;

import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team3039.auto.commands.CollectVelocityData;
import frc.team3039.auto.commands.DriveMotionCommand;
import frc.team3039.auto.commands.LazyLoadCommandGroup;
import frc.team3039.robot.paths.TrajectoryGenerator;

public class AutoTest extends LazyLoadCommandGroup {

  public AutoTest() {
    addSequential(new DriveMotionCommand(
        registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().tommysPath), true, false)); //First resets everything, Second resets X,Y
    addParallel(new WaitCommand("Break", .5));
  }

}
