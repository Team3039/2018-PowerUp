/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.team3039.controllers.GameController;
import frc.team3039.controllers.Playstation;
import frc.team3039.robot.commands.MoveBackward;
import frc.team3039.robot.commands.MoveFoward;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//Calls the Gamepad Classes: Defines gp and cp for the robot
	// private PS4Controller gp = new PS4Controller(RobotMap.pilot);
	// private PS4Controller cp = new PS4Controller(RobotMap.coPilot);

	private GameController m_driver;
	private GameController m_operator;
	
	// //Returns Controller Data for use with certain Methods
	// public PS4Controller getGamepad() {
	// 	return gp;
	// }
	
	// public PS4Controller getCopad() {
	// 	return cp;
	// }
	

	private static OI instance;

	public static OI getInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}
	//Creates a Button Object from the Controllers
	//Pilot
	// Button buttonTriangle = gp.getButtonTriangle();
	// Button buttonSquare = gp.getButtonSquare();
	// Button buttonCircle = gp.getButtonCircle();
	// Button buttonX = gp.getButtonX();
	// Button buttonOptions = gp.getOptionsButton();
	// Button buttonShare = gp.getShareButton();
	// Button buttonStart = gp.getStartButton();
	// Button buttonPad = gp.getButtonPad();
	// Button L1 = gp.getL1();
	// Button R1 = gp.getR1();
	// Button L2 = gp.getL2();
	// Button R2 = gp.getR2();
	// Button L3 = gp.getL3();
	// Button R3 = gp.getR3();
	
	
	// //CoPilot
	// Button cobuttonTriangle = cp.getButtonTriangle();
	// Button cobuttonSquare = cp.getButtonSquare();
	// Button cobuttonCircle = cp.getButtonCircle();
	// Button cobuttonX = cp.getButtonX();
	// Button cobuttonOptions = cp.getOptionsButton();
	// Button cobuttonShare = cp.getShareButton();
	// Button cobuttonPad = cp.getButtonPad();
	// Button coL1 = cp.getL1();
	// Button coR1 = cp.getR1();
	// Button coL2 = cp.getL2();
	// Button coR2 = cp.getR2();
	// Button coL3 = cp.getL3();
	// Button coR3 = cp.getR3();
	
	private OI() {

		m_driver = new GameController(RobotMap.pilot, new Playstation());
		m_operator = new GameController(RobotMap.coPilot, new Playstation());

		Button moveForawrd = m_driver.getButtonTriangle();
		moveForawrd.whileHeld(new MoveFoward());

		Button moveBackward = m_driver.getButtonX();
		moveBackward.whileHeld(new MoveBackward());
		
//Pilot Controls
		
		// buttonSquare.toggleWhenPressed(new GetCube());
		// buttonX.toggleWhenPressed(new SetIntakeUp());
		// buttonTriangle.toggleWhenPressed(new SetIntakeAngle());

		// L2.whileHeld(new LiftIntake());
		// R2.whileHeld(new LowerIntake());
		
		// R1.toggleWhenPressed(new ShootCube());
		// L1.toggleWhenPressed(new ShootCubeSlow());
		
		// buttonPad.toggleWhenPressed(new SetIntakePosition());
		// buttonOptions.whileHeld(new ResetIntakeEncoder());
		
		// R1.whileHeld(new ShootCube());

//CoPilot Controls
		// coL3.whileHeld(new LowerElevatorClimb());
		// coR3.whileHeld(new RaiseElevatorClimb());
		// coL2.whileHeld(new LowerElevator());
		// coR2.whileHeld(new LiftElevator());
		// coR1.whileHeld(new ShootCube());
		// coL1.whileHeld(new ShootCubeSlow());
		// cobuttonX.whenPressed(new RunElevatorIntake());
		// cobuttonSquare.toggleWhenPressed(new DropForks());
		// cobuttonTriangle.whileHeld(new Liftoff());
		// cobuttonCircle.whenPressed(new RunPreClimb());
		// cobuttonOptions.whenPressed(new StopElevator());
  }

  	public GameController getDriverController() {
		return m_driver;
	}

	public GameController getOperatorController() {
		return m_operator;
	}
}
