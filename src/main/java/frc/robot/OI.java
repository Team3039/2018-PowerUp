/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.controllers.PS4Controller;
import frc.robot.commands.LowerElevator;
import frc.robot.commands.RaiseElevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//Calls the Gamepad Classes: Defines gp and cp for the robot
	private PS4Controller gp = new PS4Controller(RobotMap.pilot);
	private PS4Controller cp = new PS4Controller(RobotMap.coPilot);
	
	//Returns Controller Data for use with certain Methods
	public PS4Controller getGamepad() {
		return gp;
	}
	
	public PS4Controller getCopad() {
		return cp;
	}
	
	//Creates a Button Object from the Controllers
	//Pilot
	Button buttonTriangle = gp.getButtonTriangle();
	Button buttonSquare = gp.getButtonSquare();
	Button buttonCircle = gp.getButtonCircle();
	Button buttonX = gp.getButtonX();
	Button buttonOptions = gp.getOptionsButton();
	Button buttonShare = gp.getShareButton();
	Button buttonStart = gp.getStartButton();
	Button buttonPad = gp.getButtonPad();
	Button L1 = gp.getL1();
	Button R1 = gp.getR1();
	Button L2 = gp.getL2();
	Button R2 = gp.getR2();
	Button L3 = gp.getL3();
	Button R3 = gp.getR3();
	
	
	//CoPilot
	Button cobuttonTriangle = cp.getButtonTriangle();
	Button cobuttonSquare = cp.getButtonSquare();
	Button cobuttonCircle = cp.getButtonCircle();
	Button cobuttonX = cp.getButtonX();
	Button cobuttonOptions = cp.getOptionsButton();
	Button cobuttonShare = cp.getShareButton();
	Button cobuttonPad = cp.getButtonPad();
	Button coL1 = cp.getL1();
	Button coR1 = cp.getR1();
	Button coL2 = cp.getL2();
	Button coR2 = cp.getR2();
	Button coL3 = cp.getL3();
	Button coR3 = cp.getR3();
	
	public OI() {
		
//Pilot Controls
		
		R1.whileHeld(new RaiseElevator());
		L1.whileHeld(new LowerElevator());

//CoPilot Controls

	}
}
