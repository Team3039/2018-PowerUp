package org.usfirst.frc.team3039.robot.commands;

import org.usfirst.frc.team3039.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoTurnPIDLong extends Command {
    double angle;
    public AutoTurnPIDLong(double angle) {
        requires(Robot.drivetrain);
        this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drivetrain.motorSafety(false);
//        Robot.drivetrain.setEncoder();
//        Robot.drivetrain.resetNavX();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //System.out.println("Encoder : " + Robot.drivetrain.getDistance());
        //System.out.println("Gyro : " + Robot.drivetrain.getAngle());
        Robot.drivetrain.getAngle();
        Robot.turnpid.rotate(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.turnpid.onTarget());
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.turnpid.disablePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.turnpid.disablePID();
    }
}
