
package frc.team3039.robot;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // r^2:0.8806194352164078 r^2:0.767598614180931
    // ks:0.9161956930322136
    // kv:0.16733812536413087
    // ka:0.044153957047831344

    // r^2:0.9698900328076605
    // r^2:0.8177253879747561
    // ks:0.8636096258554712
    // kv:0.1609164308465849
    // ka:0.046632968167755015

    // r^2:0.9915196195980441
    // r^2:0.6603735758687641
    // ks:1.0915688350353723
    // kv:0.1547622821585623
    // ka:0.04117024510627512

    // New battery 0.5 velocity
    // r^2:0.9950393018114683
    // r^2:0.711344547566271
    // ks:0.928112644250295
    // kv:0.14242500692715937
    // ka:0.03305866811140018

    // Wheels
    // 2019 Robot Values
    public static final double kDriveWheelTrackWidthInches = 22; // 22.61; wheel to wheel
    public static final double kDriveWheelDiameterInches = 6; // 3.875
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.9; // 0.924; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 59.602037; // kg TODO tune //weight
    public static final double kRobotAngularInertia = 10.0; // kg m^2 TODO tune //weight x legnth
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.928112644250295; // V
    public static final double kDriveKv = 0.10305; // 0.14242500692715937; // V per rad/s
    public static final double kDriveKa = 0.01; // 0.011505866811140018; // V per rad/s^2


    // Geometry
    // 2019 Robot Values
    public static final double kCenterToFrontBumperDistance = 19.175; 
    public static final double kCenterToRearBumperDistance = 19.175;
    public static final double kCenterToSideBumperDistance = 15.875;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    public static final double kPathKX = 4.0;// 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    public static final double kDriveVelocityKp = 0.7; // 0.9;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 3.0; // 10.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static double kDriveVelocityRampRate = 0.05; // 0.05; // 0.02
    public static double kDriveNominalOutput = 0.1;// 0.5 / 12.0;
    public static double kDriveMaxSetpoint = 11.0 * 12.0; // 11 fps


    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

}
