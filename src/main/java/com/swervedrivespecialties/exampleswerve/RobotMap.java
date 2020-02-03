package com.swervedrivespecialties.exampleswerve;

public class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; // CAN

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 3; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 0; // CAN

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6; // CAN

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 0; // CAN

    public static final int SHOOTER_TALON = 12;
    public static final int SHOOTER_NEO  = 4; 
    public static final int KICKER_NEO = 8;
    public static final int FEEDER_TALON = 1;

    	// Logging
        // this is where the USB stick is mounted on the RoboRIO filesystem.  
        // You can confirm by logging into the RoboRIO using WinSCP
        public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
        public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
}