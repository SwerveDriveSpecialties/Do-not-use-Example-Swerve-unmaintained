package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder.MotorType;

public class DrivetrainSubsystem implements Subsystem {
    private static final double TRACKWIDTH = 21.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(357.16);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(359.11);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(238.53);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(140.59);

    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(1.5, 0.0, 0.0001);
    private static final double STANDARD_REDUCTION = 18.0 / 1.0;

    boolean isFieldOriented = true;
    double curMinControllerSpeed = .25;

    private static DrivetrainSubsystem instance;

    CANSparkMax frontLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(frontLeftSteer, Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontLeftDrive,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(frontRightSteer, Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontRightDrive, Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(backLeftSteer, Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backLeftDrive, Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(backRightSteer, Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backRightDrive, Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), new Pose2d());

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

    public void updateOdometry(){
        odometry.update(getGyroRotation(), getModuleStates());
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        updateOdometry();

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    public void toggleMinControllerSpeed(){
        if (curMinControllerSpeed < 1){
            curMinControllerSpeed += .25;
        } else {
            curMinControllerSpeed = .25;
        }
    }

    public double getMinControllerSpeed(){
        return curMinControllerSpeed;
    }

    public void resetMinControllerSpeed(){
        curMinControllerSpeed = .25;
    }

    public void setFieldOriented(boolean isFO){
        isFieldOriented = isFO;
    }

    public boolean getFieldOriented(){
        return isFieldOriented;
    }

    public void toggleFieldOriented(){
        setFieldOriented(!getFieldOriented());
    }

    public void outPutToSDB(){
        SmartDashboard.putNumber("FL", frontLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("FR", frontRightModule.getCurrentAngle());
        SmartDashboard.putNumber("BL", backLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("BR", backRightModule.getCurrentAngle());
        SmartDashboard.putNumber("Kinematic Position X", getKinematicPosition().x);
        SmartDashboard.putNumber("Kinematic Position Y", getKinematicPosition().y);
        SmartDashboard.putNumber("Kinematics Theta", getGyroAngle().toDegrees());
    }

    public Rotation2d getGyroRotation(){
        return Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees());
    }

    private SwerveModuleState getCurrentState(SwerveModule mod){
        double velo = util.inchesToMeters(mod.getCurrentVelocity());
        Rotation2d rot = Rotation2d.fromDegrees(Math.toDegrees(mod.getCurrentAngle()));
        return new SwerveModuleState(velo, rot);
    }

    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {getCurrentState(frontLeftModule),
                                        getCurrentState(frontRightModule), 
                                        getCurrentState(backLeftModule), 
                                        getCurrentState(backRightModule)};
    }

    public Vector2 getKinematicPosition(){
        Pose2d odPos = odometry.getPoseMeters();
        double x = util.metersToInches(odPos.getTranslation().getX());
        double y = util.metersToInches(odPos.getTranslation().getY());
        return new Vector2(x, y);
    }

    public Vector2 getKinematicVelocity(){
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        double x_dot = util.metersToInches(chassisSpeeds.vxMetersPerSecond);
        double y_dot = util.metersToInches(chassisSpeeds.vxMetersPerSecond);
        return new Vector2(x_dot, y_dot);
    }

    public double getGyroRate(){
        return gyroscope.getRate();
    }

    public Rotation2 getGyroAngle(){
        return gyroscope.getAngle();
    }

    public void reset(){
        odometry.resetPosition(new Pose2d(), getGyroRotation());
    }

    public void stop(){
        drive(new Translation2d(), 0.0, true);
    }

    public void updateLogData(LogDataBE logData){  
        //   logData.AddData("Forward Velocity", Double.toString(getKinematicVelocity().x));
        //   logData.AddData("Strafe Velocity", Double.toString(getKinematicVelocity().y));
        //   logData.AddData("Angular Velocity", Double.toString(gyroscope.getRate()));  
        logData.AddData("Velocity", Double.toString(getKinematicVelocity().length));
    }

    public void setCurrentLimit(int curLim){
        frontLeftDrive.setSmartCurrentLimit(curLim);
        frontRightDrive.setSmartCurrentLimit(curLim);
        backLeftDrive.setSmartCurrentLimit(curLim);
        backRightDrive.setSmartCurrentLimit(curLim);
    }
}
