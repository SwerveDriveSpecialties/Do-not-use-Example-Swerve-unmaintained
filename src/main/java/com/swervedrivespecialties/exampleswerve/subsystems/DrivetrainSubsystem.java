package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;

public class DrivetrainSubsystem extends Subsystem implements Updatable {
    private static final double TRACKWIDTH = 21.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(357.16);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(359.11);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(238.53);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(140.59);

    private boolean _isFieldOriented = true; //When the robot starts up, the drivetrain is field oriented.
    private double curMinSpeed = .25;

    private static DrivetrainSubsystem instance;

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR))
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new Spark(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR))
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new Spark(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR))
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new Spark(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR))
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    private final SwerveKinematics kinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    // Logging stuff
    private NetworkTableEntry poseXEntry;
    private NetworkTableEntry poseYEntry;
    private NetworkTableEntry poseAngleEntry;

    private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

//     private final Object sensorLock = new Object();
//     @GuardedBy("sensorLock")
    private final NavX navX = new NavX(SPI.Port.kMXP);

//     private final Object kinematicsLock = new Object();
//     @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;

//     private final Object stateLock = new Object();
//     @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

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

    public RigidTransform2 getPose() {
        return pose;
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    @Override
    public void update(double timestamp, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        driveSignal = this.driveSignal;

        updateModules(driveSignal, dt);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        angle = navX.getAngle();

        RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

        this.pose = pose;
    }

    private void updateModules(HolonomicDriveSignal signal, double dt) {
        ChassisVelocity velocity;
        if (signal == null) {
            velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (signal.isFieldOriented()) {
            velocity = new ChassisVelocity(
                    signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    signal.getRotation()
            );
        } else {
            velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
        }

        Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    @Override
    public void periodic() {
        // var pose = getPose();
        // poseXEntry.setDouble(pose.translation.x);
        // poseYEntry.setDouble(pose.translation.y);
        // poseAngleEntry.setDouble(pose.rotation.toDegrees());

        // for (int i = 0; i < modules.length; i++) {
        //     var module = modules[i];
        //     moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
        // }
    }


    public void toggleFieldOriented(){
        _isFieldOriented = !_isFieldOriented;
    }

    public boolean getFieldOriented(){
        return _isFieldOriented;
    }

    public void toggleMinSpeed(){
        if (curMinSpeed == 1.){
            curMinSpeed = .25;
        } else {
            curMinSpeed += .25;
        }
    }

    public double getMinSpeed(){
        return curMinSpeed;
    }

    public Rotation2 getGyro(){
        return gyroscope.getAngle();
    }

    public double getGyroRate(){
        return gyroscope.getRate();
    }

    public Vector2 getKinematicPosition(){
        return this.getPose().translation;
    }

    public Vector2 getKinematicVelocity(){
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();
            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }
        return kinematics.toChassisVelocity(moduleVelocities).getTranslationalVelocity();
    }

    public void reset(){
        odometry.resetPose(RigidTransform2.ZERO);
    }

    public void stop(){
        drive(Vector2.ZERO, 0, false);
    }
}
