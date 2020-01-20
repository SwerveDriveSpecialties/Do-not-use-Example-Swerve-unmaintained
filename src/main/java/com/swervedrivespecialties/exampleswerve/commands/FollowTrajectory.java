package com.swervedrivespecialties.exampleswerve.commands;

import java.util.function.Supplier;
import java.util.Optional;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase {
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

  ///////////// PATH FOLLOWING CONSTANTS //////////////////
  private static final double kMaxVelo = 12 * 12; //This is the physical max velocity of the machine, not of any path
  private static final double kInterceptVoltage = .03; //the physical minimum voltage to make the robot move forward
  private static final double kPathFollowingAccelFeedForward = 0;
  private static final double kPathFollowingTranslationP = .05;
  private static final double kPathFollowingTranslationI = 0;
  private static final double kPathFollowingTranslationD = 0;
  private static final double kPathFollowingRotationP = .0085;
  private static final double kPathFollowingRotationI = 0;
  private static final double kPathFollowingRotationD = .00025;
  ////////////////////////////////////////////////////////////

  //create appropriate constant classes and eventually a TrajectoryFollower from constants given above
  PidConstants translationConstants = new PidConstants(kPathFollowingTranslationP, kPathFollowingTranslationI, kPathFollowingTranslationD);
  PidConstants rotationConstants = new PidConstants(kPathFollowingRotationP, kPathFollowingRotationI, kPathFollowingRotationD);
  double kFeedForwardVelocity = (1 - kInterceptVoltage) / kMaxVelo;
  DrivetrainFeedforwardConstants feedforwardConstants = new DrivetrainFeedforwardConstants(kFeedForwardVelocity, kPathFollowingAccelFeedForward, kInterceptVoltage);
  HolonomicFeedforward feedforward = new HolonomicFeedforward(feedforwardConstants); //can have separate forward and strafe feed forwards if desired. 
  HolonomicMotionProfiledTrajectoryFollower follower;

  private Trajectory trajectory;
  private Supplier<Trajectory> trajectorySupplier;
  private InertiaGain iGain;

  double timestamp;
  double lastTimestamp;
  double dt;


  public FollowTrajectory(Supplier<Trajectory> trajSupplier, InertiaGain i) {
    addRequirements();
    trajectorySupplier = trajSupplier;
    iGain = i;
  }

  public FollowTrajectory(Supplier<Trajectory> trajSupplier){
    this(trajSupplier, InertiaGain.id);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    follower = new HolonomicMotionProfiledTrajectoryFollower(translationConstants, rotationConstants, feedforward);
    trajectory = trajectorySupplier.get();
    timestamp = Timer.getFPGATimestamp();
    resetDriveKinematics();
    follower.follow(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateTime();
    Optional<HolonomicDriveSignal> driveSignal = calculate();
    if (driveSignal.isPresent()){
      holonomicDrive(iGain.apply(driveSignal.get()));
    } else {
      _drive.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.getCurrentTrajectory().isEmpty();
  }

  ////////////// Drivetrain-based utilities ///////////////
  private RigidTransform2 getPose(){
    return _drive.getPose();
  }

  private void updateTime(){
    lastTimestamp = timestamp;
    timestamp = Timer.getFPGATimestamp();
    dt = timestamp - lastTimestamp;
  }

  private Optional<HolonomicDriveSignal> calculate(){
    return follower.update(getPose(), _drive.getKinematicVelocity(), _drive.getGyroRate(), timestamp, dt);
  }

  private void holonomicDrive(HolonomicDriveSignal sig){
    _drive.drive(sig.getTranslation(), sig.getRotation(), sig.isFieldOriented());
  }

  private void resetDriveKinematics(){
    _drive.reset();
  }
}