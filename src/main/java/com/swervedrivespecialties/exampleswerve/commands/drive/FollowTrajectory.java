/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import java.util.Optional;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase {
  DrivetrainSubsystem _drive;

  ///////////// PATH FOLLOWING CONSTANTS //////////////////
  private static final double kMaxVelo = 12 * 12; //This is the physical max velocity of the machine, not of any path
  private static final double kInterceptVoltage = .015; //the physical minimum voltage to make the robot move forward
  private static final double kPathFollowingVeloFeedForward = 0.0051896679;
  private static final double kPathFollowingAccelFeedForward = 0;
  private static final double kPathFollowingTranslationP = .05;
  private static final double kPathFollowingTranslationI = 0;
  private static final double kPathFollowingTranslationD = 0;
  private static final double kPathFollowingRotationP = .5;
  private static final double kPathFollowingRotationI = 0;
  private static final double kPathFollowingRotationD = .06;
  ////////////////////////////////////////////////////////////

  //create appropriate constant classes and eventually a TrajectoryFollower from constants given above
  PidConstants translationConstants = new PidConstants(kPathFollowingTranslationP, kPathFollowingTranslationI, kPathFollowingTranslationD);
  PidConstants rotationConstants = new PidConstants(kPathFollowingRotationP, kPathFollowingRotationI, kPathFollowingRotationD);
  double kFeedForwardVelocity = (1 - kInterceptVoltage) / kMaxVelo;
  DrivetrainFeedforwardConstants feedforwardConstants = new DrivetrainFeedforwardConstants(kPathFollowingVeloFeedForward, kPathFollowingAccelFeedForward, kInterceptVoltage);
  HolonomicFeedforward feedforward = new HolonomicFeedforward(feedforwardConstants); //can have separate forward and strafe feed forwards if desired. 
  HolonomicMotionProfiledTrajectoryFollower follower;

  private Trajectory trajectory;
  private Supplier<Trajectory> trajectorySupplier;
  private InertiaGain iGain;

  double timestamp;
  double lastTimestamp;
  double dt;


  public FollowTrajectory(DrivetrainSubsystem drive, Supplier<Trajectory> trajSupplier, InertiaGain i) {
    _drive = drive;
    addRequirements(_drive);
    trajectorySupplier = trajSupplier;
    iGain = i;
  }

  public FollowTrajectory(DrivetrainSubsystem drive, Supplier<Trajectory> trajSupplier){
    this(drive, trajSupplier, InertiaGain.id);
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
    printInfo();
    if (interrupted){
      _drive.stop();
    } else {
      CommandBase finishRotate = DriveSubsystemCommands.getRotateToAngleCommand(trajectory.calculateSegment(trajectory.getDuration()).rotation.toDegrees());
      finishRotate.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.getCurrentTrajectory().isEmpty();
  }

  ////////////// Drivetrain-based utilities ///////////////
  private RigidTransform2 getPose(){
    return new RigidTransform2(_drive.getKinematicPosition(), _drive.getGyroAngle());
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
    _drive.drive(fromTranslation2(sig.getTranslation()), sig.getRotation(), sig.isFieldOriented());
  }

  private void resetDriveKinematics(){
    _drive.reset();
  }
  
  private Translation2d fromTranslation2(Vector2 trans){
      return new Translation2d(trans.x, trans.y);
  }

  private void printInfo(){
    RigidTransform2 curPose = getPose();
    RigidTransform2 goalPose = new RigidTransform2(trajectory.calculateSegment(trajectory.getDuration()).translation, trajectory.calculateSegment(trajectory.getDuration()).rotation);
    System.out.println("Goal Pose: (" + Double.toString(goalPose.translation.x) + ", " + Double.toString(goalPose.translation.y) + ", " + Double.toString(goalPose.rotation.toDegrees()));
    System.out.println("Current Pose: (" + Double.toString(curPose.translation.x) + ", " + Double.toString(curPose.translation.y) + ", " + Double.toString(curPose.rotation.toDegrees()));
  }
}