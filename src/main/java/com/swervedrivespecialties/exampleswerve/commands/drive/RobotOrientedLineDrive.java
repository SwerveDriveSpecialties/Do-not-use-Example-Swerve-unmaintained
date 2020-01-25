package com.swervedrivespecialties.exampleswerve.commands.drive;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotOrientedLineDrive extends CommandBase {
  DrivetrainSubsystem _drive;
  Vector2 vecRobotOriented;
  int numStallCycles = 1;
  Rotation2 targetRot;

  public RobotOrientedLineDrive(DrivetrainSubsystem drive, Vector2 vec, Rotation2 targetRotation) {
    _drive = drive;
    vecRobotOriented = vec;
    targetRot = targetRotation;
  }

  public RobotOrientedLineDrive(DrivetrainSubsystem drive, Vector2 vec){
    this(drive, vec, null);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Rotation2 rot =_drive.getGyroAngle();
    Vector2 vecFieldOriented = vecRobotOriented.rotateBy(rot);
    Trajectory traj;
    if (targetRot == null){
      traj = Trajectories.generateLineTrajectory(vecFieldOriented, rot, rot);
    } else {
      traj = Trajectories.generateLineTrajectory(vecFieldOriented, rot, targetRot);
    }
    Supplier<Trajectory> trajSupplier = () -> traj;
    CommandBase followTraj = DriveSubsystemCommands.getFollowTrajectoryCommand(trajSupplier);
    followTraj.schedule();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
}