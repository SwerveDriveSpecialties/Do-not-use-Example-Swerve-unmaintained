/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Add your docs here.
 */
public class DriveSubsystemCommands {
    public static DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    
    public static CommandBase getDriveCommand(){
        return new DriveCommand(drivetrainSubsystem);
    }

    public static CommandBase getRotateToAngleCommand(double targetAngleDegrees, double timeout){
        return new RotateToAngle(drivetrainSubsystem, targetAngleDegrees, timeout);
    }

    public static CommandBase getRotateToAngleCommand(double targetAngleDegrees){
        return new RotateToAngle(DrivetrainSubsystem.getInstance(), targetAngleDegrees);
    }

    public static CommandBase getFieldOrientedLineDriveCommand(Vector2 vec, Rotation2 rot){
        return new FieldOrientedLineDrive(drivetrainSubsystem, vec, rot);
    }

    public static CommandBase getFieldOrientedLineDriveCommand(Vector2 vec){
        return new FieldOrientedLineDrive(drivetrainSubsystem, vec);
    }

    public static CommandBase getRobotOrientedLineDriveCommand(Vector2 vec, Rotation2 rot){
        return new RobotOrientedLineDrive(drivetrainSubsystem, vec, rot);
    }

    public static CommandBase getRobotOrientedLineDriveCommand(Vector2 vec){
        return new RobotOrientedLineDrive(drivetrainSubsystem, vec);
    }

    public static CommandBase getFollowTrajectoryCommand(Supplier<Trajectory> trajSupplier, InertiaGain i){
        return new FollowTrajectory(drivetrainSubsystem, trajSupplier, i);
    }

    public static CommandBase getFollowTrajectoryCommand(Supplier<Trajectory> trajSupplier){
        return new FollowTrajectory(drivetrainSubsystem, trajSupplier);
    }

    public static CommandBase getVictorySpinCommand(double timeout){
        return new VictorySpin(drivetrainSubsystem).withTimeout(timeout);
    }

    public static CommandBase getLineDriveCommand(Vector2 vec, boolean isFieldOriented, Rotation2 rot, double timeOut){
        return new LineDrive(vec, isFieldOriented, rot, timeOut);
    }

    public static CommandBase getLineDriveCommand(Vector2 vec, boolean isFieldOriented, Rotation2 rot){
        return new LineDrive(vec, isFieldOriented, rot);
    }

    public static CommandBase getLineDriveCommand(Vector2 vec, boolean isFieldOriented){
        return new LineDrive(vec, isFieldOriented);
    }

    public static CommandBase getLineDriveCommand(Vector2 vec){
        return new LineDrive(vec);
    }

    public static CommandBase getZeroGyroCommand(){
        return new ZeroGyro(drivetrainSubsystem);
    }

    public static CommandBase getToggleSpeedCommand(){
        return new ToggleSpeed(drivetrainSubsystem);
    }

    public static CommandBase getToggleFieldOrientedCommand(){
        return new ToggleSpeed(drivetrainSubsystem);
    }
    public static CommandBase getLLRotateToTargetCommand(){
        return new LLRotateToTarget(Limelight.getInstance(), drivetrainSubsystem).withTimeout(2.5);
    }
}
