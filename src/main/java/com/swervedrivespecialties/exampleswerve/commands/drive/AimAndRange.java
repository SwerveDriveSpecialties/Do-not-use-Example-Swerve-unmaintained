/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.robot.drivers.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimAndRange extends CommandBase {

  DrivetrainSubsystem _drive;
  double targetRange;
  RigidTransform2 toTarget;

  public AimAndRange(DrivetrainSubsystem drive) {
    _drive = drive;
  }

  @Override
  public void initialize() {
    toTarget = RigidTransform2.ZERO; //Change this to get Limelight values
    double len = toTarget.translation.length;
    if (len > 0){
      DriveSubsystemCommands.getLineDriveCommand(toTarget.translation.scale((len - targetRange) / len), false, toTarget.rotation).schedule();;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
