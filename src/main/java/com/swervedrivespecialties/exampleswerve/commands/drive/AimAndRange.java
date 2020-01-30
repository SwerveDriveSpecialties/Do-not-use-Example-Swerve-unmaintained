/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;

import org.frcteam2910.common.math.RigidTransform2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimAndRange extends CommandBase {

  DrivetrainSubsystem _drive;
  double targetRange;
  RigidTransform2 toTarget;

  public AimAndRange(DrivetrainSubsystem drive, double range) {
    _drive = drive;
    targetRange = range;
  }

  @Override
  public void initialize() {
    toTarget = Limelight.getInstance().getToTarget(Target.HIGH); //Change this to get Limelight values
    double len = toTarget.translation.length;
    if (len > 0){
      DriveSubsystemCommands.getLineDriveCommand(toTarget.translation.scale((len - targetRange) / len), false, toTarget.rotation).schedule();
    } else {
      System.out.println("Limelight Doesn't See Target");
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
