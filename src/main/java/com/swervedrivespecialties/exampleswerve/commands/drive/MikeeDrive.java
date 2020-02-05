/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MikeeDrive extends CommandBase {
  DrivetrainSubsystem _drive;

  double kRot = .025;
  double kDeadBand = .06; //custom because of the unique demands of this task

  boolean shouldFinish = false;
  
  public MikeeDrive(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drive = drive;
    addRequirements(_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotVal = -Robot.getRobotContainer().getPrimaryRightXAxis();
    if (rotVal > kDeadBand){
      _drive.drive(new Translation2d(), kRot, true);
    } else if (rotVal < -kDeadBand){
      _drive.drive(new Translation2d(), -kRot, true);
    } else {
      _drive.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.drive(new Translation2d(), 0, true);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
