/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.ShooterTable;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RunShooterFromVision extends CommandBase {
  Shooter _shooter;
  Limelight _ll = Limelight.getInstance();
  double curTargVelo = .7;

  public RunShooterFromVision(Shooter shooter) {
    _shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //getSpeed();
    curTargVelo = .7;
    _shooter.runShooter(.7);
    System.out.println("Initializing");
  }

  @Override 
  public void execute(){
    //getSpeed();
    _shooter.runShooter(.7);
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override 
  public void end(boolean interrupted){
    _shooter.runShooter(0.0);
  }

  // private void getSpeed(){
  //   if (_ll.getDistanceToTarget(Target.HIGH) > 0 && _ll.getDistanceToTarget(Target.HIGH) < 420){
  //     double dist = util.inchesToFeet(_ll.getDistanceToTarget(Target.HIGH));
  //     double targetSpeed = ShooterTable.getInstance().CalcShooterValues(dist).MotorTargetRPM;
  //     curTargVelo = targetSpeed;
  //   }
  // }
}
