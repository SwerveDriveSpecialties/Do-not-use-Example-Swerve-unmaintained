/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake implements Subsystem {
  
  
  private TalonSRX feedinTalon = new TalonSRX(14);
  private TalonSRX indexTalon = new TalonSRX(15);
  // private TalonSRX driveThruForwardTalon = new TalonSRX(15);
  private TalonSRX driveThruTalon = new TalonSRX(1);

  private static final double kFeedInVBus = -.7;
  private static final double kIndexVBus = -.5;
  private static final double kDriveThruVBus = -.25;

  public Intake() {}

  private static final Intake instance = new Intake();

  public static Intake getInstance(){
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFeedIn(boolean run){
    double spd = run ? kFeedInVBus : 0.0;
    System.out.println("FeedIn Spd: " + spd);
    feedinTalon.set(ControlMode.PercentOutput, spd);
  }

  public void runDriveThru(boolean run){
    double spd = run ? kDriveThruVBus : 0.0;
    driveThruTalon.set(ControlMode.PercentOutput, spd);
  }

  public void runIndex(boolean run){
    double spd = run ? -kIndexVBus: 0.0;
    indexTalon.set(ControlMode.PercentOutput, spd);
  }
}
