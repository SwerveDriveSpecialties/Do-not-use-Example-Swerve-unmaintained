/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.exampleswerve.commands.shooter.runConveyorMotors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {

  private static Infeed _instance = new Infeed();

  public static Infeed get_instance() {
    return _instance;
  }

  private TalonSRX _conveyorTalon;
  private DigitalInput _beamSensor;
  private boolean _isFinished = false;
  private boolean _sensorlast;
  private boolean _sensorthis;
  private boolean _isFirstCycle;

  public enum CONVEYORPHASES {
    PHASE1, PHASE2;
  }

  CONVEYORPHASES phase = CONVEYORPHASES.PHASE1;

  /**
   * Creates a new Infeed.
   */
  public Infeed() {
    _conveyorTalon = new TalonSRX(12);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _beamSensor = new DigitalInput(0);
  }

  public void setDefault() {
    phase = CONVEYORPHASES.PHASE1;
    _isFinished = false;
    _conveyorTalon.setSelectedSensorPosition(0);
  }

  public void vbusFeederWheel(){
    _conveyorTalon.set(ControlMode.PercentOutput, -0.7);
  }
  public void setConveyorZero()
  {
    _conveyorTalon.set(ControlMode.PercentOutput, 0.0);
  }

  public void conveyorPhases() {
    if (_conveyorTalon.getSelectedSensorPosition() <= 7500) {
      _conveyorTalon.set(ControlMode.PercentOutput, -0.3);
      _isFinished = false;
      phase = CONVEYORPHASES.PHASE2;
    }

    else if (phase == CONVEYORPHASES.PHASE2) {
      _conveyorTalon.set(ControlMode.PercentOutput, 0);
      _isFinished = true;
    }

    else {
      System.out.println("you're bad");
    }
  }

  

  public void PrintSmartDash()
  {
    SmartDashboard.putBoolean("SensorVAl", _beamSensor.get());
  }
  public void endConveyorPhases() {
    _conveyorTalon.set(ControlMode.PercentOutput, 0);
  }

  public boolean isSensorTrue() {
    if (_isFirstCycle) {
      _sensorthis = _beamSensor.get();
      return false;
    } else {
      _sensorlast = _sensorthis;
      _sensorthis = _beamSensor.get();
      return !_sensorthis && _sensorlast;
    }
  }

  public boolean isConveyorFinished() {
    return _isFinished;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isSensorTrue()) {
      CommandBase conveyorCommand = new runConveyorMotors(_instance);
      conveyorCommand.schedule();
    }
  }
}
