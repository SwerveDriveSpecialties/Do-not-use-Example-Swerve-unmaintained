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
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {

  private static final double kEncoderCountsPerBall = 7500;
  private static final double kConveyorTalonConstantVBus = -0.3;
  private static final double kFeederTalonConstantVBus = 0.7;

  private static Infeed _instance = new Infeed();

  public static Infeed get_instance() {
    return _instance;
  }

  private TalonSRX _conveyorTalon;
  private DigitalInput _preConveyorSensor;
  private DigitalInput _preShooterSensor;
  private boolean _isFinished = false;
  private boolean _preConveyorSensorLastCycle;
  private boolean _preConveyorSensorThisCycle;
  private boolean _isFirstCycle;

  /**
   * Creates a new Infeed.
   */
  private Infeed() {
    _conveyorTalon = new TalonSRX(RobotMap.CONVEYOR_MOTOR);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _preConveyorSensor = new DigitalInput(RobotMap.BEAM_SENSOR);
    _preShooterSensor = new DigitalInput(RobotMap.BEAM_SENSOR_STOP_BALL);
  }

  public void zeroEcnoder(){
    _conveyorTalon.setSelectedSensorPosition(0);
  }

  public void runConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, kConveyorTalonConstantVBus);
  }

  public void stopConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, 0.0);
  }

  public void outputToSDB() {
    SmartDashboard.putBoolean("PRE-SHOOTER SENSOR", _preShooterSensor.get());
    SmartDashboard.putNumber("CONVEYOR TALON ENCODER", _conveyorTalon.getSelectedSensorPosition());
    SmartDashboard.putBoolean("PRE-CONVEYOR SENSOR", _preConveyorSensor.get());
  }

  public boolean getHasBallConveyedBallLength(){
    return _conveyorTalon.getSelectedSensorPosition() <= kEncoderCountsPerBall;
  }

  public boolean preConveyorSensorPressed() {
    if (_isFirstCycle) {
      _preConveyorSensorThisCycle = _preConveyorSensor.get();
      return false;
    } else {
      _preConveyorSensorLastCycle = _preConveyorSensorThisCycle;
      _preConveyorSensorThisCycle = _preConveyorSensor.get();
      return !_preConveyorSensorThisCycle && _preConveyorSensorLastCycle;
    }
  }

  public boolean getPreShooterSensor() {
    return !_preShooterSensor.get();
  }

  @Override
  public void periodic() {
    if (preConveyorSensorPressed()) {
      CommandBase conveyorCommand = InfeedSubsystemCommands.getRunConveyorMotorsCommand();
      conveyorCommand.schedule();
    }
  }
}
