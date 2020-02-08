/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {

  public static final double kEncoderCountsPerBall = 6000;
  private static final double kConveyorTalonConstantVBus = -0.3;
  private static final double kConveyToShootConstantVBUS = -.7;
  private static final double kInfeedVBus = -.4;
  private static final double kSingulatorVBus = -.45;
  private static final double kSingulateToShootVBus = -.5;

  private static Infeed _instance = new Infeed();

  public static Infeed get_instance() {
    return _instance;
  }

  private TalonSRX _conveyorTalon;
  private DigitalInput _preConveyorSensor;
  private DigitalInput _preShooterSensor;
  private boolean _preConveyorSensorLastCycle;
  private boolean _preConveyorSensorThisCycle;
  private boolean _isFirstCycle;

  private TalonSRX _singulatorTalon;
  private VictorSPX _infeedVictor;
  private DigitalInput _postSingulatorSensor;

  /**
   * Creates a new Infeed.
   */
  private Infeed() {
    _conveyorTalon = new TalonSRX(RobotMap.CONVEYOR_MOTOR);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _preConveyorSensor = new DigitalInput(RobotMap.PRE_CONVEYOR_SENSOR);
    _preShooterSensor = new DigitalInput(RobotMap.PRE_SHOOTER_SENSOR);
    _conveyorTalon.setNeutralMode(NeutralMode.Brake);

    _singulatorTalon = new TalonSRX(RobotMap.SINGULATOR_MOTOR);
    _infeedVictor = new VictorSPX(RobotMap.INFEED_MOTOR);
    _postSingulatorSensor = new DigitalInput(RobotMap.POST_SINGULATOR_SENSOR);
  }

  public void zeroEcnoder(){
    _conveyorTalon.setSelectedSensorPosition(0);
  }

  public double getConveyorPosiiton(){
    return _conveyorTalon.getSelectedSensorPosition();
  }

  public void conveyConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, kConveyorTalonConstantVBus);
  }

  public void conveyConveyorToShoot(){
    _conveyorTalon.set(ControlMode.PercentOutput, kConveyToShootConstantVBUS);
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
    return _conveyorTalon.getSelectedSensorPosition() > kEncoderCountsPerBall;
  }

  public boolean preConveyorSensorPressed() {
    if (_isFirstCycle) {
      _preConveyorSensorThisCycle = getPreConveyorSensor();
      _isFirstCycle = false;
      return false;
    } else {
      _preConveyorSensorLastCycle = _preConveyorSensorThisCycle;
      _preConveyorSensorThisCycle = getPreConveyorSensor();
      return _preConveyorSensorThisCycle && !_preConveyorSensorLastCycle;
    }
  }

  public boolean getPreConveyorSensor(){
    return !_preConveyorSensor.get();
  }

  public boolean getPreShooterSensor() {
    return !_preShooterSensor.get();
  }

  public void runInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, kInfeedVBus);
  }

  public void stopInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, 0.0);
  }

  public void runSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, kSingulatorVBus);
  }

  public void runSingulatorToShoot(){
    _singulatorTalon.set(ControlMode.PercentOutput, kSingulateToShootVBus);
  }

  public void stopSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean getPostSingulatorSensor(){
    return !_postSingulatorSensor.get();
  }

  @Override
  public void periodic() {
    if (preConveyorSensorPressed()) {
      CommandBase conveyorCommand = InfeedSubsystemCommands.getConveyCommand();
      conveyorCommand.schedule();
    }
  }
}
