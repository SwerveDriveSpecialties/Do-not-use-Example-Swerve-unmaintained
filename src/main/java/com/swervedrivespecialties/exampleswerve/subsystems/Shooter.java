/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.exampleswerve.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem{

    private Shooter(){}

    double kShooterDefaultVBus = .6; //.7
    double kFeederDefaultVBus = -.4;

    double kShooterCurrentVBus = kShooterDefaultVBus;

    double kInfeedHoldVBus = .25;
    double kOutfeedVBusDeadband = .05;

    boolean shouldRunShooter = false;
    boolean shouldRunFeeder = false;

    boolean isAuto = false;
    boolean isAutoInfeed = true;
    double kAutoInfeedVBus = .5;
    double kAutoOutfeedVBus = -.8;

    private static Shooter _instance = new Shooter();

    DigitalInput infeedLimitSwitch = new DigitalInput(0);

    public static Shooter getInstance(){
        return _instance;
    }

    TalonSRX _shooterTalonA = new TalonSRX(RobotMap.SHOOTER_TALON_A);
    TalonSRX _shooterTalonB = new TalonSRX(RobotMap.SHOOTER_TALON_B);
    TalonSRX _feederTalon = new TalonSRX(RobotMap.FEEDER_TALON);
    TalonSRX _infeedTalon = new TalonSRX(RobotMap.INFEED_TALON);

    DoubleSolenoid _puncher = new DoubleSolenoid(2, 3);
    DoubleSolenoid _succ = new DoubleSolenoid(0, 1);
    Value kPuncherDefaultValue = Value.kReverse;
    Value kSuccDefaultValue = Value.kReverse;

    public void runShooter(boolean shouldRun){
        if (shouldRun){
            _shooterTalonA.set(ControlMode.PercentOutput, kShooterDefaultVBus);
            _shooterTalonB.set(ControlMode.PercentOutput, kShooterDefaultVBus);
        } else {
            _shooterTalonA.set(ControlMode.PercentOutput, 0);
            _shooterTalonB.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runFeeder(boolean shouldRun){
        if (shouldRun){
            _feederTalon.set(ControlMode.PercentOutput, kFeederDefaultVBus);
        } else {
            _feederTalon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runInfeed(double vbus){
        if (!isAuto){
            if (infeedLimitSwitch.get()){
                if (vbus > kOutfeedVBusDeadband){
                    _infeedTalon.set(ControlMode.PercentOutput, vbus);
                } else {
                    _infeedTalon.set(ControlMode.PercentOutput, kInfeedHoldVBus);
                }
            } else {
                _infeedTalon.set(ControlMode.PercentOutput, vbus);
            }
        } else {
            if (isAutoInfeed){
                if (infeedLimitSwitch.get()){
                    _infeedTalon.set(ControlMode.PercentOutput, kInfeedHoldVBus);
                } else {
                    _infeedTalon.set(ControlMode.PercentOutput, kAutoInfeedVBus);
                }
            } else {
                _infeedTalon.set(ControlMode.PercentOutput, kAutoOutfeedVBus);
            }
        }
    }

    public void reset(){
        shouldRunShooter = false;
        shouldRunFeeder = false;
    }

    public boolean getShouldRunShooter(){
        return shouldRunShooter;
    }

    public boolean getShouldRunFeeder(){
        return shouldRunFeeder;
    }

    public void setShouldRunShooter(boolean shouldRun){
        shouldRunShooter = shouldRun;
    }

    public void setShouldRunFeeder(boolean shouldRun){
        shouldRunFeeder = shouldRun;
    }

    public void togglePuncher(){
        if (_puncher.get() == Value.kForward){
            _puncher.set(Value.kReverse);
        } else if (_puncher.get() == Value.kReverse){
            _puncher.set(Value.kForward);
        } else{
            _puncher.set(kPuncherDefaultValue);
        }
    }

    public void resetPuncher(){
        _puncher.set(kPuncherDefaultValue);
    }

    public void toggleSucc(){
        if (_succ.get() == Value.kForward){
            _succ.set(Value.kReverse);
        } else if (_puncher.get() == Value.kReverse){
            _succ.set(Value.kForward);
        } else{
            _succ.set(kSuccDefaultValue);
        }
    }

    public void resetSucc(){
        _succ.set(kSuccDefaultValue);
    }

    public boolean getSwitch(){
        return infeedLimitSwitch.get();
    }

    public void setShooterRunSpeed(double vbus){
        kShooterCurrentVBus = vbus;
    } 

    public void resetShooterRunSpeed(){
        kShooterCurrentVBus = kShooterDefaultVBus;
    }

    public void startAuto(boolean isInfeed){
        isAuto = true;
        isAutoInfeed = isInfeed;
    }

    public void endAuto(){
        isAuto = false;
    }
}