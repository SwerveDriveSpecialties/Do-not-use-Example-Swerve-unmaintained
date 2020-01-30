/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem{

    private Shooter(){}

    double kFeederDefaultVBus = .8;

    private static Shooter _instance = new Shooter();

    public static Shooter getInstance(){
        return _instance;
    }

    TalonSRX _shooterTalon = new TalonSRX(RobotMap.SHOOTER_TALON);
    CANSparkMax _shooterNEO = new CANSparkMax(RobotMap.SHOOTER_NEO, MotorType.kBrushless);
    TalonSRX _feederTalon = new TalonSRX(RobotMap.FEEDER_TALON);

    public void runShooter(double speed){
        _shooterNEO.set(speed);
        _shooterTalon.set(ControlMode.PercentOutput, -speed);
    }

    public void runFeeder(boolean shouldRun){
        double runSpeed = shouldRun ? kFeederDefaultVBus : 0.;
        _feederTalon.set(ControlMode.PercentOutput, runSpeed);
    }
}