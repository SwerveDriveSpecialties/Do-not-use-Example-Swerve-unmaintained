/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.ShooterTable;
import com.swervedrivespecialties.exampleswerve.util.ShooterTableEntry;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem{

    double kFeederDefaultVBus = 0.8;
    double kShooterTalonDefaultVBus = -.5;
    double kMaxSpeed = 5440.0; //Native Units

    private static Shooter _instance = new Shooter();
    private static ShooterTable _shooterTable = ShooterTable.getInstance();

    public static Shooter getInstance(){
        return _instance;
    }

    TalonSRX _kickerTalon = new TalonSRX(RobotMap.KICKER_TALON);
    CANSparkMax _shooterNEO = new CANSparkMax(RobotMap.SHOOTER_MASTER_NEO, MotorType.kBrushless);
    CANSparkMax _shooterSlave = new CANSparkMax(RobotMap.SHOOTER_SLAVE_NEO, MotorType.kBrushless);
    TalonSRX _feederTalon = new TalonSRX(RobotMap.KICKER_TALON);
    Servo _linearActuator = new Servo(0);

    CANPIDController _pidController;
    CANEncoder _encoder;
    double _P = 0.00014;
    double _I = 0;
    double _D = 0.002;
    double _F = 0.000201897;
    double minOutput = -1;
    double maxOutput = 1;
    double maxRPM = 4885;
//1/4953
    int _MtrTargetRPM;
   
    private Shooter(){

        _shooterNEO.restoreFactoryDefaults();
        _shooterSlave.restoreFactoryDefaults();
        _shooterNEO.setIdleMode(IdleMode.kCoast);
        _shooterNEO.setIdleMode(IdleMode.kCoast);


        _shooterNEO.setInverted(true);
        _shooterSlave.setInverted(false);
        _shooterSlave.follow(_shooterNEO, true);
        _encoder = _shooterNEO.getEncoder();
        _pidController = new CANPIDController(_shooterNEO);

        //_shooterSlave.follow(_shooterNEO, true);

        _pidController.setP(_P);
        _pidController.setI(_I);
        _pidController.setD(_D);
        _pidController.setFF(_F);
        _pidController.setOutputRange(minOutput, maxOutput);
        
    } 

    public void runShooter(double spd, double actuatorVal){
        SmartDashboard.putNumber("spd", spd);
        SmartDashboard.putNumber("Target RPM", 4200);
        SmartDashboard.putNumber("velo", _encoder.getVelocity());
        SmartDashboard.putNumber("Vello", _encoder.getVelocity());
        SmartDashboard.putNumber("ActuatorVal", actuatorVal); 
        double talonSpeed = spd > 0 ? spd / kMaxSpeed : 0.0;
        _kickerTalon.set(ControlMode.PercentOutput, -talonSpeed);
        if (spd > 20){
            _pidController.setReference(spd, ControlType.kVelocity);
        } else {
            _shooterNEO.set(0.0);
        }
        _linearActuator.set(actuatorVal);
      }

    public void outputToSDB(){
        SmartDashboard.putNumber("Distance to Target", Limelight.getInstance().getDistanceToTarget(Target.HIGH));
    }

    public void updateLogData(LogDataBE logData){  
       
    }
}