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

    TalonSRX _shooterTalon = new TalonSRX(RobotMap.SHOOTER_TALON);
    CANSparkMax _shooterNEO = new CANSparkMax(RobotMap.SHOOTER_MASTER_NEO, MotorType.kBrushless);
    CANSparkMax _shooterSlave = new CANSparkMax(RobotMap.SHOOTER_SLAVE_NEO, MotorType.kBrushless);
    TalonSRX _feederTalon = new TalonSRX(RobotMap.FEEDER_TALON);
    Servo _linearActuator = new Servo(0);

    CANPIDController _pidController;
    CANEncoder _encoder;
    double _P = 0.0011;
    double _I = 0;
    double _D = 0.01;
    double _F = 0.0001838;
    double minOutput = -1;
    double maxOutput = 1;
    double maxRPM = 5440;

    int _MtrTargetRPM;
   
    private Shooter(){

        _encoder = _shooterNEO.getEncoder();
        _pidController = new CANPIDController(_shooterNEO);

        //_shooterSlave.follow(_shooterNEO, true);

        _pidController.setP(_P);
        _pidController.setI(_I);
        _pidController.setD(_D);
        _pidController.setFF(_F);
        _pidController.setOutputRange(minOutput, maxOutput);
    } 
    
    public void runFeeder(boolean shouldRun){
        double runSpeed = shouldRun ? kFeederDefaultVBus : 0.;
        _feederTalon.set(ControlMode.PercentOutput, -runSpeed);
    }

    public void runShooter(double spd, double actuatorVal){
        SmartDashboard.putNumber("spd", spd);
        SmartDashboard.putNumber("Target RPM", spd);
        SmartDashboard.putNumber("velo", _encoder.getVelocity());
        SmartDashboard.putNumber("ActuatorVal", actuatorVal); 
        double talonSpeed = spd > 0 ? spd / kMaxSpeed : 0.0;

        _shooterTalon.set(ControlMode.PercentOutput, -talonSpeed);
        _linearActuator.set(.67);
      if (spd <= 20)
      {
        _shooterNEO.set(-0.0);
        _shooterSlave.set(0.0);
      
      }
      else{

        //atual
       _shooterNEO.set(-0.8);
       _shooterSlave.set(0.8);
      }
    }


 

    public void outputToSDB(){
        SmartDashboard.putNumber("Distance to Target", Limelight.getInstance().getDistanceToTarget(Target.HIGH));
    }
    public void updateLogData(LogDataBE logData){  
       
    }
}