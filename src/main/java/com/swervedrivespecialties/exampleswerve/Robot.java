package com.swervedrivespecialties.exampleswerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
// import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot{

    static RobotContainer robotContainer;

    TalonSRX talon11 = new TalonSRX(3);
    TalonSRX talon14 = new TalonSRX(14);
    Joystick joy = new Joystick(1);
    JoystickButton joyA = new JoystickButton(joy, 1);
    JoystickButton joyB = new JoystickButton(joy, 2);

    @Override
    public void robotInit() {
        Trajectories.generateAllTrajectories();
        //robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        //CommandScheduler.getInstance().run();
        if (!isDisabled()){
            // robotContainer.logAllData();
            // robotContainer.outputToSDB();
        }
    }

    @Override
    public void autonomousInit() {
        //robotContainer.setupLogging(true);
    }

    @Override
    public void autonomousPeriodic() {
        // DrivetrainSubsystem.getInstance().drive(new Translation2d(0., 0), 0, true);
    }

    @Override
    public void teleopInit() {
        //robotContainer.setupLogging(false);
        CommandScheduler.getInstance().cancelAll();
    }
    @Override
    public void teleopPeriodic(){
        if (joyA.get()){
            talon11.set(ControlMode.PercentOutput, -.35);
        } else {
            talon11.set(ControlMode.PercentOutput, 0.0);
        }
        if (joyB.get()){
            talon14.set(ControlMode.PercentOutput, -1.0);
        } else {
            talon14.set(ControlMode.PercentOutput, 0.);
        }
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
