package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private static OI oi;

    private static DrivetrainSubsystem drivetrain;
    private static Limelight _limelight = Limelight.getInstance();

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        oi = new OI();
        drivetrain = DrivetrainSubsystem.getInstance();
        _limelight.setPipeline(6);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
        //System.out.println(DrivetrainSubsystem.getInstance().getMinControllerSpeed());
        SmartDashboard.putNumber("LL YAng", _limelight.getYAng());
        SmartDashboard.putNumber("LL Skew", _limelight.getSkew());
        SmartDashboard.putNumber("LL X", _limelight.getAngle1());
        SmartDashboard.putNumber("LL Distance", _limelight.getDistanceToTarget(Target.HIGH));
        //_limelight.splitLLStream();
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        super.teleopInit();
    }
    @Override
    public void teleopPeriodic(){
        //super.teleopPeriodic();
        //Scheduler.getInstance().run();
        _limelight.splitLLStream();
    }
}
