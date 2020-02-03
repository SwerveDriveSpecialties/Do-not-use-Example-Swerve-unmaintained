// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package com.swervedrivespecialties.exampleswerve.commands.drive;

// import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
// import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;

// import org.frcteam2910.common.control.PidConstants;
// import org.frcteam2910.common.control.PidController;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class LLRotateToTarget extends CommandBase {
  
//   DrivetrainSubsystem _drive;
//   Limelight _limelight;


//   PidController _rotController = new PidController(new PidConstants(0.012, 0.01, 0.0008));

//   private double _prevTime, _initTime, error;

//   public LLRotateToTarget(Limelight limelight, DrivetrainSubsystem subsystem) {
//     _limelight = limelight;
//     _drive = subsystem;
//     _rotController.setContinuous(true);
//     _rotController.setInputRange(-180, 180);
//     _rotController.setOutputRange(-1, 1);
//     _rotController.setSetpoint(0);
//     _rotController.setIntegralRange(10);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     _prevTime = Timer.getFPGATimestamp();
//     _initTime = Timer.getFPGATimestamp();
//     error = _limelight.getAngle1();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double dt = Timer.getFPGATimestamp() - _prevTime;
//     _prevTime = Timer.getFPGATimestamp();

//     error = _limelight.getAngle1();
//     double rot = _rotController.calculate(error, dt);
//     _drive.drive(new Translation2d(0, 0), rot, true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(error) <= 0.5;
//   }
// }
