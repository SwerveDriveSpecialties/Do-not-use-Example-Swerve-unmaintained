// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package com.swervedrivespecialties.exampleswerve.commands.drive;

// import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class ZeroGyro extends CommandBase {
//   DrivetrainSubsystem _drive;

//   /**
//    * Creates a new ZeroGyro.
//    */
//   public ZeroGyro(DrivetrainSubsystem drive) {
//     _drive = drive;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("Should Be Running");
//     _drive.resetGyroscope();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted){
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }