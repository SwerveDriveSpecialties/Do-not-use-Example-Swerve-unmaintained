// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package com.swervedrivespecialties.exampleswerve.commands.drive;

// import org.frcteam2910.common.math.Rotation2;
// import org.frcteam2910.common.math.Vector2;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class LineDrive extends SequentialCommandGroup {

//   private static final double kDefaultTimeout = 4;
//   /**
//    * Creates a new LineDrive.
//    */
//   public LineDrive(Vector2 vec, boolean isFieldOriented, Rotation2 rot, double timeOut) {
//     super(isFieldOriented ? DriveSubsystemCommands.getFieldOrientedLineDriveCommand(vec, rot).withTimeout(timeOut) : DriveSubsystemCommands.getRobotOrientedLineDriveCommand(vec, rot).withTimeout(timeOut));
//   }

//   public LineDrive(Vector2 vec, boolean isFieldOriented, Rotation2 rot){
//     this(vec, isFieldOriented, rot, kDefaultTimeout);
//   }

//   public LineDrive(Vector2 vec, boolean isFieldOriented, double timeOut){
//     this(vec, isFieldOriented, null, timeOut);
//   }

//   public LineDrive(Vector2 vec, boolean isFieldOriented){
//     this(vec, isFieldOriented, null, kDefaultTimeout);
//   }

//   public LineDrive(Vector2 vec){
//     this(vec, false, null, kDefaultTimeout);
//   }
// }
