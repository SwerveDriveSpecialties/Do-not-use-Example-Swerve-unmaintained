// package com.swervedrivespecialties.exampleswerve.commands.drive;

// import java.util.function.Supplier;

// import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
// import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

// import org.frcteam2910.common.control.Trajectory;
// import org.frcteam2910.common.math.Rotation2;
// import org.frcteam2910.common.math.Vector2;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class FieldOrientedLineDrive extends CommandBase {
  
//   DrivetrainSubsystem _drive;
//   Vector2 driveVector;
//   Rotation2 driveRotation;

//   public FieldOrientedLineDrive(DrivetrainSubsystem drive, Vector2 vec, Rotation2 rot) {
//     _drive = drive;
//     driveVector = vec;
//     driveRotation = rot;
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     addRequirements(_drive);
//   }

//   public FieldOrientedLineDrive(DrivetrainSubsystem drive, Vector2 vec){
//       this(drive, vec, null);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Supplier<Trajectory>lineTrajSupplier;
//     Rotation2 curRot = _drive.getGyroAngle(); 
//     if (driveRotation == null){
//       Trajectory lineTraj = Trajectories.generateLineTrajectory(driveVector, curRot, curRot);
//       lineTrajSupplier = () -> lineTraj;
//     } else {
//       Trajectory lineTraj = Trajectories.generateLineTrajectory(driveVector, curRot, driveRotation);
//       lineTrajSupplier = () -> lineTraj;
//     }
//     CommandBase driveCommand = DriveSubsystemCommands.getFollowTrajectoryCommand(lineTrajSupplier);
//     driveCommand.schedule();
//   }

//     // Called repeatedly when this Command is scheduled to run
//     @Override
//     public void execute() {
//     }

// // Make this return true when this Command no longer needs to run execute()
// @Override
// public boolean isFinished() {
//     return true;
//   }
// }