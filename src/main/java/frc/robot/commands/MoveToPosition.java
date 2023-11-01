// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.SwerveSubsystem;

// public class MoveToPosition extends CommandBase{
//     SwerveSubsystem swerveSubsystem;
//     Pose2d targetPos;
//     Double duration;

//     public MoveToPosition(SwerveSubsystem swerveSubsystem, Pose2d targetPos){
//         this.swerveSubsystem = swerveSubsystem;
//         this.targetPos = targetPos;
//     }

//     public void initialize(){
//     }

//     public void execute(){
//         Transform2d posDiff = targetPos.minus(swerveSubsystem.getPose());
//         SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(posDiff.getX(), posDiff.getY(), 0));
//         swerveSubsystem.setModuleStates(swerveModuleStates);
//     }

//     public void end(boolean interrupted){
       
//     }
    
//     public boolean isFinished(){
//         Transform2d posDiff = targetPos.minus(swerveSubsystem.getPose());
//         double dx = posDiff.getX();
//         double dy = posDiff.getX();
//         return dx*dx + dy*dy < 10;
//     }
// }
