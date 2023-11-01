package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveAtSpeedForTime extends CommandBase{
    SwerveModuleState[] swerveModuleStates;
    SwerveSubsystem swerveSubsystem;
    Double duration;
    Timer timer;

    public MoveAtSpeedForTime(SwerveSubsystem swerveSubsystem, double fSpeed, double sSpeed, double rotation, double duration){
        this.swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(fSpeed, sSpeed, rotation));
        this.swerveSubsystem = swerveSubsystem;
        this.duration = duration;
        this.timer = new Timer();
    }

    public void initialize(){
        this.timer.start();
    }

    public void execute(){
        swerveSubsystem.setModuleStates(this.swerveModuleStates);
    }

    public void end(boolean interrupted){
       
    }
    
    public boolean isFinished(){
        return this.timer.hasElapsed(this.duration);
    }
}
