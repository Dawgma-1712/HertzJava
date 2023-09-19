package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveZeroHeading extends CommandBase{
    
    public final SwerveSubsystem swerveSubsystem;

    public SwerveZeroHeading(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
