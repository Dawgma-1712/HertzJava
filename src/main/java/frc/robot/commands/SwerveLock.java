package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SwerveLock extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveModuleState[] states = {
    new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0)),//Front left
    new SwerveModuleState(0, new Rotation2d(Math.PI/4.0)),//Front right
    new SwerveModuleState(0, new Rotation2d(Math.PI/4.0)),//Back left
    new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0))//Back right
  };

  public SwerveLock(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    swerveSubsystem.setModuleStates(states);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(swerveSubsystem.getFL().getTurnPosition() - states[0].angle.getRadians()) < 3 && 
          Math.abs(swerveSubsystem.getFR().getTurnPosition() - states[1].angle.getRadians()) < 3 && 
          Math.abs(swerveSubsystem.getBL().getTurnPosition() - states[2].angle.getRadians()) < 3 && 
          Math.abs(swerveSubsystem.getBR().getTurnPosition() - states[3].angle.getRadians()) < 3;
  }
}
