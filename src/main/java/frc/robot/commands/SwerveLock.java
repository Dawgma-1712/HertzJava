package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SwerveLock extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;

  public SwerveLock(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("Happened");
    swerveSubsystem.locked = !swerveSubsystem.locked;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended ended");
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return true;
  //   return Math.abs(swerveSubsystem.getFL().getTurnPosition() - states[0].angle.getRadians()) < 0.1 && 
  //         Math.abs(swerveSubsystem.getFR().getTurnPosition() - states[1].angle.getRadians()) < 0.1 && 
  //         Math.abs(swerveSubsystem.getBL().getTurnPosition() - states[2].angle.getRadians()) < 0.1 && 
  //         Math.abs(swerveSubsystem.getBR().getTurnPosition() - states[3].angle.getRadians()) < 0.1;
  }
}
