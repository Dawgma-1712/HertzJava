package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SwervePivotCMD extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xFunction, yFunction;

  public SwervePivotCMD(SwerveSubsystem swerveSubsystem, Supplier<Double> xFunction, Supplier<Double> yFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xFunction = xFunction;
    this.yFunction = yFunction;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Math.atan2(xFunction.get(), yFunction.get());
    if(swerveSubsystem.getHeading() <= angle + Math.PI/2.0){

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
