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

  @Override
  public void execute() {
    double angle = Math.atan2(xFunction.get(), yFunction.get());
    double heading = swerveSubsystem.getHeading() * Math.PI/180.0;
    if(heading < angle + Math.PI/2.0){
      //Spin according to FL
    }
    else if(heading < angle + Math.PI){
      //Spin according to FR
    }
    else if(heading < angle + 3 * Math.PI / 2.0){
      //Spin according to BR
    }
    else{
      //Spin according to BL
    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
