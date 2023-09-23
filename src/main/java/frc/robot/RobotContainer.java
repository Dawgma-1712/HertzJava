// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.ArmMode;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final Joystick driver = new Joystick(OperatorConstants.DriverControllerPort);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick operator = new Joystick(OperatorConstants.OperatorControllerPort);
  public final Arm arm = new Arm();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(
      swerveSubsystem,
      () -> -driver.getRawAxis(OperatorConstants.DriverYAxis),
      () -> driver.getRawAxis(OperatorConstants.DriverXAxis),
      () -> driver.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !driver.getRawButton(OperatorConstants.DriverFieldOrientedButton)
    ));
    configureBindings();
  }

  private void configureBindings() {
    //Swerve controls
    new JoystickButton(driver, 1).onTrue(new SwerveZeroHeading(swerveSubsystem));

    //Arm controls
    new JoystickButton(operator, 5).onTrue(new ArmMode(arm, !arm.getIsCone()));

    if(arm.getIsCone()){
      new JoystickButton(operator, 0).onTrue(new ArmPIDCommand(arm, "coneMid"));
      new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "coneHigh"));
    }
    else{
      new JoystickButton(operator, 0).onTrue(new ArmPIDCommand(arm, "cubeMid"));
      new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "cubeHigh"));
    }
    new JoystickButton(operator, 2).onTrue(new ArmPIDCommand(arm, "stow"));
    new JoystickButton(operator, 1).onTrue(new ArmPIDCommand(arm, "ground"));
    new JoystickButton(operator, 6).onTrue(new ArmPIDCommand(arm, "substation"));
    //Claw button is 4

  }
  public Command getAutonomousCommand() {
    return null;
  }
}
