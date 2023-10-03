// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.ArmJoystickCommand;
import frc.robot.commands.ArmMode;
import frc.robot.commands.ClawCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  private final Joystick driver = new Joystick(OperatorConstants.DriverControllerPort);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick operator = new Joystick(OperatorConstants.OperatorControllerPort);
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(
      swerveSubsystem,
      () -> -driver.getRawAxis(OperatorConstants.DriverYAxis),
      () -> driver.getRawAxis(OperatorConstants.DriverXAxis),
      () -> driver.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !driver.getRawButton(OperatorConstants.DriverFieldOrientedButton)
    ));

    arm.setDefaultCommand(new ArmJoystickCommand(
      arm,
      () -> -operator.getRawAxis(OperatorConstants.OperatorExtend), 
      () -> operator.getRawAxis(OperatorConstants.OperatorRaise)
      )); 
    configureBindings(); 
  }

  private void configureBindings() {
    //Swerve controls
    new JoystickButton(driver, 1).onTrue(new SwerveZeroHeading(swerveSubsystem));

    //Arm controls
    new JoystickButton(operator, 6).onTrue(new ArmMode(arm, !arm.getIsCone()));//RB

    if(arm.getIsCone()){
      new JoystickButton(operator, 1).onTrue(new ArmPIDCommand(arm, "coneMid"));//Button A
      new JoystickButton(operator, 4).onTrue(new ArmPIDCommand(arm, "coneHigh"));//Button Y
    }
    else{
      new JoystickButton(operator, 1).onTrue(new ArmPIDCommand(arm, "cubeMid"));
      new JoystickButton(operator, 4).onTrue(new ArmPIDCommand(arm, "cubeHigh"));
    }
    new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "stow"));//Button X
    new JoystickButton(operator, 2).onTrue(new ArmPIDCommand(arm, "ground"));//Button B
    new JoystickButton(operator, 7).onTrue(new ArmPIDCommand(arm, "substation"));//Button Back

    //Claw controls
    new JoystickButton(operator, 5).onTrue(new ClawCMD(claw, !claw.getIsOpen()));//Button LB
  }
  public Command getAutonomousCommand() {
    return null;
  }
}
