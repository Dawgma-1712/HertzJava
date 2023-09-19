// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.Mode;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;




public class RobotContainer {
  
  public final Arm arm = new Arm();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final Joystick operator = new Joystick(OperatorConstants.DriveControllerPort);


    public RobotContainer() {
    
    configureBindings();
    }

    private void configureBindings() {
      new JoystickButton(operator, 8).onTrue(new Mode(arm, true));
      new JoystickButton(operator, 9).onFalse(new Mode(arm, false));
      if(Mode.isCone){
        new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "coneMid"));
        new JoystickButton(operator, 4).onTrue(new ArmPIDCommand(arm, "coneHigh"));
      }else{
        //something else
      }
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
