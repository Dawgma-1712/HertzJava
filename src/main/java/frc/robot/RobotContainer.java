// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

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
    new JoystickButton(driver, 9).onTrue(new SwerveLock(swerveSubsystem));
    new JoystickButton(driver, 8).onTrue(new SwerveSlowMode(swerveSubsystem, 0.3)).onFalse(new SwerveSlowMode(swerveSubsystem, 1));
    // new JoystickButton(driver, 8).whileTrue(new SwerveSlowMode(swerveSubsystem, 0));
    //Arm controls
    new JoystickButton(operator, 6).toggleOnTrue(new ArmMode(arm));//RB

    if(arm.getIsCone()){
      new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "coneMid"));//Button X
      new JoystickButton(operator, 4).onTrue(new ArmPIDCommand(arm, "coneHigh"));//Button Y
    }
    if(!arm.getIsCone()){
      new JoystickButton(operator, 3).onTrue(new ArmPIDCommand(arm, "cubeMid"));
      new JoystickButton(operator, 4).onTrue(new ArmPIDCommand(arm, "cubeHigh"));
    }
    new JoystickButton(operator, 2).onTrue(new ArmPIDCommand(arm, "stow"));//Button B
    new JoystickButton(operator, 1).onTrue(new ArmPIDCommand(arm, "ground"));//Button A
    //new JoystickButton(operator, 7).onTrue(new ArmPIDCommand(arm, "substation"));//Button Back

    //Claw controls
    new JoystickButton(operator, 5).toggleOnTrue(new ClawCMD(claw));//Button LB
  }
  public Command getAutonomousCommand() {
    //Trajectory setting
    TrajectoryConfig config = new TrajectoryConfig(
      10,
      5)
      .setKinematics(DriveConstants.kDriveKinematics);

    //Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 0),
        new Translation2d(1, -1)
      ),
      new Pose2d(2, -1, new Rotation2d(Math.PI)),
      config);
    
    //PID Controllers for trajectory following
    PIDController xPID = new PIDController(AutoConstants.XController, 0, 0);
    PIDController yPID = new PIDController(AutoConstants.YController, 0, 0);
    ProfiledPIDController thetaPID = new ProfiledPIDController(AutoConstants.ThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    //Constructing command to follow trajectory
    SwerveControllerCommand command = new SwerveControllerCommand(
      trajectory,
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      xPID,
      yPID,
      thetaPID,
      swerveSubsystem::setModuleStates,
      swerveSubsystem
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      command,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
