package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCMD;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveMotorPort,
        DriveConstants.frontLeftTurningMotorPort,
        false, 
        false,
        DriveConstants.frontLeftAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveMotorPort,
        DriveConstants.frontRightTurningMotorPort,
        false,
        false,
        DriveConstants.frontRightAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveMotorPort,
        DriveConstants.backLeftTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.75, 0, 0);
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveMotorPort,
        DriveConstants.backRightTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.75, 0, 0);

    private final SwerveModuleState[] lockedStates = {
            new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0)),//Front left
            new SwerveModuleState(0, new Rotation2d(Math.PI/4.0)),//Front right
            new SwerveModuleState(0, new Rotation2d(Math.PI/4.0)),//Back left
            new SwerveModuleState(0, new Rotation2d(-Math.PI/4.0))//Back right
          };

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

    public boolean locked;

    private final SwerveDriveOdometry odometer;
    private final SwerveModulePosition[] states = new SwerveModulePosition[4];

    public SwerveSubsystem(){
        states[0] = frontLeft.getPosition();
        states[1] = frontRight.getPosition();
        states[2] = backLeft.getPosition();
        states[3] = backRight.getPosition();
        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), states);

        locked = false;
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e){}
            zeroHeading();
        }).start();

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0),
                new PIDConstants(5.0, 0, 0),
                4.5,
                0.4,
                new ReplanningConfig()
            ), 
            this
        );
    }

    public SwerveModule getFL(){
        return frontLeft;
    }
    public SwerveModule getFR(){
        return frontRight;
    }
    public SwerveModule getBL(){
        return backLeft;
    }
    public SwerveModule getBR(){
        return backRight;
    }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        return states;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroHeading(){
        gyro.reset();
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }


    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), states, pose); //States - potential issue?
    }


    public void periodic(){
        states[0] = frontLeft.getPosition();
        states[1] = frontRight.getPosition();
        states[2] = backLeft.getPosition();
        states[3] = backRight.getPosition();
        odometer.update(getRotation2d(), states);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left Rotation", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right Rotation", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left Rotation", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right Rotation", backRight.getTurnPosition());

        SmartDashboard.putNumber("Front Left Turn Output", frontLeft.getTurnMotorOutput());
        SmartDashboard.putNumber("Front Right Turn Output", frontRight.getTurnMotorOutput());
        SmartDashboard.putNumber("Back Left Turn Output", backLeft.getTurnMotorOutput());
        SmartDashboard.putNumber("Back Right Turn Output", backRight.getTurnMotorOutput());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public void setSpeed(double speed){
        frontLeft.setPercentSpeed(speed);
        frontRight.setPercentSpeed(speed);
        backLeft.setPercentSpeed(speed);
        backRight.setPercentSpeed(speed);
    }
    public void setModuleStates(Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
        // SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
        // SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
        // SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationUnitsPerSecond);

        if(locked){
            frontLeft.setDesiredState(lockedStates[0]);
            frontRight.setDesiredState(lockedStates[1]);
            backLeft.setDesiredState(lockedStates[2]);
            backRight.setDesiredState(lockedStates[3]);
            return;
        }
        else{
          //Gets real time joystick input
          double xSpeed = xSpdFunction.get();
          double ySpeed = ySpdFunction.get();
          double turningSpeed = turningSpdFunction.get();

      
          //Deadband
          xSpeed = Math.abs(xSpeed) > OperatorConstants.deadband ? xSpeed : 0.0;
          ySpeed = Math.abs(ySpeed) > OperatorConstants.deadband ? ySpeed : 0.0;
          turningSpeed = Math.abs(turningSpeed) > OperatorConstants.deadband ? turningSpeed : 0.0;

      
          //Limits rate - if pushing drive stick too fast, robot will still accelerate slowly
          //xSpeed = xLimiter.calculate(xSpeed);
          //ySpeed = yLimiter.calculate(ySpeed);
          //turningSpeed = turningLimiter.calculate(turningSpeed);
      
          ChassisSpeeds chassisSpeeds;
          if(fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, getRotation2d());
          }
          else{
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
          }

          driveRobotRelative(chassisSpeeds);
        }
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){ //May not need this function since it's already converted conditionally
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getRotation2d()));
    }
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    
    // public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    //     driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    //   }
    
    //   public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
    //     SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    //     frontLeft.setDesiredState(targetStates[0]);
    //     frontRight.setDesiredState(targetStates[1]);
    //     backLeft.setDesiredState(targetStates[2]);
    //     backRight.setDesiredState(targetStates[3]);
    //   }
}
