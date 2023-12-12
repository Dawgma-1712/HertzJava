package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
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

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

    private double FLTarget, FRTarget, BLTarget, BRTarget;

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

        /*AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0),
                new PIDConstants(5.0, 0, 0),
                4.5,
                0.4,
                new ReplanningConfig()
            ), 
            this
        );*/
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

        SmartDashboard.putNumber("Front Left Target Rotation", FLTarget);
        SmartDashboard.putNumber("Front Right Target Rotation", FRTarget);
        SmartDashboard.putNumber("Back Left Target Rotation", BLTarget);
        SmartDashboard.putNumber("Back Right Target Rotation", BRTarget);

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
    public void setModuleStates(SwerveModuleState[] states){
        FLTarget = states[0].angle.getRadians();
        FRTarget = states[1].angle.getRadians();
        BLTarget = states[2].angle.getRadians();
        BRTarget = states[3].angle.getRadians();

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
}
