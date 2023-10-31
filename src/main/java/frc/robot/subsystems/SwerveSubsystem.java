package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystickCMD;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e){}
            zeroHeading();
        }).start();
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

    public void periodic(){
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
