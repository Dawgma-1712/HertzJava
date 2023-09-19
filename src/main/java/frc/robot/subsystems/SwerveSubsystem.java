package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveMotorPort,
        DriveConstants.frontLeftTurningMotorPort,
        false, 
        false,
        DriveConstants.frontLeftAbsoluteEncoder, 0.17, 0, 0);
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveMotorPort,
        DriveConstants.frontRightTurningMotorPort,
        false,
        false,
        DriveConstants.frontRightAbsoluteEncoder, 0.17, 0, 0);
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveMotorPort,
        DriveConstants.backLeftTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.17, 0, 0);
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveMotorPort,
        DriveConstants.backRightTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.13, 0, 0);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e){}
            zeroHeading();
        }).start();
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
        SmartDashboard.putNumber("Robot Heading", getHeading());
        
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public void setModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
}
