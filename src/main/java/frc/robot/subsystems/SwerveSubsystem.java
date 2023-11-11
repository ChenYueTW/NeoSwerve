package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorId.Neo;
import frc.robot.MotorId.Encoder;
import frc.robot.Constants.MotorReverse;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;

    public SwerveSubsystem() {
        this.frontLeft = new SwerveModule(
            Neo.frontLeftDrive,
            Neo.frontLeftTurn,
            Encoder.frontLeft,
            MotorReverse.FRONT_LEFT_DRIVE,
            MotorReverse.FRONT_LEFT_TURN
        );
        this.frontRight = new SwerveModule(
            Neo.frontRightDrive,
            Neo.frontRightTurn,
            Encoder.frontRight,
            MotorReverse.FRONT_RIGHT_DRIVE,
            MotorReverse.FRONT_RIGHT_TURN
        );
        this.backLeft = new SwerveModule(
            Neo.backwardLeftDrive,
            Neo.backwardLeftTurn,
            Encoder.backwardLeft,
            MotorReverse.BACK_LEFT_DRIVE,
            MotorReverse.BACK_LEFT_TURN
        );
        this.backRight = new SwerveModule(
            Neo.backwardRightDrive,
            Neo.backwardRightTurn,
            Encoder.backwardRight,
            MotorReverse.BACK_RIGHT_DRIVE,
            MotorReverse.BACK_RIGHT_TURN
        );
        this.gyro = new AHRS(SPI.Port.kMXP);
    }

    public void driveSwerve(double xSpeed, double ySpeed,double rotation) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d())
        );
        this.setModuleState(state);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }
}