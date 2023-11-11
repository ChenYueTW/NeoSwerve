package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turnEncoder;

    private final PIDController turnPidController;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    public SwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, boolean driveMotorReverse, boolean turnMotorReverse, String motorName) {
        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new CANCoder(turnEncoderPort);
    
        this.driveMotor.setInverted(driveMotorReverse);
        this.turnMotor.setInverted(turnMotorReverse);

        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.turnMotor.setIdleMode(IdleMode.kBrake);

        this.driveMotor.setSmartCurrentLimit(30);
        this.turnMotor.setSmartCurrentLimit(30);

        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.turnPidController = new PIDController(0, 0, 0);
        this.turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        this.motorName = motorName;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(), Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveEncoder.getPosition(), Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond;
        this.turnOutput = this.turnPidController.calculate(this.getState().angle.getDegrees(), this.getState().angle.getDegrees());

        this.setEncoderDashboard();
        this.setMotorDashboard();

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    public void setEncoderDashboard() {
        SmartDashboard.putNumber(motorName + " DrivePosition", this.driveEncoder.getPosition());
        SmartDashboard.putNumber(motorName + " DriveVelocity", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber(motorName + " TurnPosition", this.turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(motorName + " TurnVelocity", this.turnEncoder.getVelocity());
    }

    public void setMotorDashboard() {
        SmartDashboard.putNumber(motorName + " DriveMotor", this.driveOutput);
        SmartDashboard.putNumber(motorName + " TurnMotor", this.turnOutput);
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turnMotor.set(0);
    }
}
