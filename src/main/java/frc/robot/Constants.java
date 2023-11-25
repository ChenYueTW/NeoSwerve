package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
	public static final class SwerveConstants {
		public static final double TRACK_WIDTH = 0.62; // 寬
		public static final double WHEEL_BASE = 0.62; // 長
		public static final double WHEEL_RADIUS = 0.0508; // 輪子半徑
		
		public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.0; // 最大速度m/s
		public static final double PHYSICAL_MAX_ACCELERATION_METERS_PER_SECONE = 3.0; // 最大加速度m/s^2
		public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0; // 齒輪比

		public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = WHEEL_RADIUS / DRIVE_GEAR_RATIO * Math.PI;
		public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_RADIUS / DRIVE_GEAR_RATIO / 60 * Math.PI;
	}
	
	public static final class MotorReverse {
		public static final boolean FRONT_LEFT_DRIVE = true;
		public static final boolean FRONT_RIGHT_DRIVE = false;
		public static final boolean BACK_LEFT_DRIVE = true;
		public static final boolean BACK_RIGHT_DRIVE = false;

		public static final boolean FRONT_LEFT_TURN = true;
		public static final boolean FRONT_RIGHT_TURN = true;
		public static final boolean BACK_LEFT_TURN = true;
		public static final boolean BACK_RIGHT_TURN = true;
	}

	public static final class EncoderOffset {
		public static final double FRONT_LEFT = 122.34375;
		public static final double FRONT_RIGHT = 89.208984375;
		public static final double BACK_LEFT = 271.7578125;
		public static final double BACK_RIGHT = 211.640625;
	}

	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2)
    );

	public static final class AutoConstants {
		public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 1.0; // 最大速度m/s
		public static final double PHYSICAL_MAX_ACCELERATION_METERS_PER_SECONE = 0.5; // 最大加速度m/s^2
	}

    public static final double DEAD_BAND = 0.05;
	public static final double MAX_SPEED = 1.5;
	public static final double MAX_ANGULAR_SPEED = 1.5;
}
