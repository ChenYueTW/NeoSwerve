package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;;

public class RobotContainer {
	private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(swerveSubsystem, joystick);

	private final PathPlannerTrajectory trajectory = PathPlanner.loadPath(
		"SwervePath",
		AutoConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
		AutoConstants.PHYSICAL_MAX_ACCELERATION_METERS_PER_SECONE
	);
	private final PIDController xPid = new PIDController(0.01, 0, 0);
	private final PIDController yPid = new PIDController(0.01, 0, 0);
	private final PIDController rotationPid = new PIDController(0.01, 0, 0);

	private final PPSwerveControllerCommand autoPath = new PPSwerveControllerCommand(
		this.trajectory,
		this.swerveSubsystem::getPose,
		this.xPid,
		this.yPid,
		this.rotationPid,
		this.swerveSubsystem::setAutoModuleState,
		this.swerveSubsystem
	);

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
	}

	public Command getAutonomousCommand() {
		return new SequentialCommandGroup(
			this.autoPath,
			Commands.run(this.swerveSubsystem::stopModules, this.swerveSubsystem)
		);
	}
}
