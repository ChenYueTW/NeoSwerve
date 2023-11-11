package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class SwerveDriveCmd extends CommandBase {
	private final SwerveSubsystem swerveSubsystem;
	private final XboxController controller;

	public SwerveDriveCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
		this.swerveSubsystem = swerveSubsystem;
		this.controller = controller;
		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double xSpeed = MathUtil.applyDeadband(this.controller.getLeftX(), Constants.DEAD_BAND);
		double ySpeed = MathUtil.applyDeadband(this.controller.getLeftY(), Constants.DEAD_BAND);
		double rotation = MathUtil.applyDeadband(this.controller.getRightY(), Constants.DEAD_BAND);

		this.swerveSubsystem.driveSwerve(xSpeed, ySpeed, rotation);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}