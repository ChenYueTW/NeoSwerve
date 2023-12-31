package frc.robot.Auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDrive extends SequentialCommandGroup{
    private final PIDConstants movePid;
    private final PIDConstants rotationPid;
    private final SwerveSubsystem swerveSubsystem;
    private final PathPlannerPath path;

    public AutoDrive(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.movePid = new PIDConstants(4.0, 0, 0);
        this.rotationPid = new PIDConstants(3.0, 0, 0);
        this.path = PathPlannerPath.fromPathFile("SwervePath");

        addCommands(this.autoDrive());
    }

    public Command autoDrive() {
        return new FollowPathWithEvents(
            new FollowPathHolonomic(
                this.path,
                this.swerveSubsystem::getPose,
                this.swerveSubsystem::getSpeeds,
                this.swerveSubsystem::autoDriveSwerve,
                new HolonomicPathFollowerConfig(
                    this.movePid,
                    this.rotationPid,
                    AutoConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
                    SwerveConstants.TRACK_WIDTH / 2,
                    new ReplanningConfig()
                ),
                this.swerveSubsystem
            ),
            this.path,
            this.swerveSubsystem::getPose
        );
    }
}
