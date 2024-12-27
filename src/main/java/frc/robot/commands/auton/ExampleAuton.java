package frc.robot.commands.auton;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ExampleAuton extends AutonCommand
{
    private final PathPlannerPath path1;
    private final PathPlannerPath path2;
    private final PathPlannerPath path3;
    private final PathPlannerPath path4;

    public ExampleAuton()
    {
        // path1 = AutonUtils.loadPath("Path1");
        path1 = AutonUtils.loadPath("Path1");
        path2 = AutonUtils.loadPath("Path2");
        path3 = AutonUtils.loadPath("Path3");
        path4 = AutonUtils.loadPath("Path4");

        if (Robot.isSimulation())
        {
            addCommands(AutonUtils.resetOdometry(path1));
        }

        addCommands(
            Commands.sequence(
                AutoBuilder.followPath(path1),
                AutoBuilder.followPath(path2),
                RobotContainer.drivebase.aimAtSpeaker(0.1),
                RobotContainer.drivebase.driveToPose(
                    new Pose2d(8.1, 5.75, Rotation2d.fromDegrees(180))
                ),
                AutoBuilder.followPath(path3),
                RobotContainer.drivebase.aimAtSpeaker(0.1),
                RobotContainer.drivebase.driveToPose(
                    new Pose2d(8.1, 4.1, Rotation2d.fromDegrees(180))
                ),
                AutoBuilder.followPath(path4),
                RobotContainer.drivebase.aimAtSpeaker(0.1)
            )
        );
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Stream.of(
            path1.getPathPoses(),
            path2.getPathPoses(),
            path3.getPathPoses(),
            path4.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose()
    {
        return path1
            .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), AutonUtils.robotConfig)
            .getInitialPose(); // Should be getInitialTargetHolonomicPose(), deprecated?
    }
}
