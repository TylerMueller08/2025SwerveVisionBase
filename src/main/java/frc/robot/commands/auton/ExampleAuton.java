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

    public ExampleAuton()
    {
        path1 = AutonUtils.loadPath("Path1");
        path2 = AutonUtils.loadPath("Path2");
        path3 = AutonUtils.loadPath("Path3");

        if (Robot.isSimulation())
        {
            addCommands(AutonUtils.resetOdometry(path1));
        }

        addCommands(
            Commands.sequence(
                AutoBuilder.followPath(path1),
                AutoBuilder.followPath(path2),
                RobotContainer.drivebase.aimAtSpeaker(0.1),
                AutoBuilder.pathfindThenFollowPath(path3, AutonUtils.CONSTRAINTS)
            )
        );
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Stream.of(
            path1.getPathPoses(),
            path2.getPathPoses(),
            path3.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose()
    {
        return path1
            .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), AutonUtils.robotConfig)
            .getInitialPose();
    }
}
