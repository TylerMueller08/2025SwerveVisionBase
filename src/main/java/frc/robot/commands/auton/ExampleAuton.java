package frc.robot.commands.auton;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class ExampleAuton extends AutonCommand {
    private final List<PathPlannerPath> paths;

    public ExampleAuton() {
        paths = List.of(
            AutonUtils.loadPath("Path1"),
            AutonUtils.loadPath("Path2"),
            AutonUtils.loadPath("Path3"),
            AutonUtils.loadPath("Path4"),
            AutonUtils.loadPath("Path5"),
            AutonUtils.loadPath("Path6")
        );

        if (Robot.isSimulation()) {
            addCommands(AutonUtils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                AutoBuilder.followPath(paths.get(0)),
                AutoBuilder.followPath(paths.get(1)),
                AutonUtils.aimAtSpeaker(),
                AutoBuilder.pathfindThenFollowPath(paths.get(2), AutonUtils.CONSTRAINTS),
                AutoBuilder.followPath(paths.get(3)),
                AutonUtils.aimAtSpeaker(),
                AutoBuilder.pathfindThenFollowPath(paths.get(4), AutonUtils.CONSTRAINTS),
                AutoBuilder.followPath(paths.get(5)),
                AutonUtils.aimAtSpeaker()
            )
        );
    }

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.subList(0, 5).stream()
            .map(PathPlannerPath::getPathPoses)
            .flatMap(Collection::stream)
            .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose() {
        return paths.get(0)
            .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), AutonUtils.robotConfig)
            .getInitialPose();
    }
}
