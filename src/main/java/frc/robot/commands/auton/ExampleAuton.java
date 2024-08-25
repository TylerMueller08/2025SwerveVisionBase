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

public class ExampleAuton extends AutoCommand {

    private final PathPlannerPath path1;
    private final PathPlannerPath path2;
    private final PathPlannerPath path3;
    private final PathPlannerPath path4;

    public ExampleAuton() {
        path1 = PathPlannerPath.fromChoreoTrajectory("Path1");
        path2 = PathPlannerPath.fromChoreoTrajectory("Path2");
        path3 = PathPlannerPath.fromChoreoTrajectory("Path3");
        path4 = PathPlannerPath.fromChoreoTrajectory("Path4");

        if (Robot.isSimulation()) {
            addCommands(AutoBuildingBlocks.resetOdometry(path1));
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
    public List<Pose2d> getAllPathPoses() {
        return Stream.of(
            path1.getPathPoses(),
            path2.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose() {
        return path1
            .getTrajectory(new ChassisSpeeds(), new Rotation2d())
            .getInitialTargetHolonomicPose();
    }
}
