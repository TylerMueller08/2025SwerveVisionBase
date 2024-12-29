package frc.robot.commands.auton.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutonCommand extends SequentialCommandGroup
{
    public abstract List<Pose2d> getAllPathPoses();
    public abstract Pose2d getStartingPose();
}
