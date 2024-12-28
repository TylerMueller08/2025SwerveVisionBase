package frc.robot.commands.auton;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutonUtils
{
    /**
     * Field length in meters
     */
    private static final double FIELD_LENGTH = 16.54;

    /**
     * Robot Constraints for Pathfinding
     */
    public static final PathConstraints CONSTRAINTS = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    /**
     * Robot Configuration for generating trajectories
     */
    public static RobotConfig robotConfig;

    static {
        ModuleConfig moduleConfig = new ModuleConfig(0.0508, 3.0, 1.19, DCMotor.getKrakenX60(1), 60.0, 1);

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.err.println("Error loading robot configuration from GUI settings. Using default values.");
            e.printStackTrace();
            robotConfig = new RobotConfig(27.2155, 6.883, moduleConfig, 0.530225);
        }
    }
    
    public static Command resetOdometry(PathPlannerPath choreoPath)
    {
        return RobotContainer.drivebase.runOnce(
            () -> {
                Pose2d pose = choreoPath
                    .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), robotConfig)
                    .getInitialPose();

            if (RobotContainer.drivebase.isRedAlliance())
            {
                pose = flipFieldPose(pose);
            }

            RobotContainer.drivebase.resetOdometry(pose);
        });
    }

    /**
     * Load the PathPlanner trajectory file to path
     * @param pathName Name of the path
     * @return PathPlanner Path
     */
    public static PathPlannerPath loadPath(String pathName)
    {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.err.println("Failed to load path: " + pathName);
            e.printStackTrace();
        }

        // Return null or provide default fallback PathPlannerPath
        return null;
    }

    /**
     * Flip a field position to the other side of the field, maintaining a blue alliance origin
     * 
     * @param position The position to flip
     * @return The flipped position
     */
    private static Translation2d flipFieldPosition(Translation2d position)
    {
        return new Translation2d(FIELD_LENGTH - position.getX(), position.getY());
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
     * 
     * @param rotation The rotation to flip
     * @return The flipped rotation
     */
    private static Rotation2d flipFieldRotation(Rotation2d rotation)
    {
        return new Rotation2d(Math.PI).minus(rotation);
    }
    
    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin
     * 
     * @param pose The pose to flip
     * @return The flipped pose
     */
    private static Pose2d flipFieldPose(Pose2d pose)
    {
        return new Pose2d(flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
    }
}
