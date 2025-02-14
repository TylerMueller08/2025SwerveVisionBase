package frc.robot.commands.auton.utils;

import java.util.ArrayList;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonUtils {
    /** Robot Constraints for Pathfinding. */
    public static final PathConstraints CONSTRAINTS = new PathConstraints(Constants.RobotConfiguration.MAX_DRIVE_VELOCITY, Constants.RobotConfiguration.MAX_DRIVE_ACCELERATION, Constants.RobotConfiguration.MAX_ANGULAR_VELOCITY, Constants.RobotConfiguration.MAX_ANGULAR_ACCELERATION);

    /** Robot Configuration for generating trajectories. */
    public static RobotConfig robotConfig;

    static {
        ModuleConfig moduleConfig = new ModuleConfig(Constants.ModuleConfiguration.WHEEL_RADIUS, Constants.RobotConfiguration.MAX_DRIVE_VELOCITY, Constants.ModuleConfiguration.WHEEL_COF, DCMotor.getKrakenX60(1), 60.0, 1);

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.err.println("Error loading robot configuration from GUI settings. Using default values.");
            e.printStackTrace();
            robotConfig = new RobotConfig(Constants.ROBOT_MASS, Constants.ModuleConfiguration.MOI, moduleConfig, Constants.ModuleConfiguration.TRACK_WIDTH);
        }
    }
    
    public static Command resetOdometry(PathPlannerPath choreoPath) {
        return RobotContainer.drivebase.runOnce(
            () -> {
                Pose2d pose = choreoPath
                    .generateTrajectory(new ChassisSpeeds(), new Rotation2d(Math.PI), robotConfig)
                    .getInitialPose();

            if (RobotContainer.drivebase.isRedAlliance()) {
                pose = flipFieldPose(pose);
            }

            RobotContainer.drivebase.resetOdometry(pose);
        });
    }

    public static Command aimAtSpeaker() {
        return RobotContainer.drivebase.aimAtSpeaker(1.0);
    }

    /**
     * Load the PathPlanner trajectory file to path.
     * @param pathName Name of the path.
     * @return PathPlanner Path.
     */
    public static PathPlannerPath loadPath(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.err.println("Failed to load path: " + pathName);
            e.printStackTrace();
        }

        // Return an empty path as fallback.
        return generateEmptyPath();
    }

    /** Result an empty path with zero constraints and no waypoints. */
    private static PathPlannerPath generateEmptyPath() {
        return new PathPlannerPath(
            new ArrayList<>(),
            new PathConstraints(0, 0, 0, 0),
            new IdealStartingState(0, new Rotation2d()),
            new GoalEndState(0, new Rotation2d())
        );
    }

    /**
     * Flip a field position to the other side of the field, maintaining a blue alliance origin.
     * @param position The position to flip.
     * @return The flipped position.
     */
    private static Translation2d flipFieldPosition(Translation2d position) {
        return new Translation2d(Constants.FIELD_LENGTH - position.getX(), position.getY());
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin.
     * @param rotation The rotation to flip.
     * @return The flipped rotation.
     */
    private static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }
    
    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin.
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    private static Pose2d flipFieldPose(Pose2d pose) {
        return new Pose2d(flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
    }
}
