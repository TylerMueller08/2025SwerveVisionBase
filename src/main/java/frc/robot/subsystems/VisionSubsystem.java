package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private SwerveSubsystem swerveSubsystem;
    
    private final PhotonCamera apriltagCamera;
    // private final PhotonCamera objectCamera;

    private final AprilTagFieldLayout tagLayout;
    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        apriltagCamera = new PhotonCamera("AprilTag");
        // objectCamera = new PhotonCamera("ObjectDeetection");

        tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Camera mounted facing forward, half a meter forward of center, half a meter up from center.

        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, apriltagCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> estimatedPoseOpt = photonPoseEstimator.update();
        estimatedPoseOpt.ifPresent(estimatedPose -> swerveSubsystem.addVisionReading(estimatedPoseOpt.get().estimatedPose.toPose2d()));
    }
    
    public PhotonCamera returnCamera() {
        return apriltagCamera;
    }
}
