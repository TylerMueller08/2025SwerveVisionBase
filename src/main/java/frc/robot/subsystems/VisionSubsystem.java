package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

    private SwerveSubsystem swerveSubsystem;
    
    private final PhotonCamera apriltagCamera;
    // private final PhotonCamera objectCamera;

    private final AprilTagFieldLayout tagLayout;
    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        apriltagCamera = new PhotonCamera("AprilTagDetectionCamera");
        // objectCamera = new PhotonCamera("ObjectDeetectionCamera");

        tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Camera mounted facing forward, half a meter forward of center, half a meter up from center.

        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, apriltagCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        // Simulation
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);

            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(apriltagCamera, cameraProp);
            visionSim.addCamera(cameraSim, robotToCamera);
            cameraSim.enableDrawWireframe(true);
        }
    }

    public void simulationPeriodic() {
        visionSim.update(swerveSubsystem.getPose());
        visionSim.getDebugField(); // localhost:1182
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
