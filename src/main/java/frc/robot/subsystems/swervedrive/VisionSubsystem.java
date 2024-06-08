package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerveSubsystem;
    
    private final PhotonCamera apriltagCamera = new PhotonCamera("AprilTagDetectionCamera");
    // private final PhotonCamera objectCamera = new PhotonCamera("ObjectDetectionCamera");

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void periodic() {
        estimateLocalPose();

        // Returns the current pose (position and rotation) of the robot, as reported by odometry
        // Results should be relative of the blue alliance driverstation wall
        Pose2d robotPose = swerveSubsystem.getPose();

        System.out.println("<--------------->");
        System.out.println("Swerve X Position: " + robotPose.getX());
        System.out.println("Swerve Y Position: " + robotPose.getY());
        System.out.println("Swerve Rotation: " + robotPose.getRotation().getDegrees());
        System.out.println("<--------------->");
    }

    private void estimateLocalPose() {
        var result = apriltagCamera.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldToCameraPose = result.getMultiTagResult().estimatedPose.best;

            double xPosition = fieldToCameraPose.getX();
            double yPosition = fieldToCameraPose.getY(); // Z component denotes depth, which is not needed.
            Rotation2d rotation = new Rotation2d(fieldToCameraPose.getRotation().getZ()); // Z component denotes yaw.

            swerveSubsystem.swerveDrive.addVisionMeasurement(new Pose2d(xPosition, yPosition, rotation), Timer.getFPGATimestamp());
        }
    }
    
    public PhotonCamera returnCamera() {
        return apriltagCamera;
    }
}
