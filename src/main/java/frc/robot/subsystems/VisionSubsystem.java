package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

public class VisionSubsystem {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  /**
   * Count of times that the odm thinks we're more than 10 meters away from the April Tag.
   */
  private double longDistangePoseEstimationCount = 0;

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;

  /**
   * Ambiguity defined as a value between (0,1). Used in {@link VisionSubsystem#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, getEstimationStdDevs(camera));
      }
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    Optional<EstimatedRobotPose> poseEst = filterPose(camera.poseEstimator.update());
    return poseEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link VisionSubsystem#getEstimatedGlobalPose(Cameras)}, for use with
   * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should only be used
   * when there are targets visible.
   *
   * @param camera Desired camera to get the standard deviation of the estimated pose.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Cameras camera)
  {
    var    poseEst    = getEstimatedGlobalPose(camera);
    var    estStdDevs = camera.singleTagStdDevs;
    var    targets    = getLatestResult(camera).getTargets();
    int    numTags    = 0;
    double avgDist    = 0;

    for (var tgt : targets)
    {
      var tagPose = camera.poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) 
      {
        continue;
      }
      numTags++;

      if (poseEst.isPresent())
      {
        avgDist += PhotonUtils.getDistanceToPose(poseEst.get().estimatedPose.toPose2d(), tagPose.get().toPose2d());
      }
    }

    if (numTags == 0)
    {
      return estStdDevs;
    }
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
    {
      estStdDevs = camera.multiTagStdDevs;
    }

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
    {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else
    {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }

      // ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity)
      {
        return Optional.empty();
      }

      // est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        // if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get the latest result from a given Camera.
   *
   * @param camera Given camera to take the result from.
   * @return Photon result from sim or a real camera.
   */
  public PhotonPipelineResult getLatestResult(Cameras camera)
  {
    return camera.camera.getLatestResult();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget  target = null;
    PhotonPipelineResult result = getLatestResult(camera);
    if (result.hasTargets())
    {
      for (PhotonTrackedTarget i : result.getTargets())
      {
        if (i.getFiducialId() == id) {
          target = i;
        }
      }
    }
    return target;
  }

  /**
   * Update the {@link Field2d} to include tracked targets.
   */
  public void updateVisionField()
  {
    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values())
    {
      if (getLatestResult(c).hasTargets())
      {
        targets.addAll(getLatestResult(c).targets);
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  enum Cameras
  {
    /**
     * Primary AprilTag Camera
     */
    APRILTAG_CAM("OV9281",
               new Rotation3d(0, Units.degreesToRadians(0), Math.toRadians(0)),
               new Translation3d(0.5,
                                 0.0,
                                 0.5),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));


    // Latency alert to use when high latency is detected.
    public final  Alert               latencyAlert;
    
    // Camera instance for comms.
    public final  PhotonCamera        camera;
    
    // Pose estimator for camera.
    public final  PhotonPoseEstimator poseEstimator;
    public final  Matrix<N3, N1>      singleTagStdDevs;
    public final  Matrix<N3, N1>      multiTagStdDevs;
    
    // Transform of the camera rotation and translation relative to the center of the robot
    private final Transform3d         robotToCamTransform;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.WARNING);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(VisionSubsystem.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              camera,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
    }
  }
}