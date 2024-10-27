# VisionSubsystem
Class package: `frc.robot.subsystems`

## Purpose
The `VisionSubsystem` class integrates AprilTag-based vision data and field layouts, utilizing PhotonVision to provide accurate positioning information for the robot on the field. It uses both actual and simulated PhotonVision cameras to estimate the robot’s pose, facilitate autonomous navigation, and optimize camera-based position tracking with real-time pose estimations and target distance calculations.

## Public Fields:
- `fieldLayout`: An `AprilTagFieldLayout` object representing the April Tag field layout for the specified year.
- `visionSim`: A `VisionSystemSim` instance used for simulating vision in a simulated environment.

## Public Methods

### `public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field)`
- **Description:** Constructs a `VisionSubsystem` instance, initializing necessary vision simulation components and setting up AprilTag layouts for use in simulations.
- **Parameters**:
  - `currentPose`: A `Supplier<Pose2d>` providing the current pose, typically retrieved from `SwerveDrive`.
  - `field`: A `Field2d` object representing the field visualization.

---

### `public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)`
- **Description:** Computes the pose of a specified AprilTag relative to the robot, allowing the robot to position itself accurately with respect to the AprilTag.
- **Parameters**:
  - `aprilTag`: The integer ID of the desired AprilTag.
  - `robotOffset`: A `Transform2d` representing the robot's positional offset relative to the tag.
- **Returns**: `Pose2d` representing the target pose adjusted for the robot's offset.

---

### `public void updatePoseEstimation(SwerveDrive swerveDrive)`
- **Description:** Updates the robot's pose estimation in `SwerveDrive` based on the latest PhotonVision measurements. It adjusts the estimate with multiple camera data when available.

---

### `public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)`
- **Description:** Retrieves the robot's global pose estimate from the specified camera, if available. Returns empty if the estimate is considered inaccurate.
- **Parameters**:
  - `camera`: A `Cameras` enum value representing the specific camera to use for the pose estimate.
- **Returns**: `Optional<EstimatedRobotPose>` containing the estimated robot pose, timestamp, and target data if the estimate is valid.

---

### `public Matrix<N3, N1> getEstimationStdDevs(Cameras camera)`
- **Description:** Calculates the standard deviation of the estimated pose, factoring in the number of tags detected and the average distance to targets, providing variance data for pose estimations.
- **Parameters**:
  - `camera`: A `Cameras` enum value representing the desired camera for standard deviation calculations.
- **Returns**: `Matrix<N3, N1>` containing the pose standard deviations.

---

### `public PhotonPipelineResult getLatestResult(Cameras camera)`
- **Description:** Retrieves the latest image processing result from a specified camera, supporting both real-world and simulation environments.
- **Parameters**:
  - `camera`: A `Cameras` enum value representing the desired camera.
- **Returns**: `PhotonPipelineResult` containing the latest detection results.

---

### `public double getDistanceFromAprilTag(int id)`
- **Description:** Calculates the distance between the robot and a specific AprilTag based on the current pose.
- **Parameters**:
  - `id`: The integer ID of the AprilTag.
- **Returns**: `double` representing the distance to the specified AprilTag. Returns `-1.0` if the tag pose is unavailable.

---

### `public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)`
- **Description:** Retrieves a tracked target corresponding to a specified AprilTag ID from a given camera, if available.
- **Parameters**:
  - `id`: The integer ID of the AprilTag.
  - `camera`: A `Cameras` enum value representing the camera to search.
- **Returns**: `PhotonTrackedTarget` representing the tracked target if found; otherwise, null.

---

### `public VisionSystemSim getVisionSim()`
- **Description:** Returns the `VisionSystemSim` instance used for simulation purposes.
- **Returns**: `VisionSystemSim` instance.

---

### `public void updateVisionField()`
- **Description:** Updates the field visualization (`Field2d`) with the tracked target poses, enabling visualization of the robot’s position and detected targets on the field.

## Enums

### `Cameras`
- **Description:** An enumeration of available cameras used for AprilTag detection and vision tracking. The enum defines each camera’s properties and provides methods for simulation-specific setup.

- **Values**:
  - `APRILTAG_CAM`: The primary AprilTag camera, including rotation, translation, and noise properties for simulation and tracking.