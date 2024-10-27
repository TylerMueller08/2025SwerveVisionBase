# SwerveSubsystem
Class package: `frc.robot.subsystems`

## Purpose
The `SwerveSubsystem` class is responsible for managing a swerve-drive robot, enabling field-oriented and robot-oriented driving, path-following with PathPlanner integration, and pose estimation using PhotonVision. This subsystem interacts with the VisionSubsystem, AprilTag layout, and various controllers to facilitate precise navigation on the field.

## Public Methods

### `public SwerveSubsystem(File directory)`
- **Description:** Initializes the swerve drive and PhotonVision systems using a directory of swerve drive configuration files.
- **Parameters**:
  - `directory`: File - The directory containing swerve drive configuration files.

---

### `public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)`
- **Description:** Alternative constructor that initializes the swerve drive with provided drive and controller configurations.
- **Parameters**:
  - `driveCfg`: SwerveDriveConfiguration - Configuration for the swerve drive.
  - `controllerCfg`: SwerveControllerConfiguration - Configuration for the swerve controller.

---

### `public void setupPhotonVision()`
- **Description**: Initializes the VisionSubsystem for tracking and updating the robot’s pose estimation using PhotonVision.

---

### `public void updatePoseWithVision()`
- **Description**: Updates the pose estimation with the latest vision data from the VisionSubsystem.

---

### `public Pose2d getVisionPose()`
- **Description**: Returns the robot's pose after updating with vision estimates.
- **Returns**: Pose2d - The updated robot pose based on vision data.

---

### `public void setupPathPlanner()`
- **Description**: Configures the PathPlanner's AutoBuilder for holonomic path-following with the robot's kinematic and drive configurations.

---

### `public double getDistanceToSpeaker()`
- **Description**: Calculates the distance from the robot to the speaker using AprilTag data.
- **Returns**: double - The distance to the speaker in meters.

---

### `public Rotation2d getSpeakerYaw()`
- **Description**: Calculates the yaw angle needed for the robot to aim at the speaker.
- **Returns**: Rotation2d - The yaw angle required to aim at the speaker.

---

### `public Command aimAtSpeaker(double tolerance)`
- **Description**: Provides a command for the robot to turn and aim at the speaker within a specified tolerance.
- **Parameters**:
  - `tolerance`: double - The angle tolerance, in degrees, within which the robot should aim at the speaker.
- **Returns**: Command - A command for the robot to turn to face the speaker.

---

### `public Command driveToPose(Pose2d pose)`
- **Description**: Creates a path-finding command to drive the robot to a specified pose on the field.
- **Parameters**:
  - `pose`: Pose2d - The target pose for the robot to reach.
- **Returns**: Command - A command that drives the robot to the target pose.

---

### `public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)`
- **Description**: Provides a command to drive the robot with translational and angular velocities, updated with pose estimation.
- **Parameters**:
  - `translationX`: DoubleSupplier - Translation speed in the X direction.
  - `translationY`: DoubleSupplier - Translation speed in the Y direction.
  - `angularRotationX`: DoubleSupplier - Angular velocity for rotation.
- **Returns**: Command - The command to drive the robot with specified speeds and rotation.

---

### `public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, BooleanSupplier doAim, PhotonCamera camera)`
- **Description**: Aims the robot using vision data while driving, integrating PhotonVision for targeting AprilTags.
- **Parameters**:
  - `translationX`: DoubleSupplier - Translation speed in the X direction.
  - `translationY`: DoubleSupplier - Translation speed in the Y direction.
  - `angularRotationX`: DoubleSupplier - Angular velocity for rotation.
  - `doAim`: BooleanSupplier - If true, enables aiming at targets.
  - `camera`: PhotonCamera - The camera for vision-based aiming.
- **Returns**: Command - Command for driving and aiming simultaneously.

---

### `public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)`
- **Description**: A simulation-friendly drive command for controlling the robot’s position and orientation with specified translations and rotation.
- **Parameters**:
  - `translationX`: DoubleSupplier - Translation speed in the X direction.
  - `translationY`: DoubleSupplier - Translation speed in the Y direction.
  - `rotation`: DoubleSupplier - Desired rotation in radians.
- **Returns**: Command - The command to simulate driving.

---

### `public void drive(Translation2d translation, double rotation, boolean fieldRelative)`
- **Description**: Drives the robot with specified translation and rotation velocities, optionally in field-relative mode.
- **Parameters**:
  - `translation`: Translation2d - The commanded linear velocity.
  - `rotation`: double - Angular rate in radians per second.
  - `fieldRelative`: boolean - True for field-relative mode, false for robot-relative mode.

---

### `public void driveFieldOriented(ChassisSpeeds velocity)`
- **Description**: Drives the robot using chassis field-oriented velocity.
- **Parameters**:
  - `velocity`: ChassisSpeeds - The velocity according to the field orientation.

---

### `public void drive(ChassisSpeeds velocity)`
- **Description**: Drives the robot using robot-oriented chassis speeds.
- **Parameters**:
  - `velocity`: ChassisSpeeds - The robot-oriented velocity.

---

### `public SwerveDriveKinematics getKinematics()`
- **Description**: Returns the swerve drive kinematics object.
- **Returns**: SwerveDriveKinematics - The swerve drive kinematics.

---

### `public void resetOdometry(Pose2d initialHolonomicPose)`
- **Description**: Resets the robot's odometry to a specified pose.
- **Parameters**:
  - `initialHolonomicPose`: Pose2d - The pose to reset odometry to.

---

### `public Pose2d getPose()`
- **Description**: Retrieves the current pose of the robot as reported by odometry.
- **Returns**: Pose2d - The current robot pose.

---

### `public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)`
- **Description**: Sets chassis speeds using closed-loop velocity control.
- **Parameters**:
  - `chassisSpeeds`: ChassisSpeeds - Desired chassis speeds.

---

### `public void postTrajectory(Trajectory trajectory)`
- **Description**: Posts a trajectory to be visualized on the field.
- **Parameters**:
  - `trajectory`: Trajectory - The trajectory to post.

---

### `public void zeroGyro()`
- **Description**: Resets the robot’s gyro angle to zero.

---

### `public boolean isRedAlliance()`
- **Description**: Checks if the alliance is red. Defaults to false if alliance information is unavailable.
- **Returns**: boolean - True if the alliance is red, false otherwise.

---

### `public void zeroGyroWithAlliance()`
- **Description**: Zeros the robot’s gyro and adjusts orientation if in the red alliance.

---

### `public void setMotorBrake(boolean brake)`
- **Description**: Sets drive motors to brake or coast mode.
- **Parameters**:
  - `brake`: boolean - True for brake mode, false for coast mode.

Here's the additional documentation for the specified public methods:

---

### `public Rotation2d getHeading()`
- **Description**: Retrieves the current yaw angle of the robot as reported by the swerve pose estimator. This value is corrected by any prior calls to `resetOdometry()` and may differ from the raw gyro reading.
- **Returns**: Rotation2d - The current yaw angle of the robot.

---

### `public ChassisSpeeds getFieldVelocity()`
- **Description**: Gets the current field-relative velocity of the robot, providing x, y, and omega (angular) speeds.
- **Returns**: ChassisSpeeds - The current field-relative velocity of the robot.

---

### `public ChassisSpeeds getRobotVelocity()`
- **Description**: Retrieves the current robot-relative velocity, including x, y, and omega (angular) speeds.
- **Returns**: ChassisSpeeds - The current robot-relative velocity.

---

### `public SwerveController getSwerveController()`
- **Description**: Provides the swerve controller instance used within the swerve drive.
- **Returns**: SwerveController - The swerve controller for the robot's drive system.

---

### `public SwerveDriveConfiguration getSwerveDriveConfiguration()`
- **Description**: Returns the swerve drive configuration object containing the drive's settings and parameters.
- **Returns**: SwerveDriveConfiguration - The configuration for the current swerve drive.

---

### `public void lock()`
- **Description**: Locks the swerve drive to prevent movement, fixing its position on the field.

---

### `public Rotation2d getPitch()`
- **Description**: Gets the current pitch angle of the robot as reported by the IMU, indicating the tilt angle of the robot.
- **Returns**: Rotation2d - The current pitch angle.