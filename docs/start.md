# Welcome to the Rapid Acceleration 2025 Docs!
These docs aim to describe all components of our codebase, as well as provide links to the documentation of the libraries and frameworks that this codebase uses. It was created in part using ChatGPT, so if you discover that any part of the documentation is incorrect, fix it if you can or open an issue.

## Library Documentation Links
Below are links to the documentation of the various libraries that this codebase uses.
* **WPILib**  ------- https://docs.wpilib.org/en/stable/index.html#
* **YAGSL** -------- https://yagsl.gitbook.io/yagsl

This list should be updated as new components are used.

## Codebase Overview  

### 1. Main.java  

**Purpose**: The `Main` class serves as the entry point of the application and initiates the robot’s lifecycle.

**Key Functionality**:
- The `main` method calls `RobotBase.startRobot(Robot::new)`, which initializes the robot by creating a new `Robot` instance.
- No variables or additional initialization code should be added to this class; it exists solely to initiate the robot’s operation through the `startRobot` call.

---

### 2. Constants.java

**Purpose**: The `Constants` class provides a centralized location for all robot-wide constants such as mass, drive settings, and PID control parameters.

**Key Fields**:
-  `ROBOT_MASS`: Mass of the robot in kilograms (calculated from pounds).

-  `CHASSIS`: Defines the robot’s chassis properties using `Matter`, including the center of mass and chassis weight.

-  `LOOP_TIME`: Sets the control loop interval to match motor controller latency.

-  `MAX_SPEED`: The maximum speed of the robot in meters per second, useful for setting acceleration limits.

**Inner Classes**:
-  `AutonConstants`: Contains PID constants for autonomous mode, specifying proportional, integral, and derivative values for translation and angle adjustments. To learn more about PID go [here][PIDLink].

-  `DrivebaseConstants`: Holds settings for drive behavior, such as motor brake hold time.

-  `OperatorConstants`: Sets joystick deadband values and turn scaling constants to filter out unintended small movements.

---

### 3. Robot.java

**Purpose**: The `Robot` class defines the robot's behavior across different operational modes. It extends `TimedRobot`, which provides timed execution methods for `init` and `periodic` events.

**Key Functionality**:

-  `robotInit`: Initializes the robot by creating a `RobotContainer` instance, which handles command and subsystem management. Also sets up a `Timer` for motor brake control.

-  `robotPeriodic`: Runs commands at a regular interval, including button polling, command scheduling, and subsystem updates.
-  `disabledInit` and `disabledPeriodic`: Manage the motor brake behavior during the disabled mode, with a timed release for smoother handling.

-  `autonomousInit` and `autonomousPeriodic`: Engage the autonomous mode by scheduling the selected command from `RobotContainer`.

-  `teleopInit` and `teleopPeriodic`: Handle teleoperation mode, ensuring any autonomous commands are canceled and that the motor brake is engaged.

-  `testInit` and `testPeriodic`: Prepare the robot for test mode by canceling all running commands and loading swerve drive configurations.

-  `simulationInit` and `simulationPeriodic`: Methods reserved for simulation-related initialization and periodic tasks.

---

### 4. RobotContainer.java

**Purpose**: The `RobotContainer` class centralizes subsystem instances, controller bindings, and autonomous command selection.

**Key Components**:
-  **Subsystems**:
	-  `SwerveSubsystem`: Manages swerve-drive commands and configurations.
	-  `ExampleSubsystem`: A placeholder or example subsystem for additional functionality.

-  **Controllers**: Uses a `CommandXboxController` for managing driver inputs.

**Main Functions**:
-  **Constructor**:
	- Configures input bindings and assigns default commands to subsystems.
	- Sets up drive commands with deadband filtering, allowing joystick inputs to control robot movement smoothly.

-  `configureBindings`: Binds controller inputs to specific actions, such as zeroing the gyro and triggering commands.

-  `getAutonomousCommand`: Returns the autonomous command selected for the robot, currently an `ExampleAuton` instance.

-  `setMotorBrake`: Activates or deactivates the motor brake on the swerve drive system based on input.

[PIDLink]: https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_(Woolf)/09%3A_Proportional-Integral-Derivative_(PID)_Control/9.02%3A_P_I_D_PI_PD_and_PID_control