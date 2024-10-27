# RobotContainer

Class package: `frc.robot`

## Purpose
The `RobotContainer` class serves as a container for subsystems, commands, and control bindings for the robot. It organizes the initialization and default behaviors for the robot's systems and defines input mappings, particularly with the Xbox controller, for operator control.

## Public Fields:

- `drivebase`: `SwerveSubsystem` instance responsible for handling all swerve drive functionalities of the robot. The swerve subsystem is initialized with configuration files stored in the deploy directory

## Public Methods

### `RobotContainer()`
Constructs the `RobotContainer` object, initializing subsystems, configuring control bindings, and setting default commands.
  
### `void setMotorBrake(boolean brake)`
Configures the drive motors to enable or disable brake mode.

**Parameters**:
- `brake`: (boolean) If `true`, sets the motors to brake mode; if `false`, sets them to coast mode.

### `Command getAutonomousCommand()`
Retrieves the command sequence to run during the autonomous phase of the game.

**Returns**: a `Command` object representing the autonomous command sequence.