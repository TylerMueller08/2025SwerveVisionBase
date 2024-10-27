# Constants

Class package: `frc.robot`

## Purpose
The `Constants` class provides a centralized location for defining robot-wide numerical and boolean constants. These constants are primarily used to avoid hardcoding values across the codebase and to promote ease of configuration. This class is intended only for the declaration of constants and should not contain any functional code. Constants are declared as `public static` fields, making them accessible across the project.

## Public Fields:

- `ROBOT_MASS`: `double` - The mass of the robot, calculated as 100 pounds converted to kilograms.
- `CHASSIS`: `Matter` - A `Matter` object representing the robot chassis with its center of mass and robot mass.
- `LOOP_TIME`: `double` - Loop time in seconds, set to account for control and response lag.
- `MAX_SPEED`: `double` - Maximum robot speed in meters per second, used to limit acceleration.

## Subclasses

### `AutonConstants`

Contains PID constants for autonomous mode, specifying proportional, integral, and derivative values for translation and angle adjustments. To learn more about PID go [here](https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_(Woolf)/09%3A_Proportional-Integral-Derivative_(PID)_Control/9.02%3A_P_I_D_PI_PD_and_PID_control).

- `TRANSLATION_PID`: `PIDConstants` - a `PIDConstants` object representing the Proportional, Integral, and Derivative values for the translation PID controller.

- `ANGLE_PID`: `PIDConstants` - a `PIDConstants` object representing the Proportional, Integral, and Derivative values for the angle PID controller.

### `DrivebaseConstants`

Holds settings for drive behavior.

- `WHEEL_LOCK_TIME`: The hold time (seconds) on motor brakes when they are disabled.

### `OperatorConstants`

Sets joystick deadband values and turn scaling constants to filter out unintended small movements.

- `LEFT_X_DEADBAND`: The left horizontal threshold for the left joystick of the robot controller, beneath which the input does not affect the robot's operation. This is so that the robot does not respond to tiny values of the joystick.

- `LEFT_Y_DEADBAND`: The same as above, but a vertical threshold instead of a horizontal one.

- `RIGHT_X_DEADBAND`: The same as above, but a horizontal threshold for the right joystick.

- `TURN_CONSTANT`: IDK, man. Probably the multiplier that takes the input from the joystick and turns it into how much the robot turns.
