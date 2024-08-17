package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommand extends Command {

    final ExampleSubsystem exampleSubsystem;

    /**
     * Creates a new ExampleCommand.
     * 
     * @param exampleSubsystem The subsystem used by this command.
     */
    public ExampleCommand(ExampleSubsystem exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
        addRequirements(exampleSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the schedular runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
