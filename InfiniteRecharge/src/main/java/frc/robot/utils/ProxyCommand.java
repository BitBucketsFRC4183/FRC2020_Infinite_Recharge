package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The ProxyCommand helper allows us to delay instantiation of commands until the command is initialized
 * This way auto command paths (like secondary trajectories) that are never run will not ever be instantiated
 * because they are never scheduled
 */
public class ProxyCommand extends CommandBase {
    private final Supplier<Command> commandSupplier;
    private Command command;

    public ProxyCommand(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    /**
     * On initialize, create the proxied command
     */
    @Override
    public void initialize() {
        command = commandSupplier.get();
        command.schedule();
    }

    /**
     * Call the proxy
     */
    @Override
    public void execute() {
        command.execute();
    }

    /**
     * Call the proxy
     */
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        CommandScheduler.getInstance().cancel(command);
      }
      command.end(interrupted);
    }

    /**
     * Call the proxy
     */
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
