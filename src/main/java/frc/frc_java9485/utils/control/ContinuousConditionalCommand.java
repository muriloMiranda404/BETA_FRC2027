// Adapted from FRC 6328 (Mechanical Advantage) — org.littletonrobotics.frc2026.util.ContinuousConditionalCommand.
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file
// at the root directory of the original project.

package frc.frc_java9485.utils.control;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class ContinuousConditionalCommand extends Command {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    private Command selectedCommand;


    public ContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = requireNonNullParam(onTrue, "onTrue", "ContinuousConditionalCommand");
        this.onFalse = requireNonNullParam(onFalse, "onFalse", "ContinuousConditionalCommand");
        this.condition = requireNonNullParam(condition, "condition", "ContinuousConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);

        addRequirements(onTrue.getRequirements());
        addRequirements(onFalse.getRequirements());
    }

    @Override
    public void initialize() {
        selectedCommand = condition.getAsBoolean() ? onTrue : onFalse;
        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        Command shouldBeRunning = condition.getAsBoolean() ? onTrue : onFalse;

        if (selectedCommand != shouldBeRunning) {

            selectedCommand.end(true);

            selectedCommand = shouldBeRunning;
            selectedCommand.initialize();
        }

        selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }


    @Override
    public boolean runsWhenDisabled() {
        return onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (onTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
                || onFalse.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            return InterruptionBehavior.kCancelSelf;
        }
        return InterruptionBehavior.kCancelIncoming;
    }


    public Command getSelectedCommand() {
        return selectedCommand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("onTrue", onTrue::getName, null);
        builder.addStringProperty("onFalse", onFalse::getName, null);
        builder.addStringProperty(
                "selected", () -> selectedCommand == null ? "null" : selectedCommand.getName(), null);
    }
}
