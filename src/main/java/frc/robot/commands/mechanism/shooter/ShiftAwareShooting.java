package frc.robot.commands.mechanism.shooter;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.utils.Rebuilt.HubTracker;
import frc.frc_java9485.utils.Rebuilt.HubTracker.Shift;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.subsystems.mechanism.SuperStructure;

import static edu.wpi.first.units.Units.Seconds;


public class ShiftAwareShooting extends Command {

    private static final String LOG_KEY = "ShiftAware/";


    private static final LoggedTunableNumber prespinLeadSeconds =
            new LoggedTunableNumber(LOG_KEY + "PrespinLeadSeconds", 2.0);

    private final SuperStructure superStructure;
    private final boolean collectWhenClosed;


    public enum Mode {
        SHOOTING,
        PRESPINNING,
        COLLECTING,
        IDLE
    }

    private Mode mode = Mode.IDLE;


    public ShiftAwareShooting(SuperStructure superStructure, boolean collectWhenClosed) {
        this.superStructure = superStructure;
        this.collectWhenClosed = collectWhenClosed;
        addRequirements(superStructure);
    }


    public static ShiftAwareShooting collectingWhenClosed(SuperStructure superStructure) {
        return new ShiftAwareShooting(superStructure, true);
    }

    @Override
    public void execute() {
        mode = decideMode();

        superStructure.setWantedState(switch (mode) {
            case SHOOTING -> SuperStructure.WantedState.SHOOTING;
            case PRESPINNING -> SuperStructure.WantedState.PREPARING;
            case COLLECTING -> SuperStructure.WantedState.COLLECTING;
            case IDLE -> SuperStructure.WantedState.OFF;
        });

        logState();
    }


    private Mode decideMode() {
        if (HubTracker.isActive()) {
            return Mode.SHOOTING;
        }

        if (HubTracker.isActiveNext() && withinPrespinLead()) {
            return Mode.PRESPINNING;
        }

        return collectWhenClosed ? Mode.COLLECTING : Mode.IDLE;
    }


    private boolean withinPrespinLead() {
        Optional<Time> remaining = HubTracker.timeRemainingInCurrentShift();
        return remaining.isPresent() && remaining.get().in(Seconds) <= prespinLeadSeconds.get();
    }

    private void logState() {
        Optional<Shift> current = HubTracker.getCurrentShift();
        Optional<Shift> next = HubTracker.getNextShift();

        Logger.recordOutput(LOG_KEY + "Mode", mode.toString());
        Logger.recordOutput(LOG_KEY + "HubActive", HubTracker.isActive());
        Logger.recordOutput(LOG_KEY + "HubActiveNext", HubTracker.isActiveNext());
        Logger.recordOutput(LOG_KEY + "CurrentShift", current.map(Enum::toString).orElse("NONE"));
        Logger.recordOutput(LOG_KEY + "NextShift", next.map(Enum::toString).orElse("NONE"));
        Logger.recordOutput(LOG_KEY + "SecondsLeftInShift",
                HubTracker.timeRemainingInCurrentShift().map(t -> t.in(Seconds)).orElse(-1.0));
    }

    public Mode getMode() {
        return mode;
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setWantedState(SuperStructure.WantedState.OFF);
        mode = Mode.IDLE;
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
