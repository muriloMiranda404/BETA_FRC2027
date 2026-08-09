package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.frc_java9485.constants.utils.LoggerConstants;
import frc.frc_java9485.utils.logger.LoggedTracer;


public class RobotAlerts {
    private static final double LOW_BATTERY_VOLTS = 11.5;
    private static final double HIGH_CAN_UTILIZATION = 0.9;

    private final PowerDistribution powerDistribution;

    private final Alert lowBattery = new Alert("Bateria baixa (< 11.5 V)", AlertType.kWarning);
    private final Alert highCanUtilization =
        new Alert("Uso do barramento CAN alto (> 90%)", AlertType.kWarning);
    private final Alert canReceiveErrors = new Alert("Erros de recepcao no barramento CAN", AlertType.kWarning);
    private final Alert brownedOut = new Alert("Brownout detectado!", AlertType.kError);
    private final Alert loopOverrun = new Alert("Loop acima do orcamento", AlertType.kWarning);

    public RobotAlerts(PowerDistribution powerDistribution) {
        this.powerDistribution = powerDistribution;
    }


    public void update() {
        lowBattery.set(powerDistribution.getVoltage() < LOW_BATTERY_VOLTS);

        var can = RobotController.getCANStatus();
        highCanUtilization.set(can.percentBusUtilization > HIGH_CAN_UTILIZATION);
        canReceiveErrors.set(can.receiveErrorCount > 0);

        brownedOut.set(RobotController.isBrownedOut());

        updateLoopOverrun();
    }


    private void updateLoopOverrun() {
        boolean recentlyOverrun =
            LoggedTracer.hasRecentOverrun(LoggerConstants.LOOP_OVERRUN_ALERT_WINDOW_SEC);

        if (recentlyOverrun) {
            loopOverrun.setText(String.format(
                "Loop em %.1f ms (orcamento %.0f ms) - fase mais lenta: %s (%.1f ms), %d estouros",
                LoggedTracer.getLastLoopMs(),
                LoggerConstants.LOOP_OVERRUN_THRESHOLD_MS,
                LoggedTracer.getWorstEpochName(),
                LoggedTracer.getWorstEpochMs(),
                LoggedTracer.getOverrunCount()));
        }
        loopOverrun.set(recentlyOverrun);
    }
}
