package frc.frc_java9485.utils.calc;

import edu.wpi.first.wpilibj.Timer;

public class MathUtils {
  public static Timer timer = new Timer();

  public static boolean inRange(double value, double max, double min) {
    return value < max && value > min;
  }

  public static double scope0To360(double value) {
    value %= 360;

    return value;
  }

   public static boolean isStableInRange(double value, double stabilityTime, double setpoint, double range) {
    if (Util.inRange(value, setpoint - range, setpoint + range)) {
      if (MathUtils.timer.get() > stabilityTime) {
        return true;
      } else {
        return false;
      }
    } else {
      MathUtils.timer.reset();
      MathUtils.timer.start();
      return false;
    }
  }
}
