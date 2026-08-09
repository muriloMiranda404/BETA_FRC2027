package frc.frc_java9485.utils.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;


public class SlewRateLimiter2d {

    private double rateLimit;

    private Translation2d prevTranslation;
    private Time prevTime;


    public SlewRateLimiter2d(double rateLimit) {
        this.rateLimit = rateLimit;
        this.prevTranslation = Translation2d.kZero;
        this.prevTime = RobotController.getMeasureTime();
    }


    public Translation2d calculate(Translation2d translation) {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(prevTime).in(Seconds);
        prevTime = currentTime;

        prevTranslation = MathUtil.slewRateLimit(prevTranslation, translation, deltaTime, rateLimit);
        return prevTranslation;
    }

    public Translation2d calculate(double x, double y) {
        return calculate(new Translation2d(x, y));
    }


    public void reset(Translation2d translation) {
        prevTranslation = translation;
        prevTime = RobotController.getMeasureTime();
    }

    public void reset() {
        reset(Translation2d.kZero);
    }

    public Translation2d getLastValue() {
        return prevTranslation;
    }

    public void setRateLimit(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    public double getRateLimit() {
        return rateLimit;
    }
}
