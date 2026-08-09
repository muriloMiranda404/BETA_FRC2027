package frc.frc_java9485.utils.calc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Pair;

class LinearLineCalculatorTest {
    private static final double DELTA = 1e-9;

    @Test
    void calculateEvaluatesTheLine() {
        LinearLineCalculator line = new LinearLineCalculator(2.0, 1.0);

        assertEquals(1.0, line.calculate(0.0), DELTA);
        assertEquals(7.0, line.calculate(3.0), DELTA);
    }

    @Test
    void nanInputGivesNanOutput() {
        assertTrue(Double.isNaN(new LinearLineCalculator(2.0, 1.0).calculate(Double.NaN)));
    }

    @Test
    void bestFitRecoversAnExactLine() {
        List<Pair<Double, Double>> points =
                List.of(Pair.of(1.0, 3.0), Pair.of(2.0, 5.0), Pair.of(3.0, 7.0), Pair.of(4.0, 9.0));

        LinearLineCalculator fit = LinearLineCalculator.bestFit(points);

        assertEquals(2.0, fit.getSlope(), 1e-9);
        assertEquals(1.0, fit.getIntercept(), 1e-9);
        assertEquals(1.0, fit.rSquared(points), 1e-9);
    }


    @Test
    void bestFitHandlesNoisyData() {
        List<Pair<Double, Double>> points =
                List.of(Pair.of(1.0, 3.1), Pair.of(2.0, 4.9), Pair.of(3.0, 7.2), Pair.of(4.0, 8.8));

        LinearLineCalculator fit = LinearLineCalculator.bestFit(points);

        assertEquals(2.0, fit.getSlope(), 0.2);
        assertTrue(fit.rSquared(points) > 0.98, "expected a good fit, got R2=" + fit.rSquared(points));
    }


    @Test
    void rSquaredIsLowForNonLinearData() {
        List<Pair<Double, Double>> points =
                List.of(Pair.of(0.0, 0.0), Pair.of(1.0, 1.0), Pair.of(2.0, 16.0), Pair.of(3.0, 81.0));

        LinearLineCalculator fit = LinearLineCalculator.bestFit(points);

        assertTrue(fit.rSquared(points) < 0.95, "expected a poor fit, got R2=" + fit.rSquared(points));
    }

    @Test
    void tooFewPointsIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> LinearLineCalculator.bestFit(List.of()));
        assertThrows(
                IllegalArgumentException.class,
                () -> LinearLineCalculator.bestFit(List.of(Pair.of(1.0, 1.0))));
    }


    @Test
    void verticalDataIsRejected() {
        assertThrows(
                IllegalArgumentException.class,
                () -> LinearLineCalculator.bestFit(List.of(Pair.of(2.0, 1.0), Pair.of(2.0, 5.0))));
    }
}
