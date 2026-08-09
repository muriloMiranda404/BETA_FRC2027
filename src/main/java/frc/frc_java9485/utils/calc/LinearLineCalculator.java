package frc.frc_java9485.utils.calc;

import java.util.List;

import edu.wpi.first.math.Pair;


public class LinearLineCalculator {

    private final double slope;
    private final double intercept;

    public LinearLineCalculator(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }

    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }

    public double calculate(double input) {
        if (Double.isNaN(input)) {
            return Double.NaN;
        }
        return slope * input + intercept;
    }


    public static LinearLineCalculator bestFit(List<Pair<Double, Double>> data) {
        int n = data.size();
        if (n < 2) {
            throw new IllegalArgumentException("A best fit needs at least 2 points, got " + n);
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;

        for (Pair<Double, Double> point : data) {
            double x = point.getFirst();
            double y = point.getSecond();

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        double denominator = n * sumX2 - sumX * sumX;
        if (denominator == 0.0) {
            throw new IllegalArgumentException("Cannot fit a line: every sample has the same x value");
        }

        double slope = (n * sumXY - sumX * sumY) / denominator;
        double intercept = (sumY / n) - slope * (sumX / n);

        return new LinearLineCalculator(slope, intercept);
    }


    public double rSquared(List<Pair<Double, Double>> data) {
        if (data.isEmpty()) {
            return Double.NaN;
        }

        double meanY = data.stream().mapToDouble(Pair::getSecond).average().orElse(0.0);

        double residualSum = 0.0;
        double totalSum = 0.0;
        for (Pair<Double, Double> point : data) {
            double predicted = calculate(point.getFirst());
            residualSum += Math.pow(point.getSecond() - predicted, 2);
            totalSum += Math.pow(point.getSecond() - meanY, 2);
        }


        return totalSum == 0.0 ? 1.0 : 1.0 - (residualSum / totalSum);
    }

    @Override
    public String toString() {
        return String.format("y = %.6f x + %.6f", slope, intercept);
    }
}
