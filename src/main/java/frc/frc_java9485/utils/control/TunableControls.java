package frc.frc_java9485.utils.control;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;


public class TunableControls {

    public static class ControlConstants {

        double kP = 0;
        double kI = 0;
        double kD = 0;
        double tolerance = 0;
        double velTolerance = Double.POSITIVE_INFINITY;
        double iZone = Double.POSITIVE_INFINITY;
        double iMin = Double.NEGATIVE_INFINITY;
        double iMax = Double.POSITIVE_INFINITY;
        double period = 0.02;


        double kV, kA = 0;
        boolean isControllingVelocity = false;


        double kS, kG = 0;


        boolean profiled = false;
        double maxVel = 0;
        double maxAcc = 0;


        boolean isContinuous = false;
        double maxInput;
        double minInput;

        public ControlConstants() {}

        public ControlConstants(ControlConstants constants) {
            this.kP = constants.kP;
            this.kI = constants.kI;
            this.kD = constants.kD;
            this.tolerance = constants.tolerance;
            this.velTolerance = constants.velTolerance;
            this.iZone = constants.iZone;
            this.iMax = constants.iMax;
            this.iMin = constants.iMin;
            this.period = constants.period;
            this.kV = constants.kV;
            this.kA = constants.kA;
            this.isControllingVelocity = constants.isControllingVelocity;
            this.kS = constants.kS;
            this.kG = constants.kG;
            this.maxVel = constants.maxVel;
            this.maxAcc = constants.maxAcc;
            this.isContinuous = constants.isContinuous;
            this.maxInput = constants.maxInput;
            this.minInput = constants.minInput;
        }


        public ControlConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }


        public ControlConstants withFeedforward(double kV, double kA) {
            this.kV = kV;
            this.kA = kA;
            return this;
        }


        public ControlConstants withVelocityControl() {
            this.isControllingVelocity = true;
            return this;
        }


        public ControlConstants withPhysical(double kS, double kG) {
            this.kS = kS;
            this.kG = kG;
            return this;
        }


        public ControlConstants withProfile(double maxVel, double maxAcc) {
            this.profiled = true;
            this.maxVel = maxVel;
            this.maxAcc = maxAcc;
            return this;
        }


        public ControlConstants withProfiled(boolean profiled) {
            this.profiled = profiled;
            return this;
        }


        public ControlConstants withTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }


        public ControlConstants withTolerance(double tolerance, double velTolerance) {
            this.tolerance = tolerance;
            this.velTolerance = velTolerance;

            return this;
        }


        public ControlConstants withIZone(double iZone) {
            this.iZone = iZone;
            return this;
        }


        public ControlConstants withIRange(double iMin, double iMax) {
            this.iMin = iMin;
            this.iMax = iMax;
            return this;
        }


        public ControlConstants withPeriod(double period) {
            this.period = period;
            return this;
        }


        public ControlConstants withContinuous(double minInput, double maxInput) {
            this.isContinuous = true;
            this.minInput = minInput;
            this.maxInput = maxInput;
            return this;
        }


        public PIDController getPIDController() {
            PIDController controller = new PIDController(kP, kI, kD);
            controller.setTolerance(tolerance);
            controller.setIntegratorRange(iMin, iMax);
            controller.setIZone(iZone);

            return controller;
        }


        public ProfiledPIDController getProfiledPIDController() {
            ProfiledPIDController controller =
                    new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAcc));
            controller.setTolerance(tolerance);
            controller.setIntegratorRange(iMin, iMax);
            controller.setIZone(iZone);

            return controller;
        }


        public ElevatorFeedforward getElevatorFeedforward() {
            return new ElevatorFeedforward(kS, kG, kV, kA);
        }


        public SimpleMotorFeedforward getSimpleFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }
    }


    public static class TunableControlConstants {

        LoggedTunableNumber kP;
        LoggedTunableNumber kI;
        LoggedTunableNumber kD;
        LoggedTunableNumber tolerance;
        LoggedTunableNumber velTolerance;
        LoggedTunableNumber iZone;
        LoggedTunableNumber iMin;
        LoggedTunableNumber iMax;
        double period;


        LoggedTunableNumber kV;
        LoggedTunableNumber kA;
        boolean isControllingVelocity;


        LoggedTunableNumber kS;
        LoggedTunableNumber kG;


        boolean profiled;
        LoggedTunableNumber maxVel;
        LoggedTunableNumber maxAcc;


        boolean isContinuous;
        double maxInput;
        double minInput;


        public TunableControlConstants(String key, ControlConstants constants) {
            this.kP = new LoggedTunableNumber(key + "/kP", constants.kP);
            this.kI = new LoggedTunableNumber(key + "/kI", constants.kI);
            this.kD = new LoggedTunableNumber(key + "/kD", constants.kD);
            this.tolerance = new LoggedTunableNumber(key + "/tolerance", constants.tolerance);
            this.velTolerance = new LoggedTunableNumber(key + "/velTolerance", constants.velTolerance);
            this.iZone = new LoggedTunableNumber(key + "/iZone", constants.iZone);
            this.iMax = new LoggedTunableNumber(key + "/maxIntegral", constants.iMax);
            this.iMin = new LoggedTunableNumber(key + "/minIntegral", constants.iMin);
            this.period = constants.period;
            this.kV = new LoggedTunableNumber(key + "/kV", constants.kV);
            this.kA = new LoggedTunableNumber(key + "/kA", constants.kA);
            this.isControllingVelocity = constants.isControllingVelocity;
            this.kS = new LoggedTunableNumber(key + "/kS", constants.kS);
            this.kG = new LoggedTunableNumber(key + "/kG", constants.kG);
            this.profiled = constants.profiled;
            this.maxVel = new LoggedTunableNumber(key + "/maxVel", constants.maxVel);
            this.maxAcc = new LoggedTunableNumber(key + "/maxAcc", constants.maxAcc);
            this.isContinuous = constants.isContinuous;
            this.maxInput = constants.maxInput;
            this.minInput = constants.minInput;
        }


        public LoggedTunableNumber[] getAllTunableNumbers() {
            return new LoggedTunableNumber[] {
                kP, kI, kD, tolerance, velTolerance, iZone, iMin, iMax, kV, kA, kS, kG, maxVel, maxAcc
            };
        }


        public PIDController getPIDController() {
            PIDController controller = new PIDController(kP.get(), kI.get(), kD.get());
            controller.setTolerance(tolerance.get());
            controller.setIntegratorRange(iMin.get(), iMax.get());
            controller.setIZone(iZone.get());

            return controller;
        }


        public ProfiledPIDController getProfiledPIDController() {
            ProfiledPIDController controller = new ProfiledPIDController(
                    kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(maxVel.get(), maxAcc.get()));
            controller.setTolerance(tolerance.get());
            controller.setIntegratorRange(iMin.get(), iMax.get());
            controller.setIZone(iZone.get());

            return controller;
        }


        public ElevatorFeedforward getElevatorFeedforward() {
            return new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        }


        public SimpleMotorFeedforward getSimpleFeedforward() {
            return new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
        }
    }


    public static class TunablePIDController {
        private final TunableControlConstants params;
        private final PIDController pidController;


        public TunablePIDController(TunableControlConstants tunableParams) {
            this.params = tunableParams;

            pidController = new PIDController(
                    tunableParams.kP.get(), tunableParams.kI.get(), tunableParams.kD.get(), tunableParams.period);

            pidController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

            if (tunableParams.isContinuous) {
                pidController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
            }
        }


        public TunableControlConstants getParams() {
            return params;
        }


        public void updateParams() {
            pidController.setP(params.kP.get());
            pidController.setI(params.kI.get());
            pidController.setD(params.kD.get());
            pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
            pidController.setIZone(params.iZone.get());
            pidController.setIntegratorRange(params.iMin.get(), params.iMax.get());
            pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
        }


        public double getAccumulatedError() {
            return pidController.getAccumulatedError();
        }


        public void setSetpoint(double goal) {
            pidController.setSetpoint(goal);
            updateParams();
        }


        public double getSetpoint() {
            return pidController.getSetpoint();
        }


        public boolean atSetpoint() {
            return pidController.atSetpoint();
        }


        public double getPositionError() {
            return pidController.getError();
        }


        public double getVelocityError() {
            return pidController.getErrorDerivative();
        }


        public double calculate(double measurement, double goal) {
            return pidController.calculate(measurement, goal);
        }


        public double calculate(double measurement) {
            return pidController.calculate(measurement);
        }


        public void reset() {
            pidController.reset();
        }
    }


    public static class TunableProfiledController {
        private final ProfiledPIDController profiledPIDController;
        private final TunableControlConstants params;

        private double previousVelocity = 0;


        public TunableProfiledController(TunableControlConstants tunableParams) {
            this.params = tunableParams;

            if (!tunableParams.profiled) {
                throw new IllegalArgumentException(
                        "TunableControlConstants must be profiled to use TunableProfiledController");
            }

            profiledPIDController = new ProfiledPIDController(
                    tunableParams.kP.get(),
                    tunableParams.kI.get(),
                    tunableParams.kD.get(),
                    new TrapezoidProfile.Constraints(tunableParams.maxVel.get(), tunableParams.maxAcc.get()),
                    tunableParams.period);

            profiledPIDController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

            if (tunableParams.isContinuous) {
                profiledPIDController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
            }
        }


        public TunableControlConstants getParams() {
            return params;
        }


        public void updateParams() {
            profiledPIDController.setP(params.kP.get());
            profiledPIDController.setI(params.kI.get());
            profiledPIDController.setD(params.kD.get());

            profiledPIDController.setConstraints(
                    new TrapezoidProfile.Constraints(params.maxVel.get(), params.maxAcc.get()));
            profiledPIDController.setIZone(params.iZone.get());
            profiledPIDController.setIntegratorRange(params.iMin.get(), params.iMax.get());
            profiledPIDController.setTolerance(params.tolerance.get(), params.velTolerance.get());
        }


        public ProfiledPIDController getProfiledPIDController() {
            return profiledPIDController;
        }


        public double getAccumulatedError() {
            return profiledPIDController.getAccumulatedError();
        }


        public void setGoal(double goal, double goalVel) {
            profiledPIDController.setGoal(new TrapezoidProfile.State(goal, goalVel));
            previousVelocity = profiledPIDController.getSetpoint().velocity;
            updateParams();
        }


        public void setGoal(double goal) {
            setGoal(goal, 0);
        }


        public double getGoal() {
            return profiledPIDController.getGoal().position;
        }


        public State getSetpoint() {
            return profiledPIDController.getSetpoint();
        }


        public boolean atGoal() {
            return profiledPIDController.atGoal();
        }


        public double getPositionError() {
            return profiledPIDController.getPositionError();
        }


        public double getVelocityError() {
            return profiledPIDController.getVelocityError();
        }


        public double calculateFeedforward() {
            State setpoint = profiledPIDController.getSetpoint();
            double velocity = params.isControllingVelocity ? setpoint.position : setpoint.velocity;
            double accel = (velocity - previousVelocity) / profiledPIDController.getPeriod();
            previousVelocity = velocity;

            return params.kS.get() * Math.signum(velocity)
                    + params.kG.get()
                    + params.kV.get() * velocity
                    + params.kA.get() * accel;
        }


        public double calculate(double measurement, double goal) {
            return profiledPIDController.calculate(measurement, goal) + calculateFeedforward();
        }


        public double calculate(double measurement) {
            return profiledPIDController.calculate(measurement) + calculateFeedforward();
        }


        public void reset(double measuredPos, double measuredVel) {
            profiledPIDController.reset(measuredPos, measuredVel);
            previousVelocity = measuredVel;
        }


        public void reset(double measuredPos) {
            reset(measuredPos, 0);
        }


        public void logData(String tableKey) {
            Logger.recordOutput(tableKey + "/Goal", getGoal());
            Logger.recordOutput(tableKey + "/Setpoint", getSetpoint());
            Logger.recordOutput(tableKey + "/Position error", getPositionError());
            Logger.recordOutput(tableKey + "/Accumulated error", getAccumulatedError());
            Logger.recordOutput(tableKey + "/Velocity error", getVelocityError());
        }
    }
}
