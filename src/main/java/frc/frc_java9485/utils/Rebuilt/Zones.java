package frc.frc_java9485.utils.Rebuilt;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.frc_java9485.constants.utils.FieldElementsConst.FieldMeansureds;


public class Zones {

    private Zones() {}

    public interface Zone {
        Trigger contains(Supplier<Pose2d> pose);
    }

    public interface PredictiveXZone extends Zone {
        Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
    }


    public static class BaseZone implements Zone {
        protected final double xMin;
        protected final double xMax;
        protected final double yMin;
        protected final double yMax;

        public BaseZone(double xMin, double xMax, double yMin, double yMax) {
            this.xMin = Math.min(xMin, xMax);
            this.xMax = Math.max(xMin, xMax);
            this.yMin = Math.min(yMin, yMax);
            this.yMax = Math.max(yMin, yMax);
        }

        public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
        }

        @Override
        public Trigger contains(Supplier<Pose2d> poseSupplier) {
            return new Trigger(() -> containsPoint(poseSupplier.get().getTranslation()));
        }

        public boolean containsPoint(Translation2d point) {
            return point.getX() >= xMin && point.getX() <= xMax
                    && point.getY() >= yMin && point.getY() <= yMax;
        }


        public BaseZone mirroredX() {
            double length = FieldMeansureds.FIELD_LENGTH_METERS;
            return new BaseZone(length - xMax, length - xMin, yMin, yMax);
        }


        public BaseZone mirroredY() {
            double width = FieldMeansureds.FIELD_WIDTH_METERS;
            return new BaseZone(xMin, xMax, width - yMax, width - yMin);
        }


        public Translation2d[] getCorners() {
            return new Translation2d[] {
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax),
                new Translation2d(xMin, yMin)
            };
        }
    }


    public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {

        public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveXBaseZone(BaseZone zone) {
            super(zone.xMin, zone.xMax, zone.yMin, zone.yMax);
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            return new Trigger(() -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
        }

        public boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
            if (point.getY() < yMin || point.getY() > yMax) {
                return false;
            }
            if (point.getX() >= xMin && point.getX() <= xMax) {
                return true;
            }

            double travel = fieldSpeeds.vxMetersPerSecond * dt.in(Seconds);

            return (point.getX() < xMin && travel >= xMin - point.getX())
                    || (point.getX() > xMax && travel <= xMax - point.getX());
        }

        @Override
        public PredictiveXBaseZone mirroredX() {
            return new PredictiveXBaseZone(super.mirroredX());
        }

        @Override
        public PredictiveXBaseZone mirroredY() {
            return new PredictiveXBaseZone(super.mirroredY());
        }
    }


    public static class ZoneCollection implements Zone {
        protected final Zone[] zones;

        public ZoneCollection(Zone... zones) {
            this.zones = zones;
        }

        @Override
        public Trigger contains(Supplier<Pose2d> pose) {
            Trigger combined = new Trigger(() -> false);
            for (Zone zone : zones) {
                combined = combined.or(zone.contains(pose));
            }
            return combined;
        }
    }

    public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {

        public PredictiveXZoneCollection(PredictiveXZone... zones) {
            super(zones);
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            Trigger combined = new Trigger(() -> false);
            for (Zone zone : zones) {
                combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
            }
            return combined;
        }
    }



    private static final double FIELD_LENGTH = FieldMeansureds.FIELD_LENGTH_METERS;
    private static final double FIELD_WIDTH = FieldMeansureds.FIELD_WIDTH_METERS;
    private static final double ALLIANCE_ZONE_DEPTH = FieldMeansureds.ALLIANCE_ZONE.in(Meters);


    public static final PredictiveXBaseZone BLUE_ALLIANCE_ZONE =
            new PredictiveXBaseZone(0.0, ALLIANCE_ZONE_DEPTH, 0.0, FIELD_WIDTH);

    public static final PredictiveXBaseZone RED_ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE.mirroredX();

    public static final PredictiveXZoneCollection ALLIANCE_ZONES =
            new PredictiveXZoneCollection(BLUE_ALLIANCE_ZONE, RED_ALLIANCE_ZONE);


    public static final PredictiveXBaseZone NEUTRAL_ZONE = new PredictiveXBaseZone(
            ALLIANCE_ZONE_DEPTH, FIELD_LENGTH - ALLIANCE_ZONE_DEPTH, 0.0, FIELD_WIDTH);


    public static void logAllZones() {
        Logger.recordOutput("Zones/AllianceZone/Blue", BLUE_ALLIANCE_ZONE.getCorners());
        Logger.recordOutput("Zones/AllianceZone/Red", RED_ALLIANCE_ZONE.getCorners());
        Logger.recordOutput("Zones/NeutralZone", NEUTRAL_ZONE.getCorners());
    }
}
