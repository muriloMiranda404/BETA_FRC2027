package frc.frc_java9485.utils.calc;

import edu.wpi.first.math.MathUtil;


public class ClosedInterval {

    private final int start;
    private final int end;

    public ClosedInterval(int start, int end) {
        if (end < start) {
            throw new IllegalArgumentException("End (" + end + ") must not be before start (" + start + ")");
        }
        this.start = start;
        this.end = end;
    }

    public int getStart() {
        return start;
    }

    public int getEnd() {
        return end;
    }


    public int getLength() {
        return end - start;
    }


    public int size() {
        return end - start + 1;
    }


    public int getIndex(int index) {
        int value = start + index;
        if (value < start || value > end) {
            throw new IndexOutOfBoundsException(
                    "Index " + index + " is outside " + this);
        }
        return value;
    }


    public ClosedInterval getFromIndexRange(int indexStart, int indexEnd) {
        int clampedStart = MathUtil.clamp(indexStart + this.start, this.start, this.end);
        int clampedEnd = MathUtil.clamp(indexEnd + this.start, this.start, this.end);
        return new ClosedInterval(Math.min(clampedStart, clampedEnd), Math.max(clampedStart, clampedEnd));
    }

    public ClosedInterval getFromIndexRange(ClosedInterval indexInterval) {
        return getFromIndexRange(indexInterval.getStart(), indexInterval.getEnd());
    }


    public ClosedInterval getFromIndexRange(int endIndex) {
        return getFromIndexRange(0, endIndex);
    }


    public boolean collides(ClosedInterval other) {
        return Math.max(this.start, other.start) <= Math.min(this.end, other.end);
    }


    public boolean contains(int value) {
        return value >= start && value <= end;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof ClosedInterval)) {
            return false;
        }
        ClosedInterval other = (ClosedInterval) o;
        return other.start == start && other.end == end;
    }

    @Override
    public int hashCode() {
        return 31 * start + end;
    }

    @Override
    public String toString() {
        return "[" + start + ", " + end + "]";
    }
}
