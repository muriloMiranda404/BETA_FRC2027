package frc.frc_java9485.utils.calc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MathUtilsTest {
    private static final double DELTA = 1e-9;

    @Test
    void inRange_valueWithinBounds_isTrue() {

        assertTrue(MathUtils.inRange(5.0, 10.0, 0.0));
    }

    @Test
    void inRange_valueOutsideBounds_isFalse() {
        assertFalse(MathUtils.inRange(15.0, 10.0, 0.0));
        assertFalse(MathUtils.inRange(-5.0, 10.0, 0.0));
    }

    @Test
    void scope0To360_wrapsValuesAbove360() {
        assertEquals(10.0, MathUtils.scope0To360(370.0), DELTA);
    }

    @Test
    void scope0To360_leavesValuesInRangeUnchanged() {
        assertEquals(45.0, MathUtils.scope0To360(45.0), DELTA);
    }
}
