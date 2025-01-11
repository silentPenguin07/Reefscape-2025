package entechlib.swerve.exceptions;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import entechlib.entech_swerve.exceptions.InvalidTypeException;

class TestInvalidTypeException {
    @Test
    void testGetMessageNull() {
        Exception ex = new InvalidTypeException("Item", null);
        assertEquals("Invalid Item type. Given: null", ex.getMessage());
    }

    @Test
    void testGetMessageText() {
        Exception ex = new InvalidTypeException("Item", "Wrong");
        assertEquals("Invalid Item type. Given: Wrong", ex.getMessage());
    }
}
