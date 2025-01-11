package entechlib.input;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class TestDriveInput {
    @Test
    void testGetRotationSpeed() {
        DriveInput example = new DriveInput(0.75, 0.8, 0.3);
        assertEquals(0.3, example.getRotationSpeed());
    }

    @Test
    void testGetXSpeed() {
        DriveInput example = new DriveInput(0.75, 0.8, 0.3);
        assertEquals(0.75, example.getXSpeed());
    }

    @Test
    void testGetYSpeed() {
        DriveInput example = new DriveInput(0.75, 0.8, 0.3);
        assertEquals(0.8, example.getYSpeed());
    }
}
