package entechlib.entech_swerve.exceptions;

public class InvalidTypeException extends RuntimeException {
    public InvalidTypeException(String fieldType, String given) {
        super("Invalid " + fieldType + " type. Given: " + given);
    }
}
