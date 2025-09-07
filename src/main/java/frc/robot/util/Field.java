package frc.robot.util;

public class Field {
    // Field dimensions
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;

    // Whether the field this season has rotational (C2) or reflected (D2) symmetry
    public enum FieldSymmetry {
        C2,
        D2
    }

    public static final FieldSymmetry symm = FieldSymmetry.C2;
}
