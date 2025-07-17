package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    // Should be Mode.REPLAY when replaying, else Mode.SIM
    public static final Mode simMode = Mode.SIM;

    // Current mode the robot program is in
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double debounceTime =
            0.5; // time in seconds to wait before reporting a motor or encoder disconnected

    // Whether LoggedTunableNumbers should be published to NT
    public static final boolean tuningMode = true;
}
