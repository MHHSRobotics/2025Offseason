package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface TalonFXIO {
    @AutoLog
    public static class TalonFXIOInputs{
        public boolean connected=false;

    }

    public default void updateInputs(){};

    public default void set(double value){};

    public default void setPosition(double pos){};
}
