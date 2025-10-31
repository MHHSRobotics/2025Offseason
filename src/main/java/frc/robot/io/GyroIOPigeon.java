package frc.robot.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class GyroIOPigeon extends GyroIO {
    private Pigeon2 gyro;
    private Pigeon2SimState sim;

    private boolean disconnected = false;

    public GyroIOPigeon(int id, CANBus canBus, String name, String logPath) {
        super(name, logPath);
        gyro = new Pigeon2(id, canBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0);

        sim = gyro.getSimState();
    }

    public GyroIOPigeon(int id, String canBus, String name, String logPath) {
        this(id, new CANBus(canBus), name, logPath);
    }

    public GyroIOPigeon(int id, String name, String logPath) {
        this(id, new CANBus(), name, logPath);
    }

    @Override
    public void update() {
        inputs.connected = disconnected ? false : gyro.isConnected();
        inputs.yawPositionRad = Degrees.of(gyro.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec =
                DegreesPerSecond.of(gyro.getAngularVelocityZWorld().getValueAsDouble());
        inputs.hardwareFault = gyro.getFault_Hardware().getValue();

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    @Override
    public void setYaw(Angle yaw) {
        gyro.setYaw(yaw);
    }

    @Override
    public void setMechYaw(Angle yaw) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechYaw on " + getName(), AlertType.kWarning);
            return;
        }
        sim.setRawYaw(yaw.in(Degrees));
    }

    @Override
    public void setMechYawVelocity(AngularVelocity yawVelocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechYawVelocity on " + getName(), AlertType.kWarning);
            return;
        }
        sim.setAngularVelocityZ(yawVelocity.in(DegreesPerSecond));
    }

    @Override
    public void setConnected(boolean connected) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setConnected on " + getName(), AlertType.kWarning);
            return;
        }
        disconnected = !connected;
    }
}
