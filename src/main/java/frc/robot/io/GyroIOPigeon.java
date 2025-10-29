package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

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
        inputs.yawPositionRad = Units.degreesToRadians(gyro.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());
        inputs.hardwareFault = gyro.getFault_Hardware().getValue();

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    @Override
    public void setYaw(double yaw) {
        gyro.setYaw(yaw);
    }

    @Override
    public void setMechYaw(double yaw) {
        sim.setRawYaw(Units.radiansToDegrees(yaw));
    }

    @Override
    public void setMechYawVelocity(double yawVelocity) {
        sim.setAngularVelocityZ(Units.radiansToDegrees(yawVelocity));
    }

    @Override
    public void disconnect() {
        disconnected = true;
    }
}
