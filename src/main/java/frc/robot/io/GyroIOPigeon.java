package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

public class GyroIOPigeon extends GyroIO {
    private Pigeon2 gyro;
    private Pigeon2SimState sim;

    public GyroIOPigeon(int id, CANBus canBus) {
        gyro = new Pigeon2(id, canBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0);

        sim = gyro.getSimState();
    }

    public GyroIOPigeon(int id, String canBus) {
        this(id, new CANBus(canBus));
    }

    public GyroIOPigeon(int id) {
        this(id, new CANBus());
    }

    @Override
    public void updateInputs() {
        inputs.connected = gyro.isConnected();
        inputs.yawPositionRad = Units.degreesToRadians(gyro.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());
        inputs.hardwareFault = gyro.getFault_Hardware().getValue();
    }

    @Override
    public void setMechYaw(double yaw) {
        sim.setRawYaw(Units.radiansToDegrees(yaw));
    }

    @Override
    public void setMechYawVelocity(double yawVelocity) {
        sim.setAngularVelocityZ(Units.radiansToDegrees(yawVelocity));
    }
}
