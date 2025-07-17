package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;

// Code from team 6328
public class PhoenixUtil {
    // Signals for refresh. We can't refresh canivore and RIO signals in one method call so they need to be stored
    // separately
    // Also BaseStatusSignal.refreshAll takes in an array so there's no point using Lists
    private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
        if (canivore) {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
            System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
            System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
            canivoreSignals = newSignals;
        } else {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
        for (BaseStatusSignal signal : signals) {
            signal.setUpdateFrequency(50);
        }
    }

    /** Refresh all registered signals. */
    public static void refreshAll() {
        if (canivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(canivoreSignals);
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
        }
    }
}
