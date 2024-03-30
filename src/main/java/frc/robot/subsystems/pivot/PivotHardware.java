package frc.robot.subsystems.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface PivotHardware {
    enum PivotHardwareType {
        SINGLE_ACTUATOR,
        DUAL_ACTUATOR,
        NONE

    }

    public void setVoltage(double voltage);

    public double getEncoderPosition();

    public void zeroEncoder();

    public double getForwardLimit();
    public void setForwardLimit(double limit);
    public double getReverseLimit();
    public void setReverseLimit(double limit);
}
