package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotMap.FeederMap;

import java.util.Optional;

public class Feeder extends SubsystemBase {
    private static Feeder instance;

    public static Feeder getInstance() {
        if (instance == null) instance = new Feeder();
        return instance;
    }

    public enum NoteStatus {
        EMPTY, LOADED
    }

    public enum FeederDirection {
        FORWARD, FORWARD_SLOW, REVERSE, STOPPED
    }

    private NoteStatus status = NoteStatus.EMPTY;


    private Optional<CANSparkBase> feeder;
    private Optional<SparkPIDController> feedPID;
    private Optional<DigitalInput> lowerBeamBreak;
    private Optional<DigitalInput> upperBeamBreak;

    private double feedVel;

    private Feeder() {
        super();

        if (Config.Subsystems.FEEDER_ENABLED) {
            feeder = Optional.of(new CANSparkMax(FeederMap.FEEDER, MotorType.kBrushless));

            var motor = feeder.get();
            motor.setIdleMode(IdleMode.kBrake);

            feedPID = Optional.of(motor.getPIDController());

            var motorPID = feedPID.get();
            motorPID.setP(FeederMap.FEEDER_PID.kP);
            motorPID.setI(FeederMap.FEEDER_PID.kI);
            motorPID.setD(FeederMap.FEEDER_PID.kD);
            motorPID.setFF(FeederMap.FEEDER_FF);

            motor.setClosedLoopRampRate(FeederMap.FEEDER_RAMP_RATE);
            motor.setInverted(false);

            motor.burnFlash();

            lowerBeamBreak = Optional.of(new DigitalInput(FeederMap.LOWER_BEAMBREAK));
            upperBeamBreak = Optional.of(new DigitalInput(FeederMap.UPPER_BEAMBREAK));
        } else {
            feeder = Optional.empty();
            feedPID = Optional.empty();
            lowerBeamBreak = Optional.empty();
            upperBeamBreak = Optional.empty();
        }

        var tab = Shuffleboard.getTab("Feeder");

        tab.add(this);
    }

    public boolean isNoteLoaded() {
        return status == NoteStatus.LOADED;
    }

    public boolean getUpperBeamBreak() {
        return upperBeamBreak.isPresent() ? upperBeamBreak.get().get() : false;
    }

    public void setFeederState(FeederDirection direction) {
        if (direction == FeederDirection.FORWARD) {
            feedVel = FeederMap.FEEDER_RPM;
        } else if (direction == FeederDirection.STOPPED) {
            feedVel = 0;
        } else if (direction == FeederDirection.FORWARD_SLOW) {
            feedVel = FeederMap.FEEDER_RPM_SLOW;
        } else if (direction == FeederDirection.REVERSE) {
            feedVel = FeederMap.FEEDER_RPM * -1;
        }
    }

    private void updateSpeed() {
        if (feeder.isPresent() && feedPID.isPresent()) {
            if (feedVel != 0) {
                feedPID.get().setReference(feedVel, ControlType.kVelocity);
            } else feeder.get().set(0);
        }
    }

    @Override
    public void periodic() {
        status = (lowerBeamBreak.isPresent() && lowerBeamBreak.get().get()) ? NoteStatus.EMPTY : NoteStatus.LOADED;
        // System.out.println("Beambreak " + ((beamBreak.isPresent() ? beamBreak.get().get() : false) ? "TRUE" : "FALSE or OFF"));
        updateSpeed();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("feeder velocity", () -> feedVel, (v) -> feedVel = v);
        builder.addDoubleProperty("real feeder velo",
                () -> feeder.map(canSparkBase -> canSparkBase.getEncoder().getVelocity()).orElse(0.0),
                (d) -> {
                }
        );
        builder.addBooleanProperty("has note", this::isNoteLoaded, null);
        builder.addBooleanProperty("upper beam break", this::getUpperBeamBreak, null);

        builder.addStringProperty("status", () -> status.toString(), null);
    }
}
