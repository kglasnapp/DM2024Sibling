package frc.robot.utilities;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

public class EncoderSync {
    private DoubleSupplier integratedEncoderReading;
    private DoubleSupplier externalEncoderReading;
    private DoubleConsumer integratedAngleSetter;
    private double alpha;
    private double cutoff;

    // Alpha: Controls how quickly the intrgrated encoder should approch the
    // external encoder
    // Cutoff: When the deivance is larger than this value, set the integrated
    // encoder directly to the external encoder
    public EncoderSync(DoubleSupplier integratedEncoderReading, DoubleSupplier externalEncoderReading,
            DoubleConsumer integratedAngleSetter, double alpha, double cutoff) {
        this.integratedEncoderReading = integratedEncoderReading;
        this.externalEncoderReading = externalEncoderReading;
        this.integratedAngleSetter = integratedAngleSetter;
        this.alpha = alpha;
        this.cutoff = cutoff;
    }

    public void update() {
        double absolutePosition = externalEncoderReading.getAsDouble();
        double relativePosition = integratedEncoderReading.getAsDouble();
        double delta = Util.normalizeAngle(absolutePosition - relativePosition);
        // if (Math.abs(delta) < cutoff) {
            // Nudge relative encoder towards correct position
            integratedAngleSetter.accept(Util.normalizeAngle(delta * alpha + relativePosition));
        // } else {
            // integratedAngleSetter.accept(absolutePosition);
        // }
    }
}
