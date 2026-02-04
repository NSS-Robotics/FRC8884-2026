package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final CANdle candle = new CANdle(23);

    // Inclusive ranges
    private final int onboardStart = 0;
    private final int onboardEnd = 7;
    private final int externalStart = 8;
    private final int externalEnd = 8 + 154;

    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(config);

        for (int i = onboardStart; i < onboardEnd; i++) {
            candle.setControl(new EmptyAnimation(i));
        }

        candle.setControl(new SolidColor(onboardStart, onboardEnd).withColor(new RGBWColor(64, 0, 128)));
    }

    @Override
    public void periodic() {
        candle.setControl(new RainbowAnimation(externalStart, externalEnd));
    }
}
