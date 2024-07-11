package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
public final class CTREConfig {

    public static void configureCANcoder(CANcoder canCoder, boolean isInverted) {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = isInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;

        canCoderConfig.MagnetSensor = magnetSensorConfigs;

        canCoder.getConfigurator().apply(canCoderConfig);
    }

}
