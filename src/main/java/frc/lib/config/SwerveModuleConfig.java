package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
        public static class ModuleConfig {
            public final int propulsionMotorID;
            public final int azimuthalMotorID;
            public final int canCoderID;
            public final boolean propulsionInvert;
            public final boolean azimuthalInvert;
            public final boolean canCoderInvert;
            public final Rotation2d angleOffset;

            public ModuleConfig(int propulsionMotorID, int azimuthalMotorID, int canCoderID,
                                boolean propulsionInvert, boolean azimuthalInvert, boolean canCoderInvert,
                                Rotation2d angleOffset) {
                this.propulsionMotorID = propulsionMotorID;
                this.azimuthalMotorID = azimuthalMotorID;
                this.canCoderID = canCoderID;
                this.propulsionInvert = propulsionInvert;
                this.azimuthalInvert = azimuthalInvert;
                this.canCoderInvert = canCoderInvert;
                this.angleOffset = angleOffset;
            }
        }
}
