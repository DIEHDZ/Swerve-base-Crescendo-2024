package frc.robot.constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class ConstantsSwerve {
    public static final class Swerve {

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(10.875 * 2);
        public static final double wheelBase = Units.inchesToMeters(10.875 * 2);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.12;

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        
        //Swerve Voltage Compensation
        public static final double voltafeComp = 12.0;

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        //Deadband
        public static final double stickDeadband = 0.05; 

        // Azimuthal motor PID values
        public static final double azimuthalKP = 0.01;
        public static final double azimuthalKI = 0.0;
        public static final double azimuthalKD = 0.0;
        public static final double azimuthalKFF = 0.0;

        // Propulsion motor PID values
        public static final double propulsionlKP = 0.01;
        public static final double propulsionKI = 0.0;
        public static final double propulsionKD = 0.0;
        public static final double propulsionKFF = 0.0;

        // Drive Motor Conversion Factors
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

        // Swerve Profiling Values
        public static final double maxSpeed = 5; // meters per second
        public static final double maxAngularVelocity = 10;

        // Neutral Modes
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        // Gyroscope Constants
        public static final class Gyroscope {
            public static final AHRS gyro = new AHRS();
            public static final gyroscope_convention convention = gyroscope_convention.RIGHT_IS_POSITIVE;
        } 

        // Gyroscope convention enumeration
        public enum gyroscope_convention {
            LEFT_IS_POSITIVE(1),
            RIGHT_IS_POSITIVE(-1);

            public final int value;

            gyroscope_convention(int value) {
                this.value = value;
            }
        }

        // Modules

        // Front Right Module - Module 0
        public static final class Mod0 {
            public static final int propulsionMotorID = 11;
            public static final int azimuthalMotorID = 12;
            public static final int canCoderID = 13;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Front Left Module - Module 1
        public static final class Mod1 {
            public static final int propulsionMotorID = 21;
            public static final int azimuthalMotorID = 22;
            public static final int canCoderID = 23;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Back Left Module - Module 2
        public static final class Mod2 {
            public static final int propulsionMotorID = 31;
            public static final int azimuthalMotorID = 32;
            public static final int canCoderID = 33;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Back Right Module - Module 3
        public static final class Mod3 {
            public static final int propulsionMotorID = 41;
            public static final int azimuthalMotorID = 42;
            public static final int canCoderID = 43;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }
    }
}
