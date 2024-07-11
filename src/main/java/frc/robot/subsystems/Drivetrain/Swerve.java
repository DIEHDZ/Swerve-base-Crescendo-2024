package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstantsSwerve;
import frc.lib.config.SwerveModuleConfig.ModuleConfig;

public class Swerve extends SubsystemBase {
    private final AHRS gyro;
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;
    private final ShuffleboardTab fieldTab;

    public Swerve() {
        gyro = new AHRS();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(
            ConstantsSwerve.Swerve.swerveKinematics,
            getYaw(),
            new SwerveModulePosition[] { // Initialize with zeros 
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0))
            }
        );

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, new ModuleConfig(
                ConstantsSwerve.Swerve.Mod0.propulsionMotorID,
                ConstantsSwerve.Swerve.Mod0.azimuthalMotorID,
                ConstantsSwerve.Swerve.Mod0.canCoderID,
                ConstantsSwerve.Swerve.Mod0.propulsionInvert,
                ConstantsSwerve.Swerve.Mod0.azimuthalInvert,
                ConstantsSwerve.Swerve.Mod0.canCoderInvert,
                ConstantsSwerve.Swerve.Mod0.angleOffset
            )),
            new SwerveModule(1, new ModuleConfig(
                ConstantsSwerve.Swerve.Mod1.propulsionMotorID,
                ConstantsSwerve.Swerve.Mod1.azimuthalMotorID,
                ConstantsSwerve.Swerve.Mod1.canCoderID,
                ConstantsSwerve.Swerve.Mod1.propulsionInvert,
                ConstantsSwerve.Swerve.Mod1.azimuthalInvert,
                ConstantsSwerve.Swerve.Mod1.canCoderInvert,
                ConstantsSwerve.Swerve.Mod1.angleOffset
            )),
            new SwerveModule(2, new ModuleConfig(
                ConstantsSwerve.Swerve.Mod2.propulsionMotorID,
                ConstantsSwerve.Swerve.Mod2.azimuthalMotorID,
                ConstantsSwerve.Swerve.Mod2.canCoderID,
                ConstantsSwerve.Swerve.Mod2.propulsionInvert,
                ConstantsSwerve.Swerve.Mod2.azimuthalInvert,
                ConstantsSwerve.Swerve.Mod2.canCoderInvert,
                ConstantsSwerve.Swerve.Mod2.angleOffset
            )),
            new SwerveModule(3, new ModuleConfig(
                ConstantsSwerve.Swerve.Mod3.propulsionMotorID,
                ConstantsSwerve.Swerve.Mod3.azimuthalMotorID,
                ConstantsSwerve.Swerve.Mod3.canCoderID,
                ConstantsSwerve.Swerve.Mod3.propulsionInvert,
                ConstantsSwerve.Swerve.Mod3.azimuthalInvert,
                ConstantsSwerve.Swerve.Mod3.canCoderInvert,
                ConstantsSwerve.Swerve.Mod3.angleOffset
            ))
        };

        fieldTab = Shuffleboard.getTab("Field");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = ConstantsSwerve.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ConstantsSwerve.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = new SwerveModulePosition(
                mSwerveMods[i].getPosition(),
                mSwerveMods[i].getAngle()
            );
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (ConstantsSwerve.Swerve.Gyroscope.convention == ConstantsSwerve.Swerve.gyroscope_convention.RIGHT_IS_POSITIVE)
            ? Rotation2d.fromDegrees(-gyro.getYaw())
            : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        
    // Update Shuffleboard with the current robot pose and gyro angle
    fieldTab.add("Robot Pose", getPose());
    fieldTab.add("Gyro Angle", getYaw().getDegrees());

    // Update module-specific data on the Shuffleboard
    for (SwerveModule mod : mSwerveMods) {
        fieldTab.add("Mod " + mod.moduleNumber + " CANCoder", mod.getCanCoder().getDegrees());
        fieldTab.add("Mod " + mod.moduleNumber + " Integrated", mod.getAngle().getDegrees());
        fieldTab.add("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
