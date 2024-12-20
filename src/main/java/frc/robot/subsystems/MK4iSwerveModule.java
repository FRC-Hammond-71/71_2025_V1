package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MK4iSwerveModule
{
    // Constants of physical Robot
    public static final double kAzimuthGearing = 150 / 7;
    public static final double kDriveGearing = 6.12f;
    public static final double kDriveCircumference = 0.31919f;

    private Rotation2d offset; //offset is in degrees :3

    private CANSparkMax AzimuthMotor;

    private SparkAbsoluteEncoder AbsoluteEncoder;

    private CANSparkMax DriveMotor;
    
    private PIDController AzimuthPID = new PIDController(0.05 , 0, 0);

    private SlewRateLimiter DriveRateLimiter = new SlewRateLimiter(4);

    public MK4iSwerveModule(int azimuthMotorDeviceId, int driveMotorDeviceId, int azimuthEncoderChannel, int azimuthOffset)
    {
        this.AzimuthMotor = new CANSparkMax(azimuthMotorDeviceId, MotorType.kBrushless);
        this.AzimuthMotor.setIdleMode(IdleMode.kCoast);
        this.AzimuthMotor.setInverted(true);

        this.AbsoluteEncoder = this.AzimuthMotor.getAbsoluteEncoder();

        this.offset = Rotation2d.fromDegrees(azimuthOffset);

        this.DriveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        this.DriveMotor.setIdleMode(IdleMode.kCoast);

        this.AzimuthPID.setTolerance(2);


    }

    public void Drive(SwerveModuleState state)
    {
        // state = SwerveModuleState.optimize(state, getAzimuthRotation());

        this.AzimuthPID.enableContinuousInput(-180, 180);

        double error = this.AzimuthPID.calculate(this.getAzimuthRotation().getDegrees(), state.angle.getDegrees());
        
        double controlVoltage = error * 0.95137420707;

        this.AzimuthMotor.setVoltage(controlVoltage);

        SmartDashboard.putNumber("MK4iSwerveModule Azimuth Control", error);
        SmartDashboard.putNumber("MK4iSwerveModule Azimuth Output Voltage", controlVoltage);

        double targetDriveSpeed = this.DriveRateLimiter.calculate(state.speedMetersPerSecond);

        double driveVoltage = targetDriveSpeed * 2.88f;

        this.DriveMotor.setVoltage(driveVoltage);

        SmartDashboard.putNumber("MK4iSwerveModule Drive Control", state.speedMetersPerSecond);
        SmartDashboard.putNumber("MK4iSwerveModule Drive Voltage", driveVoltage);
    }

    public Rotation2d getAzimuthRotation()
    {
        return Rotation2d.fromRotations(this.AbsoluteEncoder.getPosition()).minus(this.offset);
    }



    public double getDriveVelocity()
    {
        return this.DriveMotor.getEncoder().getVelocity() / kDriveGearing * kDriveCircumference;
    }

    public void Stop()
    {
        this.AzimuthMotor.stopMotor();
        this.DriveMotor.stopMotor();
        this.DriveRateLimiter.reset(0);
        this.AzimuthPID.setSetpoint(this.getAzimuthRotation().getDegrees());
    }
}