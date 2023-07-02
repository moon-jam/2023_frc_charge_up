/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.modules.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

    public double gyro_pitch_offset = 0;

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveReversed,
            DriveConstants.kFrontLeftTurningReversed,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderPort,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveReversed,
            DriveConstants.kFrontRightTurningReversed,
            DriveConstants.kFrontRightTurningAbsoluteEncoderPort,
            DriveConstants.kFrontRightTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kFrontRightTurningAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveReversed,
            DriveConstants.kBackLeftTurningReversed,
            DriveConstants.kBackLeftTurningAbsoluteEncoderPort,
            DriveConstants.kBackLeftTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kBackLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveReversed,
            DriveConstants.kBackRightTurningReversed,
            DriveConstants.kBackRightTurningAbsoluteEncoderPort,
            DriveConstants.kBackRightTurningAbsoluteEncoderOffsetTicks,
            DriveConstants.kBackRightTurningAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

    public SwerveSubsystem() {
        /**
         * 10:11
         * 1.因gyro啟動時需時間校準
         * 2.等一秒後再重製
         * 3.為避免產生錯誤用try...catch
         * 4.為讓程式不被等的那一秒delay 使用Thread讓程式在另一個核心執行
         */
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro_pitch_offset = 0;
        gyro.reset();
    }

    /**
     * @return 取得車頭角度
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle()+gyro_pitch_offset, 360);    //轉成同位角(>=0deg <=360deg)
    }

    /**
     * 11:04
     * 陀螺儀要得到機器航向所用
     * @return rotation2d(角度)
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose);
    }

    @Override
    public void periodic() {
        odometer.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });
        SmartDashboard.putData("Robot Heading_", gyro);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //如果大於最大速度 全部等比例縮小
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        frontLeft.output(desiredStates[0]);
        frontRight.output(desiredStates[1]);
        backLeft.output(desiredStates[2]);
        backRight.output(desiredStates[3]);
    }

    public void resetAllEncoder(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public double getGyroPitch(){
        return 0;
        // return (gyro.getPitch()-0.13);
    }

    public double getGyroRoll(){
        return 0;
        // return gyro.getRoll();
    }
}
