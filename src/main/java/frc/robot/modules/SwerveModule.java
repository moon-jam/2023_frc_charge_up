
/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */

package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    
    private final TalonSRX turningEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetTicks;
    double driveMotorId;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetTicks = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        this.driveMotorId = driveMotorId;
        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configOpenloopRamp(0.1);
        turningMotor.configOpenloopRamp(0.1);
        
        turningEncoder = new TalonSRX(absoluteEncoderId);
        turningEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.config_kP(0, ModuleConstants.kP, Constants.kTIMEOUT);
        driveMotor.config_kI(0, ModuleConstants.kI, Constants.kTIMEOUT);
        driveMotor.config_kD(0, ModuleConstants.kD, Constants.kTIMEOUT);
        driveMotor.config_kF(0, ModuleConstants.kF, Constants.kTIMEOUT);
        driveMotor.config_IntegralZone(0, ModuleConstants.I_Zone, Constants.kTIMEOUT);
        driveMotor.configAllowableClosedloopError(0, ModuleConstants.allow_error, Constants.kTIMEOUT);
        // driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(0.3, 0, 0.01);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putData("turningPidController", turningPidController);

        var volatage = new SupplyCurrentLimitConfiguration(true, 30, 50, 0.1);
        driveMotor.configSupplyCurrentLimit(volatage);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configVoltageCompSaturation(11);

        turningMotor.enableVoltageCompensation(true);
        turningMotor.configVoltageCompSaturation(9);

        turningMotor.configNeutralDeadband(0.1);

        turningMotor.configNominalOutputForward(0.07);
        turningMotor.configNominalOutputReverse(-0.07);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()* ModuleConstants.kDriveEncoderTicksPerPulse2RPS 
            * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return (turningEncoder.getSelectedSensorPosition()-absoluteEncoderOffsetTicks)* ModuleConstants.kTurningEncoderTicks2Rot
            * ModuleConstants.kTurningEncoderRot2Rad* (absoluteEncoderReversed? -1 : 1);
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()* ModuleConstants.kDriveEncoderTicksPerPulse2RPS 
            * ModuleConstants.kDriveEncoderRPS2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningEncoder.getSelectedSensorVelocity()* ModuleConstants.kTurningEncoderTicksPerPulse2RPS* 
            ModuleConstants.kTurningEncoderRPS2RadPerSec* (absoluteEncoderReversed? 1 : -1);
    }

    // 因為想說他不會變這麼快(relative)(0.1ms) 所以就都用absolute(4ms)
    // /**
    //  * 算出relativeEncoder & absolutionEncoder偏差弧度 
    //  * @return 正確弧度
    //  */
    // public double getAbsoluteEncoderRad() {
    //     double angle = ;
    //     angle *= 2.0 * Math.PI;
    //     angle -= absoluteEncoderOffsetRad;
    //     return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    // }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
    }

    /**
     *  轉換各個馬達的轉速和旋轉角度
     * @return
     * 每秒轉速和角度
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * 輸出
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);        //優化旋轉路徑
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond/(ModuleConstants.kDriveEncoderTicksPerPulse2RPS 
        * ModuleConstants.kDriveEncoderRPS2MeterPerSec));   //轉速
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));   //轉速->電壓(PID)

    }

    public void output(SwerveModuleState state){
        SmartDashboard.putNumber("giveSpeed[" + (int)driveMotorId/3 + "]", roundDouble(state.speedMetersPerSecond));
        SmartDashboard.putNumber("giveRad[" + (int)driveMotorId/3 + "]", roundDouble(state.angle.getRadians()));
        SmartDashboard.putNumber("getSpeed[" + (int)driveMotorId/3 + "]", roundDouble(getDriveVelocity()));
        SmartDashboard.putNumber("getRad[" + (int)driveMotorId/3 + "]", roundDouble(getTurningPosition()));
        SmartDashboard.putData("turningPID[" + (int)driveMotorId/3 + "]", turningPidController);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    private double roundDouble(double d){
        return Math.round(d*100)/100.0;
    }
}
