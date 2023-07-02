package frc.robot.subsystems;

import frc.robot.Constants.PortID;
import frc.robot.Constants.intake_position;
import frc.robot.modules.*;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    public TalonFxMotorPIDmodule lift = new TalonFxMotorPIDmodule(PortID.intake_lift_falcon500, NeutralMode.Brake,
            FeedbackDevice.IntegratedSensor);
    TalonSRX liftEncoder = new TalonSRX(PortID.intake_encoder.port);
    private CANSparkMax suck = new CANSparkMax(PortID.intake_suck_neo550.port, MotorType.kBrushless);
    SlewRateLimiter filter = new SlewRateLimiter(0.85);

    double lift_kP = -0.0004, lift_kI = 0, lift_kD = 0, lift_kF = 0;
    PIDController lift_PID = new PIDController(lift_kP, lift_kI, lift_kD);
    boolean lift_pid_enable = false;
    boolean lift_zero_enable = true;

    public Boolean stay_pid_control = true;
    double pid_setpoint = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        liftEncoder.setSelectedSensorPosition(-200);
        lift_PID.setIntegratorRange(-0.5, 0.5);
        SmartDashboard.putData("lift_PID", lift_PID);
        SmartDashboard.putNumber("lift_kF", lift_kF);
        SmartDashboard.putBoolean("lift_zero_enable", false);
        SmartDashboard.putBoolean("lift_pid_enable", false);

        // var volatage = new SupplyCurrentLimitConfiguration(true, 30, 50, 0.1);
        // lift.motor.configSupplyCurrentLimit(volatage);
        // lift.motor.enableVoltageCompensation(true);
        // lift.motor.configVoltageCompSaturation(11);

        lift.motor.configNeutralDeadband(0.05);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("lift_position", getLiftPosition());

        lift_pid_enable = SmartDashboard.getBoolean("lift_pid_enable", false);
        lift_zero_enable = SmartDashboard.getBoolean("lift_zero_enable", false);
        lift_kF = SmartDashboard.getNumber("lift_kF", 0);

        // if (lift_pid_enable) {
        //     lift_zero_enable = true;
        //     // Calculates the output of the PID algorithm based on the sensor reading
        //     // and sends it to a motor
        //     double lift_output = Math.abs(lift_PID.calculate(getLiftPosition(), lift_PID.getSetpoint()) + lift_kF)>0.8? 
        //     Math.abs(lift_PID.calculate(getLiftPosition(), lift_PID.getSetpoint()) + lift_kF)/(lift_PID.calculate(getLiftPosition(), lift_PID.getSetpoint()) + lift_kF)*0.8:
        //     (lift_PID.calculate(getLiftPosition(), lift_PID.getSetpoint()) + lift_kF);
        //     lift.setPercentOutput(filter.calculate(lift_output));
        // } else {
        //     if (lift_zero_enable) {
        //         lift.setPercentOutput(0);
        //     }
        // }

        // if(stay_pid_control){
        //     set_lift_position(pid_setpoint);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void eatCube() {
        suck.set(0.5);
    }

    public void eatCon() {
        suck.set(-0.5);
    }

    public void putCube() {
        suck.set(-0.5);
    }

    public void putCon() {
        suck.set(0.5);
    }

    public void suckStop() {
        suck.set(0);
    }

    public double getLiftPosition() {
        double read = liftEncoder.getSelectedSensorPosition() - Constants.intakeAbsoluteEncoderOffsetTicks;
        // if(Math.abs(read)>3500){
        //     if(read>0)
        //         read -= 4096;
        //     else if(read<0)
        //         read += 4096;
        // }
        return read;
    }

    public boolean set_lift_position(double position) {
        double lift_output = Math.abs(lift_PID.calculate(getLiftPosition(), position) + lift_kF)>0.8? 
        Math.abs(lift_PID.calculate(getLiftPosition(), position) + lift_kF)/(lift_PID.calculate(getLiftPosition(), position) + lift_kF)*0.8:
        (lift_PID.calculate(getLiftPosition(), position) + lift_kF);
        lift.setPercentOutput(filter.calculate(lift_output));
        SmartDashboard.putNumber("lift_output", filter.calculate(lift_output));
        return Math.abs(getLiftPosition() - position) < 90;
    }

    public boolean set_lift_position(intake_position position) {
        double lift_output = Math.abs(lift_PID.calculate(getLiftPosition(), position.value) + lift_kF)>0.8? 
        Math.abs(lift_PID.calculate(getLiftPosition(), position.value) + lift_kF)/(lift_PID.calculate(getLiftPosition(), position.value) + lift_kF)*0.8:
        (lift_PID.calculate(getLiftPosition(), position.value) + lift_kF);
        lift.setPercentOutput(filter.calculate(lift_output));
        SmartDashboard.putNumber("lift_output", filter.calculate(lift_output));
        return Math.abs(getLiftPosition() - position.value) < 90;
    }

    public void lift_stop() {
        lift.setPercentOutput(0);
    }
    
    public void addPoint(double point) {
        stay_pid_control = true;
        if(pid_setpoint+point > 0 && pid_setpoint+point<950)
            pid_setpoint += point;
        else if(pid_setpoint+point <= 0 && point > 0){
            stay_pid_control = false;
            lift_stop();
        }
    }

    public boolean set_lift_straight() {
        if(Math.abs(getLiftPosition() - intake_position.intake_substation_cube.value) < 100){
            set_lift_position(intake_position.intake_straight);
        }else{
            set_lift_position(intake_position.intake_substation_cube);
        }
        return Math.abs(getLiftPosition() - intake_position.intake_straight.value) < 90;
    }

    public boolean set_lift_floor() {
        if(Math.abs(getLiftPosition() - intake_position.intake_straight.value) < 100){
            lift.setPercentOutput(-0.05);
        }else if(Math.abs(getLiftPosition() - intake_position.intake_floor.value) < 90){
            lift.setPercentOutput(0);
        }else{
            set_lift_straight();
        }
        return Math.abs(getLiftPosition() - intake_position.intake_floor.value) < 90;
    }

    public void set_lift_NeutralMode(NeutralMode mode){
        lift.motor.setNeutralMode(mode);
    }

    public void up(){
    
    }
}
