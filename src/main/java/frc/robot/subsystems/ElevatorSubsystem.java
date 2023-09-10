package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.modules.TalonFxMotorPIDmodule;

public class ElevatorSubsystem extends SubsystemBase {

    double elevator_left_percent = 0;
    double elevator_right_percent = 0;
    double elevator_kP = 0.4, elevator_kI = 0.01, elevator_kD = 0.05;
    Boolean elevator_pid_enable = true;
    public Boolean stay_pid_control = false;
    double pid_setpoint = 0;
    public static boolean too_high = false;
    
    
    private TalonFxMotorPIDmodule elevatorl = new TalonFxMotorPIDmodule(PortID.elevator_elongation_l_falcon500, NeutralMode.Brake, FeedbackDevice.IntegratedSensor);
    private TalonFxMotorPIDmodule elevatorr = new TalonFxMotorPIDmodule(PortID.elevator_elongation_r_falcon500, NeutralMode.Brake, FeedbackDevice.IntegratedSensor);
    private DigitalInput elevator_close_limitSwitch = new DigitalInput(PortID.elevator_close_limit_switch.port);
    private DigitalInput elevator_open_limitSwitch = new DigitalInput(PortID.elevator_opne_limit_switch.port);
    private PIDController elevator_PID = new PIDController(elevator_kP, elevator_kI, elevator_kD);

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        
        elevatorl.cv2Ticks_position(Constants.elevator_tick2rotate);
        elevatorr.cv2Ticks_position(Constants.elevator_tick2rotate);
        elevatorl.reset_position();
        elevatorr.reset_position();
        elevator_PID.setIntegratorRange(-0.5, 0.5);

        // SmartDashboard.putData("elevator_PID", elevator_PID);
        SmartDashboard.putBoolean("elevator_pid_enable", elevator_pid_enable);

        var volatage = new SupplyCurrentLimitConfiguration(true, 30, 50, 0.1);
        elevatorl.motor.configSupplyCurrentLimit(volatage);
        elevatorr.motor.configSupplyCurrentLimit(volatage);
        elevatorl.motor.enableVoltageCompensation(true);
        elevatorr.motor.enableVoltageCompensation(true);
        elevatorl.motor.configVoltageCompSaturation(10);
        elevatorr.motor.configVoltageCompSaturation(10);

        elevatorl.motor.configNeutralDeadband(0.08);
        elevatorr.motor.configNeutralDeadband(0.08);
    }
    // double min_height = 0;
    @Override
    public void periodic() {
    // This method will be called once per scheduler run

        // if(elevator_get_height()<min_height){
        //     min_height = elevator_get_height();
        // }
        SmartDashboard.putBoolean("elevator_close_limitSwitch", elevator_close_limitSwitch.get());
        SmartDashboard.putBoolean("elevator_open_limitSwitch", elevator_open_limitSwitch.get());
        SmartDashboard.putNumber("elevator_height", elevator_get_height());
        // SmartDashboard.putNumber("elevator_min_height", min_height);
        // elevator_pid_enable = SmartDashboard.getBoolean("elevator_pid_enable", false);

        if(elevator_close_limitSwitch.get()){
            // elevatorr.reset_current_position(0);
            // elevatorl.reset_current_position(0);
        }
        if(elevator_open_limitSwitch.get()){
            elevatorr.reset_current_position(-Constants.elevator_open);
            elevatorl.reset_current_position(Constants.elevator_open);
        }

        too_high = elevator_get_height() < -1.2;

        // if (elevator_pid_enable) {
        //     // Calculates the output of the PID algorithm based on the sensor reading
        //     // and sends itn to a motor
        //     elevatorl.setPercentOutput(elevator_PID.calculate(elevator_get_height(), elevator_PID.getSetpoint()));
        //     elevatorr.setPercentOutput(-elevator_PID.calculate(elevator_get_height(), elevator_PID.getSetpoint()));
        // }
        // else{
        //     elevatorl.setPercentOutput(0);
        //     elevatorr.setPercentOutput(0);
        // }

        // if(stay_pid_control){
        //     elevator_setPidPosition(pid_setpoint);
        // }
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public double elevator_get_height(){
        return (elevatorl.get_position()+(-elevatorr.get_position()))/2;
    }

    public void elevator_stop(){
        elevatorl.setPercentOutput(0);
        elevatorr.setPercentOutput(0);
    }

    public void elevator_setPercentOutput(double speed){
        elevatorl.setPercentOutput(speed);
        elevatorr.setPercentOutput(-speed);
    }

    public boolean elevator_setPidPosition(double position){
        double output = ((elevator_PID.calculate(elevator_get_height(), position)));
        elevator_setPercentOutput(output);
        return Math.abs(elevator_get_height() - Constants.elevator_open)<0.5;
    }

    public void setPoint(double point) {
        stay_pid_control = true;
        if(point < -1 && pid_setpoint+point>=-5.4)
            pid_setpoint = point;
        if(point > -1){
            stay_pid_control = false;
            elevator_stop();
        }
  
    }

    public boolean elevator_up(){
        stay_pid_control = false;
        double output = ((elevator_PID.calculate(elevator_get_height(), Constants.elevator_open+0.1)));
        elevator_setPercentOutput(output+0.2);
        return Math.abs(elevator_get_height() - Constants.elevator_open)<0.5;
    }

    public boolean elevator_middle(){
        stay_pid_control = false;
        double output = ((elevator_PID.calculate(elevator_get_height(), Constants.elevator_middle)));
        elevator_setPercentOutput(output);
        return elevator_get_height()<Constants.elevator_middle+0.2;
    }

    public boolean elevator_down(){
        stay_pid_control = false;
        if(elevator_get_height()<0){
            elevatorl.setPercentOutput(0.3);
            elevatorr.setPercentOutput(-0.3);
            return false;
        }
        else{
            elevator_stop();
            return true;
        }
    
    }

    public void elevator_stupid_up(){
        elevatorl.setPercentOutput(-0.5);
        elevatorr.setPercentOutput(0.5);
    }

    public void elevator_stupid_down(){
        elevatorl.setPercentOutput(0.3);
        elevatorr.setPercentOutput(-0.3);
    }
}

