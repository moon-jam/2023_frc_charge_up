package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.modules.TalonFxMotorPIDmodule;

public class RackSubsystem extends SubsystemBase {

    double elongation_kP = 0.25, elongation_kI = 0, elongation_kD = 0;
    Boolean elongation_pid_enable = false, stay_close = false;

    public TalonFxMotorPIDmodule elongation = new TalonFxMotorPIDmodule(PortID.rack_elongation_falcon500, NeutralMode.Brake, FeedbackDevice.IntegratedSensor);
    private DigitalInput elongation_close_limitSwitch = new DigitalInput(PortID.rack_close_limit_switch.port);
    private DigitalInput elongation_open_limitSwitch = new DigitalInput(PortID.rack_opne_limit_switch.port);

    private PIDController elongation_PID = new PIDController(elongation_kP, elongation_kI, elongation_kD);
    
    /** Creates a new RackSubsystem. */
    public RackSubsystem() {
        SmartDashboard.putBoolean("lift_zero_enable", false);
       
        elongation.reset_position();
        elongation.cv2Ticks_position(Constants.rack_tick2rotate);
        elongation_PID.setIntegratorRange(-0.5, 0.5);
        
        // SmartDashboard.putData("elongation_PID", elongation_PID);
        // SmartDashboard.putBoolean("elongation_pid_enable", elongation_pid_enable);
    }

    double min_length = 0;
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("elongation_length", elongation_get_length());

        // if(elongation_get_length()<min_length){
        //     min_length = elongation_get_length();
        // }

        SmartDashboard.putBoolean("elongation_close_limitSwitch", elongation_close_limitSwitch.get());
        SmartDashboard.putBoolean("elongation_open_limitSwitch", elongation_open_limitSwitch.get());
        // SmartDashboard.putNumber("elongation_min_length", min_length);
        // elongation_pid_enable = SmartDashboard.getBoolean("elongation_pid_enable", false);
        
        if(elongation_close_limitSwitch.get()){
            elongation.reset_current_position(0);
        }
        if(elongation_open_limitSwitch.get()){
            elongation.reset_current_position(Constants.rack_open);
        }

        // if (elongation_pid_enable) {
        //     // Calculates the output of the PID algorithm based on the sensor reading
        //     // and sends it to a motor
        //     elongation.setPercentOutput(elongation_PID.calculate(elongation_get_length(), elongation_PID.getSetpoint()));
        // }
        // else{
        //     elongation.setPercentOutput(0);
        // }
        if(stay_close){
            double out = elongation_PID.calculate(elongation_get_length(), 0);
            if(out > 0.4)   out=0.4;
            elongation.setPercentOutput(out);
        }
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public double elongation_get_length(){
        return elongation.get_position();
    }

    public boolean rack_open(){
        stay_close = false;
        if(!elongation_open_limitSwitch.get()){
            elongation.setPercentOutput(-0.4);
            return false;
        }
        else{
            elongation.setPercentOutput(0);
            // elongation.setPercentOutput(elongation_PID.calculate(elongation_get_length(), Constants.elevator_open));
            return true;
        }
    }

    public boolean rack_close(){
        if(!elongation_close_limitSwitch.get()){
            stay_close = false;
            elongation.setPercentOutput(0.4);
            return false;
        }
        else{
            stay_close = true;
            return true;
        }
    }

    public boolean rack_open_auto(double output){
        stay_close = false;
        if(!elongation_open_limitSwitch.get()){
            elongation.setPercentOutput(-1*output);
            return false;
        }
        else{
            elongation.setPercentOutput(0);
            // elongation.setPercentOutput(elongation_PID.calculate(elongation_get_length(), Constants.elevator_open));
            return true;
        }
    }

    public boolean rack_close_auto(double output){
        stay_close = false;
        if(!elongation_close_limitSwitch.get()){
            elongation.setPercentOutput(output);
            return false;
        }
        else{
            elongation.setPercentOutput(elongation_PID.calculate(elongation_get_length(), 0));
            return true;
        }
    }

    public void rack_stay_close(){
        stay_close = true;
    }

    public void rack_stop(){
        stay_close = false;
        elongation.setPercentOutput(0);
    }

    public void rack_stop_close(){
        elongation.setPercentOutput(0);
    }

    public void set_elongation_NeutralMode(NeutralMode mode){
        elongation.motor.setNeutralMode(mode);
    }
}

