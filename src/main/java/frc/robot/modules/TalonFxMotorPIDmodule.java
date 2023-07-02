// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.PortID;
import frc.robot.Constants;

public class TalonFxMotorPIDmodule {
  public TalonFX motor;
  private double cvtTicks_velocity = 1;
  private double cvtTicks_position = 1;
  /** Creates a new ExampleSubsystem. */

  public TalonFxMotorPIDmodule(PortID id, NeutralMode NeutralMode, FeedbackDevice feedbackDevice) {
    motor = TalonFXInitModule(id, NeutralMode, feedbackDevice);
  }

  public void cv2Ticks_velocity(double cvtTicks_velocity){
    this.cvtTicks_velocity = cvtTicks_velocity;
  }

  public void cv2Ticks_position(double cvtTicks_position){
    this.cvtTicks_position = cvtTicks_position;
  }

  public void setVelocity(double Unit){
    motor.set(ControlMode.Velocity, (int)(Unit* cvtTicks_velocity));
  }
  
  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void set_position(double Unit){
    motor.set(ControlMode.Position, (int)(Unit* cvtTicks_position));
  }

  public double get_Velocity(){
    return motor.getSelectedSensorVelocity()/cvtTicks_velocity;
  }
  
  public void setPercentOutput(double PercentOutput){
    if(Math.abs(PercentOutput)<0.05) PercentOutput = 0;
    motor.set(ControlMode.PercentOutput, PercentOutput);
  }

  public void reset_position(){
    motor.setSelectedSensorPosition(0);
  }

  public void reset_current_position(double Unit){
    motor.setSelectedSensorPosition((int)(Unit* cvtTicks_position));
  }

  public double get_position(){
    return motor.getSelectedSensorPosition()/cvtTicks_position;
  }

  public double get_OutputPercent(){
    return motor.getMotorOutputPercent();
  }

  private TalonFX TalonFXInitModule(PortID id, NeutralMode NeutralMode, FeedbackDevice feedbackDevice) {
    TalonFX talonfx = new TalonFX(id.port);
    talonfx.configFactoryDefault();
    talonfx.setInverted(id.reversed);
    talonfx.setNeutralMode(NeutralMode);
    talonfx.configClosedloopRamp(id.ramp_rate);

    talonfx.configSelectedFeedbackSensor(feedbackDevice, 0, Constants.kTIMEOUT);
    talonfx.config_kP(0, id.kP, Constants.kTIMEOUT);
    talonfx.config_kI(0, id.kI, Constants.kTIMEOUT);
    talonfx.config_kD(0, id.kD, Constants.kTIMEOUT);
    talonfx.config_kF(0, id.kF, Constants.kTIMEOUT);
    talonfx.config_IntegralZone(0, id.I_Zone, Constants.kTIMEOUT);
    talonfx.configAllowableClosedloopError(0, id.allow_error, Constants.kTIMEOUT);
    
    return talonfx;
  }

}
