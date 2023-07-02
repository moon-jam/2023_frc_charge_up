// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.PortID;

public class VictorSPXMotorModule {
  private VictorSPX motor;
  /** Creates a new ExampleSubsystem. */

  public VictorSPXMotorModule(PortID id, NeutralMode NeutralMode) {
    motor = VictorSPXInitModule(id, NeutralMode);
  }

  public void setPercentOutput(double PercentOutput){
    motor.set(ControlMode.PercentOutput, PercentOutput);
  }
  
  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  private VictorSPX VictorSPXInitModule(PortID id, NeutralMode NeutralMode) {
    VictorSPX victorSPX = new VictorSPX(id.port);
    victorSPX.configFactoryDefault();
    victorSPX.setInverted(id.reversed);
    victorSPX.setNeutralMode(NeutralMode);
    victorSPX.configClosedloopRamp(id.ramp_rate);
    
    return victorSPX;
  }

}
