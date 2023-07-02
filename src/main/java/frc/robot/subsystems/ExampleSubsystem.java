package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.TalonFxMotorPIDmodule;

public class ExampleSubsystem extends SubsystemBase {
  public TalonFxMotorPIDmodule tt = new TalonFxMotorPIDmodule(null, null, null);
  TalonFX motor = new TalonFX(0);
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    
  }

  public CommandBase setMotorPercentOutput(double output){
    return runOnce(()->{motor.set(ControlMode.PercentOutput, output);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}


