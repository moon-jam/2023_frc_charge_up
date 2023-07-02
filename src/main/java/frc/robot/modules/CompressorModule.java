// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorModule {
  private Compressor airCompressor;
  private boolean isCompressorOn;
  /** Creates a new ExampleSubsystem. */

  public CompressorModule() {
    airCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
    compressor();
  }

  public void compressor() {
      isCompressorOn = true;
      airCompressor.enableDigital();
  }

  public void compressorStop() {
      isCompressorOn = false;
      airCompressor.disable();
  }

  public void toggleCompressor() {
      if (isCompressorOn)
          compressorStop();
      else
          compressor();
  }

}
