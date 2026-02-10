// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.LEDConfigs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PortConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  LEDPattern m_pattern = LEDConstants.defaultPattern;
  AddressableLED m_led;
  LEDConfigs configs;

  public LEDSubsystem() {
    m_led = new AddressableLED(PortConstants.ledPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
