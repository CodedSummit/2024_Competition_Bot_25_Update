// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Models the mechanism that is used to climb at the end of a match.
 * 
 * This will need to 'extend' to push the hooks up, and then 'climb' which will
 * pull the robot up.
 * lowering is just technically 'extending'. after the match, robots are not
 * activated again.
 * We'll either need to lift the robot off the chain (likely the best answer) or
 * use a ratchet to manually
 * adjust the motor (it's a worm gearbox, so not backdrivable).
 */

 
public class ClimbSubsystem extends SubsystemBase {
private TalonFX m_motor = new TalonFX(ClimbConstants.kClimbCanbusID, "rio");
  private double m_climbSpeed;
  private GenericEntry nt_climbSpeed;
  
  public ClimbSubsystem() {
    loadPreferences();
    setupShuffleboard();
  }

  private void loadPreferences() {
    m_climbSpeed = Preferences.getDouble(ClimbConstants.kClimbSpeedPrefKey, ClimbConstants.kClimbSpeed);
  }

  private void setupShuffleboard() {

    ShuffleboardTab climbTab = Shuffleboard.getTab("Climb");

    nt_climbSpeed = climbTab.addPersistent("Climber speed", m_climbSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    
    climbTab.add("Lower", Commands.startEnd(
      () -> lower(), 
      () -> stop(), this))
      .withWidget(BuiltInWidgets.kToggleButton); 
    climbTab.add("Climb", Commands.startEnd(
      () -> climb(), 
      () -> stop(), this)); 
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Climb (raise the robot on the hook). Assumes the arm/hook has already been
   * positioned correctly.
   */
  public void climb() {
    // TODO - implement
    m_motor.set(getClimbSpeed());
  }
  public double getClimbSpeed() {

    if (m_climbSpeed != nt_climbSpeed.getDouble(ClimbConstants.kClimbSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_climbSpeed = nt_climbSpeed.getDouble(ClimbConstants.kClimbSpeed);
      Preferences.setDouble(ClimbConstants.kClimbSpeedPrefKey, m_climbSpeed);
    }
    return m_climbSpeed;
  } 

  public void lower() {
    m_motor.set(-getClimbSpeed());
  }

  
  public void stop() {
    m_motor.set(0.0);
  }
}
