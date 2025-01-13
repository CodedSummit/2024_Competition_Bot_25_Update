// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeNoteCommand;

/**
 * The subsystem responsible for picking up Notes
 * 
 * Knows if it holds a Note or not
 * Can release a Note to other systems (or maybe just know that it was taken)
 * When releasing the Note to the Shooter subsystem, that triggers the shot (i.e. Shooter 
 * does not hold the Note)
 * 
 * 
 */
public class IntakeSubsystem extends SubsystemBase {

  private boolean m_hasNote = false; // is known to be holding a Note
  private SparkMax m_motor  = new SparkMax(IntakeConstants.kIntakeCanbusID, MotorType.kBrushless);
  private DigitalInput m_beamSwitch = new DigitalInput(IntakeConstants.kIntakeBeambreakID);
  private double m_intakeSpeed;
  private GenericEntry nt_intakeSpeed;
  private AddressableLedSubsystem m_led;

  public IntakeSubsystem(AddressableLedSubsystem led) {
    m_led = led;
    loadPreferences();
    setupShuffleboard();
  }

  private void loadPreferences() {
    m_intakeSpeed = Preferences.getDouble(IntakeConstants.kIntakeSpeedPrefKey, IntakeConstants.kIntakeSpeed);
  }

  private void setupShuffleboard() {
    
    ShuffleboardTab intake = Shuffleboard.getTab("Intake");
    try {
      // TODO - consider only setting up Shuffleboard for the speed in non-competition configuration
      //  to avoid inadvertent mucking with the speed in a competition
      nt_intakeSpeed = intake.addPersistent("Intake speed", m_intakeSpeed)
        .withSize(3,1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

      intake.add("Start Intake",new InstantCommand(() ->setSpeed())).withPosition(0,1);
      intake.add("Cancel Intake", new InstantCommand(() ->stop())).withPosition(1,1);
      intake.add("feed note", pickupPiece());
      intake.addBoolean("Has Note", () -> m_hasNote).withPosition(3, 0);



      

      intake.addString("Current Command", () -> current_command_name());

    } catch (Exception e) {// eat it.  for some reason it fails if the tab exists
    }
  }
  public String current_command_name(){
          String command_name = "---";
      if(this.getCurrentCommand() != null){
        command_name = this.getCurrentCommand().getName();
      }
      return command_name;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hasNote = !m_beamSwitch.get();  // assumes switch "true" means the beam is NOT broken (Thus no Note)
  
    if (hasNote()){
      m_led.setStripOrange();
    } else {
      m_led.setStripOff();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getIntakeSpeed() {

    if (m_intakeSpeed != nt_intakeSpeed.getDouble(IntakeConstants.kIntakeSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_intakeSpeed = nt_intakeSpeed.getDouble(IntakeConstants.kIntakeSpeed);
      Preferences.setDouble(IntakeConstants.kIntakeSpeedPrefKey, m_intakeSpeed);
      System.out.println(" intake speed changed to "+m_intakeSpeed);
    }
    
    return m_intakeSpeed;
  }
  public boolean hasNote() {
    // TODO - implement. Return t/f if a Note is in the handler
    return m_hasNote;
  }

  /**
   * Starts up the intake motor

  */
  public void setSpeed() {
    // TODO - implement
    m_motor.set(getIntakeSpeed());
    System.out.println(" Set intake speed to:"+m_intakeSpeed);
  }

  public void reverseIntake() {
    m_motor.set(-getIntakeSpeed());
    System.out.println(" Reversing!!!!!!");
  }

  public void feedShooter(){
    m_motor.set(IntakeConstants.kFeedShooterSpeed); //speed for feeding shooter
  }

  public void feedArm(){
    m_motor.set(IntakeConstants.kFeedArmSpeed);
  }

  /**
   * Stops the intake motor
   */
  public void stop() {
    // TODO -implement
    m_motor.set(0.0);
    System.out.println(" Stopping Intake");
  }

  /**
   * Make a command to pickup a Note
   * @return IntakeNoteCommand
   */
  public Command pickupPiece() {
    return new IntakeNoteCommand(this);
  }

}
