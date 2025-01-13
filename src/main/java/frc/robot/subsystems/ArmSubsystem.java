// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

/**
 * Models the Arm that is used for putting Notes in the Amp, as well as
 * positioning the Climbing hook at the end of the match.
 * 
 * Also has an absolute position sensor and motor to change positions.
 * This angle change position will need a PID controller. Uses FeedForward in
 * addition to PID control
 * This will likely have some preset positions, but also need to use controller
 * buttons to feed forward/backward slowly to adjust (see Bump functions).
 * 
 * Must be able to feed a Note either direction - forward into an Amp, Backward into the Stage Trap at the end of match
 * 
 * see example at
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbot
 * 
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {

  // TODO - set real motor type, real encoder type and various constants
  private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorCANbusID, MotorType.kBrushless);  // motor that moves the arm
  private final SparkMax m_armHandlerMotor = new SparkMax(ArmConstants.kArmHandlerMotorCANbusID, MotorType.kBrushless); // motor to handle the Note
  private final RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private final SparkMaxConfig m_config = new SparkMaxConfig();
  
  //private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private double m_handlerSpeed = ArmConstants.kHandlerDefaultSpeed;
  private GenericEntry nt_handlerSpeed;

  /** Creates a new . */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
                0);
    m_config.smartCurrentLimit(10);
    loadPreferences();
    setupShuffleboard();
  }

  private void loadPreferences() {
     m_handlerSpeed = Preferences.getDouble(ArmConstants.kArmHandlerSpeedPrefKey, ArmConstants.kHandlerDefaultSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // TODO - implement.
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    double target_voltage = output + feedforward;
    
    //limit max voltage at point of applying to motor.
    target_voltage = Math.min(target_voltage, 0.2);

    m_armMotor.setVoltage(target_voltage);
  }

  @Override
  public double getMeasurement() {
    // TODO - implement
    return m_encoder.getPosition() + ArmConstants.kArmOffsetRads;
  }

  
  /**
   * Raise the Arm by a (small) fixed angle to fine-tune a position
   * 
   */
  public void bumpArmUp() {
    // TODO - implement
    // maybe get the current goal from the controller and increment the angle and reset the goal?
     TrapezoidProfile.State state = m_controller.getGoal();
     double newGoal = state.position + ArmConstants.kArmBumpIncrementRad;
     if ( Math.toDegrees(newGoal) > ArmConstants.kMaxArmAngleDeg) newGoal = Math.toRadians(ArmConstants.kMaxArmAngleDeg);
     setGoal(newGoal);
  }

  /**
   * Lower the Arm by a (small) fixed angle to fine-tune a position
   * 
   */
  public void bumpArmDown() {
    // TODO - implement
    TrapezoidProfile.State state = m_controller.getGoal();
    double newGoal = state.position - ArmConstants.kArmBumpIncrementRad;
    if ( Math.toDegrees(newGoal) < ArmConstants.kMinArmAngleDeg) newGoal = Math.toRadians(ArmConstants.kMinArmAngleDeg);
    setGoal(newGoal);
  }


  public void manualArmUp(){
     m_armMotor.set(1);
  }
  public void manualArmDown(){
    m_armMotor.set(-1);
  }
 
  public void manualArmStop(){
    m_armMotor.stopMotor();
  }

  public double manualPosition(){
    //0 is home, at the bottom
    //285 is near the top
    //480 is about max on the other side.
    //Warning! this encoder value depends on where the arm is when the robot turns on
    //   make sure it is at home when turned on, or the values will not match.
    return m_encoder.getPosition();
  }

  public boolean manualAtHome(){
    return manualPosition() < 10;
  }

  public void manualDropPiece(){
    if(manualPosition() < 30){
      //piece up
      handlerMotorDriveForward();
    } else if(manualPosition() < 219){
      //piece down on front side
      handlerMotorDriveBackward();
    } else {
      //piece down on back side
      handlerMotorDriveForward();
    }
  }

  private void setupShuffleboard() {
    // use Shuffleboard to facilitate controller param tuning
    ShuffleboardTab arm = Shuffleboard.getTab("Arm");

    // arm.add("Arm Control PID", m_controller);

    nt_handlerSpeed = arm.addPersistent("Handler speed", 0.0)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .withPosition(0, 0)
        .getEntry();

    arm.add("Start Handler Forward", new InstantCommand(() -> handlerMotorDriveForward())).withPosition(0, 1);
    arm.add("Start Handler Reverse", new InstantCommand(() -> handlerMotorDriveBackward())).withPosition(1, 1);
    arm.add("Cancel Handler", new InstantCommand(() -> handlerMotorStop())).withPosition(2, 1);

    arm.addDouble("position", () -> m_encoder.getPosition()).withPosition(4, 1)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("max", 420))
        .withSize(2, 2)
        .withPosition(3, 1);
    arm.addDouble("arm motor", () -> m_armMotor.get())
        .withPosition(3, 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(3, 1);

    arm.addString("Current Command", () -> current_command_name());


  }
  public String current_command_name(){
    String command_name = "---";
    if(this.getCurrentCommand() != null){
      command_name = this.getCurrentCommand().getName();
    }
    return command_name;
    }
  public double getHandlerSpeed() {
 
    if (m_handlerSpeed != nt_handlerSpeed.getDouble(ArmConstants.kHandlerDefaultSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_handlerSpeed = nt_handlerSpeed.getDouble(ArmConstants.kHandlerDefaultSpeed);
      Preferences.setDouble(ArmConstants.kArmHandlerSpeedPrefKey, m_handlerSpeed);
    }
    return m_handlerSpeed;
  }
  /**
   * Run the Handler motor in a "forward" direction
   */
  public void handlerMotorDriveForward() {
    // TODO - implement
    m_armHandlerMotor.set(getHandlerSpeed());
  }

  /**
   * Run the Handler motor in a "backward" direction
   */
  public void handlerMotorDriveBackward() {
    // TODO - implement
    m_armHandlerMotor.set(-getHandlerSpeed());
  }

  /**
   * Stop the Handler motor
   */
  public void handlerMotorStop() {
    // TODO - implement
    m_armHandlerMotor.set(0.0);
  }

}
