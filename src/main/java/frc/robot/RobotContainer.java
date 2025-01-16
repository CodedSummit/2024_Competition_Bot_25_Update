// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.NothingCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AddressableLedSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionPoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AddressableLedSubsystem m_led = new AddressableLedSubsystem(30,9);
  private VisionPoseEstimationSubsystem m_visionPoseEstimationSubsystem = new VisionPoseEstimationSubsystem(m_led);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_visionPoseEstimationSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_led);
  private final NoteShooterSubsystem m_shooterSubsystem = new NoteShooterSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  //private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

  private SwerveJoystickCmd swerveJoystickCmd;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_drivPs5Controller = 
    new CommandPS5Controller(0);
//  private final Joystick m_buttonBoard = new Joystick(1);
//  private final Trigger m_button1 = new Trigger(() ->m_buttonBoard.getRawButton(1));
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    UsbCamera riocam_intake = CameraServer.startAutomaticCapture();
    riocam_intake.setFPS(5);
    riocam_intake.setResolution(160, 120);

    UsbCamera riocam_shooter = CameraServer.startAutomaticCapture();
    riocam_shooter.setFPS(5);
    riocam_shooter.setResolution(160, 120);
    
    // Configure the trigger bindings

    swerveJoystickCmd = new SwerveJoystickCmd(
      swerveSubsystem,
      m_drivPs5Controller);
    swerveSubsystem.setDefaultCommand(swerveJoystickCmd); 

    //Named commands for Autos
    NamedCommands.registerCommand("Intake", new IntakeNoteCommand(m_intakeSubsystem));
    NamedCommands.registerCommand("Shoot", ShootCommand());


    

    // make the chasetag command

    Command placeholderChaser = new ChaseTagCommand(m_visionSubsystem, swerveSubsystem, m_led);
    
    configureBindings();

    //   for debugging only - make a default chase command
   // CommandScheduler.getInstance().setDefaultCommand(m_visionSubsystem, placeholderChaser);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    setupShuffleboard();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("poseestimator", m_visionPoseEstimationSubsystem);

  }

  private void setupShuffleboard() {
    
    ShuffleboardTab sys = Shuffleboard.getTab("Systems");

    sys.add(m_shooterSubsystem);
    sys.add(m_intakeSubsystem);
    sys.add(m_armSubsystem);

    sys.add(ShootCommand());

    //sys.add(pdp);
    

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    m_drivPs5Controller.triangle().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    m_drivPs5Controller.cross().toggleOnTrue(new IntakeNoteCommand(m_intakeSubsystem));



    m_drivPs5Controller.button(9)
      .onTrue(new InstantCommand(() ->m_intakeSubsystem.reverseIntake()))
      .onFalse(new InstantCommand(() -> m_intakeSubsystem.stop()));



    Command shootCommand = ShootCommand();
    Command shootMaxCommand = ShootMaxCommand();
    m_drivPs5Controller.circle().and(m_drivPs5Controller.R2().negate()).toggleOnTrue(shootCommand);
    m_drivPs5Controller.circle().and(m_drivPs5Controller.R2()).toggleOnTrue(shootMaxCommand);
    
    
    m_drivPs5Controller.square()
      .onTrue(new InstantCommand(() ->m_intakeSubsystem.reverseIntake()))
      .onFalse(new InstantCommand(() -> m_intakeSubsystem.stop()));  
    
    //  .onTrue(new InstantCommand(() ->m_ClimbSubsystem.climb()))
    //  .onFalse(new InstantCommand(() -> m_ClimbSubsystem.stop()));


    //Command navToA = makeNavCommand(new Pose2d(1.81, 7.68, new Rotation2d(0)));
    //m_driverController.a().whileTrue(navToA);

    //m_driverController.x().whileTrue(new ChaseTagCommand(m_visionSubsystem, swerveSubsystem, m_led));

    // Left Bumper controls field orientation for drive mode. Upressed (default) is field oriented
    //     Pressed is robot oriented
    m_drivPs5Controller.button(5)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(false)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(true)));

      //reverse robot orientation mode
      m_drivPs5Controller.axisGreaterThan(3, 0.5).and(m_drivPs5Controller.button(5).negate())
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(true)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(false)));

    /*
    note: there is an odd state you can get into with the dampen and boost features enabled below. As the press and
          release are different operations, you can cause odd behaviors. Consider the following sequence:
        Press bumper(set dampened)
        Press Trigger(set Turbo)
        Release Trigger (Set Normal)
        now, the speed is normal, in spite of still holding the bumper in.
        this can be resolved by considering the state of both buttons when choosing the speed factor.

        Fix below may work (also applied above) that allows consideration of other button states when applying the commands.
     */

    m_drivPs5Controller.button(6)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getDampenedSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    //includes logic to not activate if dampen is pressed.
      m_drivPs5Controller.axisGreaterThan(4, 0.5).and(m_drivPs5Controller.button(6).negate())
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getTurboSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    m_drivPs5Controller.povDown()
      .onTrue(new InstantCommand(() ->m_armSubsystem.manualArmDown()))
      .onFalse(new InstantCommand(() -> m_armSubsystem.manualArmStop()));
    m_drivPs5Controller.button(11)
      .onTrue(new InstantCommand(() ->m_armSubsystem.manualArmDown()))
      .onFalse(new InstantCommand(() -> m_armSubsystem.manualArmStop()));

    m_drivPs5Controller.povUp()
      .onTrue(new InstantCommand(() ->m_armSubsystem.manualArmUp()))
      .onFalse(new InstantCommand(() -> m_armSubsystem.manualArmStop()));
    m_drivPs5Controller.button(12)
      .onTrue(new InstantCommand(() ->m_armSubsystem.manualArmUp()))
      .onFalse(new InstantCommand(() -> m_armSubsystem.manualArmStop()));
    
    m_drivPs5Controller.povRight()
      .toggleOnTrue(HandoffToArm());


    m_drivPs5Controller.povLeft()
      .onTrue(ArmPiecePlace());

    m_drivPs5Controller.button(14)
      .onTrue(ArmPiecePlace());
//11 for Up
//12 dfor down
//14 for drop

    //m_driverController.povDown().onTrue(new InstantCommand(() ->m_led.setStripBlue()));
    //m_driverController.povUp().onTrue(new InstantCommand(() ->m_led.setStripPurple()));


    // temporarily do while true so releasing button stops the path
    //m_driverController.povLeft().whileTrue(swerveSubsystem.followPathCommand("ShortRun"));

//    m_button1.onTrue(ShootCommand());
  }

    public void runStartupCalibration(){
    /*if(!armSubsystem.isCalibrated()){
      new CalibrateArmCommand(armSubsystem).schedule();
    }*/
  }

  /**
   * @param targetPose
   * @return
   */
  public Command makeNavCommand(Pose2d targetPose){

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            0.5, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );

    return pathfindingCommand;
  }

  public Command makePathCommand(String pathName) {
    PathPlannerPath path = null;
    // Load the path you want to follow using its name in the GUI
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner file:" + pathName, e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  };

  public void loadPreferences(){
    swerveSubsystem.loadPreferences();
  }

    public Command ShootMaxCommand(){
    Command x = Commands.sequence(
      new InstantCommand(() -> m_shooterSubsystem.spinUpMax()),
      new WaitCommand(1),
      new InstantCommand(() -> m_intakeSubsystem.feedShooter()),
      new WaitUntilCommand(() -> !m_intakeSubsystem.hasNote()),
      //new WaitCommand(2),
      new InstantCommand(() -> m_intakeSubsystem.stop()),
      new WaitCommand(.5),
      new InstantCommand(() -> m_shooterSubsystem.stop())
    );
    x.setName("Shoot");
    x.addRequirements(m_shooterSubsystem, m_intakeSubsystem);
    return x;
  }

  public Command ShootCommand(){
    Command x = Commands.sequence(
      new InstantCommand(() -> m_shooterSubsystem.spinUp()),
      new WaitCommand(1),
      new InstantCommand(() -> m_intakeSubsystem.feedShooter()),
      new WaitUntilCommand(() -> !m_intakeSubsystem.hasNote()),
      //new WaitCommand(2),
      new InstantCommand(() -> m_intakeSubsystem.stop()),
      new WaitCommand(.5),
      new InstantCommand(() -> m_shooterSubsystem.stop())
    );
    x.setName("Shoot");
    x.addRequirements(m_shooterSubsystem, m_intakeSubsystem);
    return x;
  }

  public Command HandoffToArm(){
    Command handoff = Commands.sequence(
      new ConditionalCommand(
        new NothingCommand(), //true
        new NothingCommand(), //false //this will do nothing till the arm is down
        () -> m_armSubsystem.manualAtHome()), //condition
      new WaitUntilCommand(()-> m_armSubsystem.manualAtHome()),
      new InstantCommand(()->m_armSubsystem.manualArmStop()),
      new PrintCommand("Arm At Home"),
      new InstantCommand(()->m_armSubsystem.handlerMotorDriveForward()),
      new InstantCommand(()->m_intakeSubsystem.feedArm()),
      new WaitCommand(1),
      new InstantCommand(()-> m_intakeSubsystem.stop()),
      new InstantCommand(() -> m_armSubsystem.handlerMotorStop(), m_armSubsystem)
    );
    handoff.setName("Handoff");
    handoff.addRequirements(m_armSubsystem, m_intakeSubsystem);
    return handoff;
  }

    public Command HandoffToArmAutoPosition(){
    Command handoff = Commands.sequence(
      new ConditionalCommand(
        new NothingCommand(), //true
        new InstantCommand(()->m_armSubsystem.manualArmDown()), //false
        () -> m_armSubsystem.manualAtHome()), //condition
      new WaitUntilCommand(()-> m_armSubsystem.manualAtHome()),
      new InstantCommand(()->m_armSubsystem.manualArmStop()),
      new PrintCommand("Arm At Home"),
      new InstantCommand(()->m_armSubsystem.handlerMotorDriveForward()),
      new InstantCommand(()->m_intakeSubsystem.feedArm()),
      new WaitCommand(1),
      new InstantCommand(()-> m_intakeSubsystem.stop()),
      new InstantCommand(() -> m_armSubsystem.handlerMotorStop(), m_armSubsystem)
    );
    handoff.setName("Handoff Auto Position");
    handoff.addRequirements(m_armSubsystem, m_intakeSubsystem);
    return handoff;
  }

  public Command ArmPiecePlace(){
    Command place = Commands.sequence(
      new InstantCommand(()->m_armSubsystem.manualDropPiece()),
      new WaitCommand(1),
      new InstantCommand(()->m_armSubsystem.handlerMotorStop())
    );
    place.addRequirements(m_armSubsystem);
    return place;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    //return new NothingCommand();
  }
}
