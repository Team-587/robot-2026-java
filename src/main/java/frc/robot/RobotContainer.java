// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ShooterIntake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase{

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private Lights m_lights = new Lights();

  PathPlannerAuto Mario;
  PathPlannerAuto Pacman;
  PathPlannerAuto Pichu;
  PathPlannerAuto DK;
  PathPlannerAuto Yoshi;
  PathPlannerAuto NewMario;
  PathPlannerAuto newAuto;
  PathPlannerAuto simpleAuto;

  SendableChooser<Command> m_Chooser;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_codriverController =
      new CommandXboxController(OIConstants.kCoDriverControllerPort);

  public Command zeroHeading() {
    return this.runOnce(() -> m_drive.zeroHeading());
  }

  public Command shooterOff(){
    return this.runOnce(() -> m_lights.shooter.setShooterSpeed(0));
  }

  public Command shortShoot(){
    return this.runOnce(() -> m_lights.shooter.setShooterSpeed(ShooterIntakeConstants.kShootCloseSpeed));
  }


  public Command farShoot(){
    return this.runOnce(() -> m_lights.shooter.setShooterSpeed(ShooterIntakeConstants.kShootFarSpeed));
  }

  public Command kickerOn(){
    return this.runOnce(() -> m_lights.shooter.setKickerSpeed(ShooterIntakeConstants.kKickerOnSpeed, ShooterIntakeConstants.kIndexerOnSpeed));
  }

  public Command kickon(){
    return Commands.sequence(Commands.waitSeconds(1), kickerOn());
  }

  public Command kickerOff(){
    return this.runOnce(() -> m_lights.shooter.setKickerSpeed(ShooterIntakeConstants.kKickerOffSpeed, ShooterIntakeConstants.kIndexerOffSpeed));
  }

  public Command intakeUp(){
    return this.runOnce(() -> m_lights.shooter.setIntakeAngle(ShooterIntakeConstants.upAngle));
  }

  public Command intakeDown(){
    return this.runOnce(() -> m_lights.shooter.setIntakeAngle(ShooterIntakeConstants.intakeAngle));
  }

  public Command intakeShoot(){
    return this.runOnce(() -> m_lights.shooter.setIntakeAngle(ShooterIntakeConstants.shootIntakeAngle));
  }

  public Command upAndDown(){
    return Commands.sequence(intakeShoot(), Commands.waitSeconds(0.2), intakeDown(), Commands.waitSeconds(0.5)).repeatedly();
  }

  public Command shootSetup(){
    return Commands.sequence(Commands.waitSeconds(1.5), kickerOn(), Commands.waitSeconds(1), upAndDown());
  }

  public Command shootAuto(){
    return Commands.sequence(farShoot(), farShotAngle(), Commands.waitSeconds(1.5), 
                              kickerOn(), Commands.waitSeconds(6), 
                              shooterOff(), kickerOff(), intakeDown());
  }

  public Command canShoot(){
    return this.runOnce(() -> m_lights.shooter.CanShoot(true));
  }

  public Command canLineUp(){
    return this.runOnce(() -> m_drive.CanLineUp(true));
  }
  
  public Command cantShoot(){
    return this.runOnce(() -> m_lights.shooter.CanShoot(false));
  }

  public Command cantLineUp(){
    return this.runOnce(() -> m_drive.CanLineUp(false));
  }

  public Command runIntake(){
    return this.runOnce(() ->m_lights.shooter.setIntakeSpeed(0.75));
  }

  public Command stopIntake(){
    return this.runOnce(() ->m_lights.shooter.setIntakeSpeed(0));
  }

  public Command reverseIntake(){
    return this.runOnce(() ->m_lights.shooter.setIntakeSpeed(-0.75));
  }

  public Command shortShotAngle(){
    return this.runOnce(() ->m_lights.shooter.setHoodAngle(ShooterIntakeConstants.kShootCloseAngle));
  }

  public Command farShotAngle(){
    return this.runOnce(() ->m_lights.shooter.setHoodAngle(ShooterIntakeConstants.kShootFarAngle));
  }

  public Command toggle(){
    return this.runOnce(() ->m_lights.shooter.toggle());
  }

  public Command stopShoot(){
    return this.runOnce(() -> m_lights.shooter.setShooterSpeed(0));
  }


  public RobotContainer() {
    NamedCommands.registerCommand("Short Shoot", shortShoot());
    NamedCommands.registerCommand("Short Angle", shortShotAngle());

    NamedCommands.registerCommand("Can Shoot", canShoot());
    NamedCommands.registerCommand("Can Line Up", canLineUp());
    NamedCommands.registerCommand("Can't Shoot", cantShoot());
    NamedCommands.registerCommand("Can't Line Up", cantLineUp());

    NamedCommands.registerCommand("Kicker On", kickerOn());
    NamedCommands.registerCommand("Kicker Off", kickerOff());

    NamedCommands.registerCommand("Intake Down", intakeDown());
    NamedCommands.registerCommand("Run Intake", runIntake());
    NamedCommands.registerCommand("Stop Intake", stopIntake());
    NamedCommands.registerCommand("Intake Up", intakeUp());

    NamedCommands.registerCommand("Intake Middle", intakeShoot());
    NamedCommands.registerCommand("Toggle", toggle());

    NamedCommands.registerCommand("Far Shoot", farShoot());
    NamedCommands.registerCommand("Far Angle", farShotAngle());
    NamedCommands.registerCommand("Shoot Auto", shootAuto());
    NamedCommands.registerCommand("Stop Shooting", stopShoot());


    m_Chooser = new SendableChooser<Command>();
    configureBindings();

    m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive));

    String Mario_Str = "Mario";
    Mario = new PathPlannerAuto(Mario_Str);
    //m_Chooser.addOption(Mario_Str, Mario);

    String NewMario_Str = "Mario New";
    NewMario = new PathPlannerAuto(NewMario_Str);
    //m_Chooser.addOption(NewMario_Str, NewMario);
    

    String Pacman_Str = "Pacman";
    Pacman = new PathPlannerAuto(Pacman_Str);
    //m_Chooser.addOption(Pacman_Str, Pacman);

    String Pichu_Str = "Pichu";
    Pichu = new PathPlannerAuto(Pichu_Str);
    //m_Chooser.addOption(Pichu_Str, Pichu);

    String DK_Str = "Donkey Kong";
    DK = new PathPlannerAuto(DK_Str);
    //m_Chooser.addOption(DK_Str, DK);

    String Yoshi_Str = "Yoshi";
    Yoshi = new PathPlannerAuto(Yoshi_Str);
    //m_Chooser.addOption(Yoshi_Str, Yoshi);

    String newAuto_Str = "Auto";
    newAuto = new PathPlannerAuto(newAuto_Str);
    m_Chooser.addOption(newAuto_Str, newAuto);

    String simpleAuto_Str = "Simple Auto";
    simpleAuto = new PathPlannerAuto(simpleAuto_Str);
    m_Chooser.addOption(simpleAuto_Str, simpleAuto);

    SmartDashboard.putData("Auto", m_Chooser);
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
    m_driverController.start().onTrue(zeroHeading());

    //m_codriverController.leftBumper().onTrue(intakeUp()).onTrue(stopIntake()).onTrue(kickerOff());
    m_driverController.x().onTrue(toggle());
    m_driverController.y().onTrue(intakeUp()).onTrue(stopIntake()).onTrue(kickerOff());
    m_driverController.b().whileTrue(intakeShoot());
    m_driverController.a().onTrue(intakeDown());

    //m_codriverController.y().onTrue(kickerOn());
    //m_codriverController.a().onTrue(kickerOff());

    //m_codriverController.povUp().onTrue(cantShoot()).onTrue(cantLineUp()).onTrue(shooterOff()).onTrue(kickerOff());
    m_driverController.povUp().onTrue(cantShoot()).onTrue(cantLineUp()).onTrue(shooterOff()).onTrue(kickerOff()).onTrue(intakeDown());
    m_driverController.povDown().whileTrue(reverseIntake()).onFalse(stopIntake());

    m_driverController.leftBumper().whileTrue(shortShoot()).onTrue(shortShotAngle()).whileTrue(shootSetup()).onFalse(shooterOff()).onFalse(kickerOff()).onFalse(intakeDown()).onFalse(cantLineUp());
    m_driverController.leftTrigger().whileTrue(farShoot()).onTrue(farShotAngle()).whileTrue(shootSetup()).onFalse(shooterOff()).onFalse(kickerOff()).onFalse(intakeDown()).onFalse(cantLineUp());
  }


  public Command getAutonomousCommand() {
    //return new InstantCommand();
    return m_Chooser.getSelected();
    //return new PathPlannerAuto("Mario");
  }
}
