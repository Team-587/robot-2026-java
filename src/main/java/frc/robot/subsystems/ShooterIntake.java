// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.Configs;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.RotateAutoAlign;

import java.util.ArrayList;

public class ShooterIntake extends SubsystemBase {
  /** Creates a new ShooterIntake. */

  private final SparkFlex m_shootMotor1;
  private final SparkFlex m_shootMotor2;
  private final SparkFlex m_intakeMotor;
  private final SparkMax m_slapMotor;
  private final SparkMax m_kickerMotor;
  private final SparkMax m_hoodMotor;
  private final SparkMax m_slapMotor2;
  private final SparkMax m_indexerMotor;

  private final SparkClosedLoopController m_slapClosedLoopController;
  private final SparkClosedLoopController m_hoodClosedLoopController;
  private final SparkClosedLoopController m_shootController;

  double shootSpeed;
  double intakeSpeed;
  double kickerSpeed;
  double intakeAngle;
  double hoodAngle;
  double indexerSpeed;

  boolean canShoot;
  boolean toggle;

  double t;
  double voy;
  double H;
  double h;
  double g;
  double LLHeight;
  double motorSpeed;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_codriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  private final RotateAutoAlign autoAlign;
  public final Climber climber;
  
  public ShooterIntake() {

      autoAlign = new RotateAutoAlign();
      climber = new Climber();

      m_shootMotor1 = new SparkFlex(ShooterIntakeConstants.kShooter1CanID, MotorType.kBrushless);
      m_shootMotor2 = new SparkFlex(ShooterIntakeConstants.kShooter2CanId, MotorType.kBrushless);
      m_intakeMotor = new SparkFlex(ShooterIntakeConstants.kIntakeCanId, MotorType.kBrushless);
      m_slapMotor = new SparkMax(ShooterIntakeConstants.kSlapDownCanId, MotorType.kBrushless);
      m_kickerMotor = new SparkMax(ShooterIntakeConstants.kKickerCanId, MotorType.kBrushless);
      m_hoodMotor = new SparkMax(ShooterIntakeConstants.kHoodCanID, MotorType.kBrushless);
      m_slapMotor2 = new SparkMax(ShooterIntakeConstants.kSlapDown2CanID, MotorType.kBrushless);
      m_indexerMotor = new SparkMax(ShooterIntakeConstants.kIndexerCanID, MotorType.kBrushless);

      m_slapClosedLoopController = m_slapMotor.getClosedLoopController();
      m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();
      m_shootController = m_shootMotor1.getClosedLoopController();

      shootSpeed = 0.0;
      kickerSpeed = 0.0;
      intakeSpeed = 0.0;
      indexerSpeed = 0.0;
      hoodAngle = 40.0;
      LLHeight = 16.5/12.0;
      H = -6.0; //Change in height
      h = H + 8 - LLHeight; //Max arc height
      g = 32.2; //Gravity
      toggle = false;
      canShoot = false;
      m_shootMotor1.configure(Configs.ShooterIntakeConfig.shooterConfig1, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);
      m_shootMotor2.configure(Configs.ShooterIntakeConfig.shooterConfig2, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);

      m_slapMotor.configure(Configs.ShooterIntakeConfig.slapDownConfig, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);

      m_hoodMotor.configure(Configs.ShooterIntakeConfig.hoodConfig, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);
      m_slapMotor2.configure(Configs.ShooterIntakeConfig.slapDown2Config,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kNoPersistParameters);
      

      m_slapClosedLoopController.setSetpoint(ShooterIntakeConstants.intakeAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      m_shootController.setSetpoint(0, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    double distance = (autoAlign.getDistance()/12);
    SmartDashboard.putNumber("Z Distance", distance);

    voy = Math.sqrt(2*g*(h-H));
    SmartDashboard.putNumber("voy", voy);

    double a = -16.1;
    double b = voy;
    double c = H;
    t = (-b - Math.sqrt(b*b - 4*a*c))/(2*a);
    SmartDashboard.putNumber("t", t);

    double vox = (distance/t);

    double launchAngleActual = Math.toDegrees(Math.atan2((voy),vox));
    double launchAngle = (210 - (launchAngleActual - 59.57)*(292/32));
    SmartDashboard.putNumber("Launch Angle", launchAngle);
    SmartDashboard.putNumber("Actual Launch Angle", launchAngleActual);

    double actualVo = Math.sqrt((vox*vox) + (voy*voy));
    motorSpeed = -(actualVo * 202.8150443);
    SmartDashboard.putNumber("Shoot Speed", motorSpeed);
    SmartDashboard.putNumber("Actual Shoot Speed", actualVo);

      if(/*m_driverController.getRightBumperButton() ||*/ canShoot){
        System.out.println("Auto Align Shooting");
        if(launchAngle > ShooterIntakeConstants.kHoodAngleHigh){
          m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleHigh, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }else if(launchAngle < ShooterIntakeConstants.kHoodAngleLow){
          m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }else{
          m_hoodClosedLoopController.setSetpoint(launchAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
        m_shootController.setSetpoint(motorSpeed, ControlType.kVelocity);
      }else{
        if(hoodAngle < ShooterIntakeConstants.kHoodAngleLow){
          m_hoodClosedLoopController.setSetpoint(ShooterIntakeConstants.kHoodAngleLow, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }else{
          m_hoodClosedLoopController.setSetpoint(hoodAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
        m_shootController.setSetpoint(shootSpeed, ControlType.kVelocity);
        m_kickerMotor.set(-kickerSpeed);
        m_indexerMotor.set(-indexerSpeed);
        m_slapClosedLoopController.setSetpoint(intakeAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
    //m_kickerMotor.set(-kickerSpeed);
    //m_indexerMotor.set(-indexerSpeed);
    if(toggle){
      setIntakeSpeed(0.75);
    }else if(!toggle && intakeSpeed != -0.75){
      setIntakeSpeed(0);
    }
    if(intakeAngle == ShooterIntakeConstants.upAngle){
      //m_kickerMotor.set(0);
      m_indexerMotor.set(0);
      setIntakeSpeed(0);
    }
    SmartDashboard.putBoolean("Intaking", checkIntake());
    SmartDashboard.putBoolean("Kicking", checkKicker());
  }

  public double getCurrentShooterVelocity(){
    return m_shootMotor1.getEncoder().getVelocity();
  }

  public double getSetShooterSpeed(){
    if(motorSpeed != 0){
      return motorSpeed;
    }else{
      return shootSpeed; 
    }
  }

  public boolean checkIntake(){
    if(intakeSpeed > 0.0){
      return true;
    }else{
      return false;
    }
  }

  public boolean checkKicker(){
    if(kickerSpeed > 0.0){
      return true;
    }else{
      return false;
    }
  }

  public void setShooterSpeed(double speed){
      shootSpeed = speed;
  }

  public void setKickerSpeed(double speed, double indexSpeed){
    kickerSpeed = speed;
    indexerSpeed = -indexSpeed;
  }

  public void setIntakeAngle(double angle){
    intakeAngle = angle;
  }

  public void toggle(){
    toggle = !toggle;
  }

  public void setHoodAngle(double angle){
    hoodAngle = angle;
  }

  public void changeHoodAngle(double change) {
    if (hoodAngle + change < ShooterIntakeConstants.kHoodAngleHigh && hoodAngle + change > ShooterIntakeConstants.kHoodAngleLow) {
      hoodAngle += change;
    }
    setHoodAngle(hoodAngle);
  }

  public void CanShoot(boolean canShoot){
    this.canShoot = canShoot;
    System.out.println("Setting Can Shoot");
  }

  public void setIntakeSpeed(double speed){
    System.out.println(speed);
    intakeSpeed = speed;
    if(speed == -0.75){
      toggle = false;
    }
    m_intakeMotor.set(speed);
  }

  public void feed(double speed, double angle){
    shootSpeed = speed;
    hoodAngle = angle;
  }
}
