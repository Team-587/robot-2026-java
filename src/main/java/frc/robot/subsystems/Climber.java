// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final SparkMax m_climberMotor;
  boolean climbMode;
  XboxController m_codriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  public Climber() {
    climbMode = false;
    m_climberMotor = new SparkMax(ClimberConstants.kClimberCanID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Dashboard.putBoolean("Climb Mode", climbMode);
    if(climbMode == true){
      m_climberMotor.set(m_codriverController.getLeftY());
    }else{
      m_climberMotor.set(0);
    }
  }

  public boolean inClimbMode(){
    return climbMode;
  }

  public void setClimbMode(){
    climbMode = !climbMode;
  }
}
