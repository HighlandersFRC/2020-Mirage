/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private double shooterPower;
  private double kf = 0.00015;
  private double kp = 0.00000000;
  private double ki = 0.000001;
  private double kd = 0.002;
  private CANPIDController vpidController = new CANPIDController(RobotMap.shooterMotorOne);

  public Shooter() {


  }
  public void initShooterPID(){
    shooterPower = 0;
    vpidController.setFF(kf);
    vpidController.setP(kp);
    vpidController.setI(ki);
    vpidController.setD(kd);
    vpidController.setOutputRange(0, 1); 

  }
  public double calculateNeededRPM(double dist){
    return 0.2089*Math.pow(dist, 4)-11.907*Math.pow(dist, 3)+246.94*Math.pow(dist,2)-2122.3*dist+10774;
  }

  @Override
  public void periodic() {
    if(!RobotState.isDisabled()&&RobotState.isOperatorControl()){ 
      if(ButtonMap.trackTarget()){
          shooterPower = calculateNeededRPM(RobotMap.lidarLite.getDistance());
      } 
      else{
        shooterPower = 0;
      }
      if(shooterPower >6000){
        shooterPower = 6000;
      }
      if(shooterPower <0){
        shooterPower = 0;
      }
      if(Math.abs(shooterPower-RobotMap.shooterMotorOne.getEncoder().getVelocity())<20){
        SmartDashboard.putBoolean("ready", true);
      }
      else{
        SmartDashboard.putBoolean("ready", false);
      }
      SmartDashboard.putNumber("shooterPower", shooterPower);
      SmartDashboard.putNumber("actualVel", RobotMap.shooterMotorOne.getEncoder().getVelocity());
      if(shooterPower!=0){
        vpidController.setReference(shooterPower, ControlType.kVelocity);
      }
      else{
        RobotMap.shooterMotorOne.set(0);
      }
    }

    // This method will be called once per scheduler run
  }
}
