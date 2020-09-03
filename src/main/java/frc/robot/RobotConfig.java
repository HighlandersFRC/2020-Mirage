/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class RobotConfig {
    public void setStartingConfig(){
        for(TalonSRX talon:RobotMap.allMotors){
            talon.configVoltageCompSaturation(12.1);
            talon.enableVoltageCompensation(true);
        }
        RobotConfig.setAllMotorsBrake();
        RobotMap.rightDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		RobotMap.leftDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        
        RobotMap.rightDriveFollowerOne.set(ControlMode.Follower, RobotMap.rightMasterTalonID);
        RobotMap.rightDriveFollowerTwo.set(ControlMode.Follower, RobotMap.rightMasterTalonID);

        RobotMap.leftDriveFollowerOne.set(ControlMode.Follower, RobotMap.leftMasterTalonID);
        RobotMap.leftDriveFollowerTwo.set(ControlMode.Follower, RobotMap.leftMasterTalonID);
        
        RobotMap.rightDriveLead.setInverted(false);
        RobotMap.rightDriveFollowerOne.setInverted(InvertType.FollowMaster);
        RobotMap.rightDriveFollowerTwo.setInverted(InvertType.FollowMaster);

    	RobotMap.leftDriveLead.setInverted(true);
        RobotMap.leftDriveFollowerOne.setInverted(InvertType.FollowMaster);
        RobotMap.leftDriveFollowerTwo.setInverted(InvertType.FollowMaster);

        RobotMap.leftDriveLead.setSensorPhase(true);
        RobotMap.rightDriveLead.setSensorPhase(true);
    	RobotMap.leftDriveLead.setSelectedSensorPosition(0, 0,0);
        RobotMap.rightDriveLead.setSelectedSensorPosition(0, 0, 0);

        RobotMap.leftDriveLead.configVoltageCompSaturation(11.5);
        RobotMap.leftDriveFollowerOne.configVoltageCompSaturation(11.5);
        RobotMap.leftDriveFollowerTwo.configVoltageCompSaturation(11.5);
        
        RobotMap.rightDriveLead.configVoltageCompSaturation(11.5);
        RobotMap.rightDriveFollowerOne.configVoltageCompSaturation(11.5);
        RobotMap.rightDriveFollowerTwo.configVoltageCompSaturation(11.5);

        RobotMap.shooterMotorOne.setIdleMode(IdleMode.kCoast);
        RobotMap.shooterMotorOne.setInverted(true);
        RobotMap.shooterMotorTwo.setIdleMode(IdleMode.kCoast);
        RobotMap.shooterMotorTwo.follow(RobotMap.shooterMotorOne, true);


    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotStats.driveMotorContinuousCurrentHighGear);
    		talon.configPeakCurrentLimit(RobotStats.driveMotorPeakCurrentHighGear);
            talon.configPeakCurrentDuration(RobotStats.driveMotorPeakCurrentDurationHighGear);
            talon.enableCurrentLimit(true);
        }

    }
    public void setTeleopConfig(){
        RobotConfig.setDriveMotorsCoast();
    }
    public void setAutoConfig(){
        RobotConfig.setDriveMotorsBrake();
    }
    public static void setAllMotorsBrake() {
		for(TalonSRX talon:RobotMap.allMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
	public static void setDriveMotorsCoast() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Coast);
        }
	}

	public static void setDriveMotorsBrake() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
}
