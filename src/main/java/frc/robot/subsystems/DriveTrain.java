/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonMap;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.DriveEncoder;
import frc.robot.tools.controlLoops.PID;
import frc.robot.tools.pathTools.Odometry;

public class DriveTrain extends SubsystemBase {

	private double deadZone = 0.0102;
	private double turn =0;
	private double throttel = 0;
	private double ratio;
	private DriveEncoder leftMainDrive = new DriveEncoder(RobotMap.leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
	private DriveEncoder rightMainDrive = new DriveEncoder(RobotMap.rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));
	private double speed;
	private final double vKF = 0.153;
	private final double vKP = 0.35025;
	private final double vKI = 0.00000000;
	private final double vKD = 0;
	private PID alignmentPID;
	private final double aKP = 0.029;
	private final double aKI = 0.00099;
	private final double aKD = 0;//0.02;
	private double visionOffset = 0;
	private final double visionDeadzone = 0.75;
	private final double visionAcceptablilityZone = 1.5;
	private int profile = 0;
	private Odometry autoOdometry;
  	public DriveTrain() {

  	}
	public void startAutoOdometry(double x, double y, double theta, boolean shouldReverse){
		autoOdometry = new Odometry(shouldReverse);
		autoOdometry.start();
	};
	public double getDriveTrainX(){
		return autoOdometry.getX();
	}
	public double getDriveTrainY(){
		return autoOdometry.getY();
	}
	public double getDriveTrainHeading(){
		return autoOdometry.gettheta();
	}

	public void setDriveTrainX(double x){
		 autoOdometry.setX(x);
	}
	public void setDriveTrainY(double y){
		 autoOdometry.setY(y);
	}
	public void setDriveTrainHeading(double theta){
		 autoOdometry.setTheta(theta);
	}
	public void setOdometryReversed(boolean reversed){
		 autoOdometry.setReversed(reversed);
	}

	public void initVelocityPIDs(){
		RobotMap.leftDriveLead.selectProfileSlot(profile, 0);
		RobotMap.leftDriveLead.config_kF(profile, vKF, 0);
		RobotMap.leftDriveLead.config_kP(profile, vKP, 0);
		RobotMap.leftDriveLead.config_kI(profile, vKI, 0);
		RobotMap.leftDriveLead.config_kD(profile, vKD, 0);
		RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
		RobotMap.rightDriveLead.selectProfileSlot(profile, 0);
		RobotMap.rightDriveLead.config_kF(profile, vKF, 0);
		RobotMap.rightDriveLead.config_kP(profile, vKP, 0);
		RobotMap.rightDriveLead.config_kI(profile, vKI, 0);
		RobotMap.rightDriveLead.config_kD(profile, vKD, 0);
		RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));
	}
	public void initAlignmentPID(){
		alignmentPID = new PID(aKP, aKI, aKD);
		alignmentPID.setMaxOutput(2);
		alignmentPID.setMinOutput(-2);
		alignmentPID.setSetPoint(visionOffset);
	}
	public boolean trackVisionTape(){
		Robot.visionCamera.updateVision();
		if(Timer.getFPGATimestamp()-Robot.visionCamera.lastParseTime>0.25||Math.abs(Robot.visionCamera.getAngle()-visionOffset)<visionDeadzone){
			alignmentPID.updatePID(visionOffset);
			setLeftSpeed(0);
			setRightSpeed(0);
		}
		else{
			alignmentPID.updatePID(Robot.visionCamera.getAngle());
		}
			
		setLeftSpeed(alignmentPID.getResult());
		setRightSpeed(-alignmentPID.getResult());
		if(Math.abs(Robot.visionCamera.getAngle()-visionOffset)<visionAcceptablilityZone){
			return true;
		}
		else{
			return false;
		}
		
	}
	public void shiftVisionLeft(){
		visionOffset = visionOffset +0.5;
		alignmentPID.setSetPoint(visionOffset);
	}
	public void shiftVisionRight(){
		visionOffset = visionOffset-0.5;
		alignmentPID.setSetPoint(visionOffset);
	}
	public void arcadeDrive(){
		double leftPower;
		double rightPower;
		double differential;
		if(Math.abs(ButtonMap.getDriveThrottle())>deadZone){
			throttel = Math.tanh(ButtonMap.getDriveThrottle())*(4/3.215926); 
		}
		else{
			throttel = 0;
		}

		ratio = Math.abs(throttel);
		if(Math.abs(ButtonMap.getRotation())>deadZone){
			turn = ButtonMap.getRotation();
		}
		else{
			turn = 0;
		}
		turn = ButtonMap.getRotation();
		differential = turn;
		SmartDashboard.putNumber("differential", differential);
		leftPower = (throttel - (differential));
		rightPower = (throttel + (differential));
	
		if(Math.abs(leftPower)>1) {
			rightPower = Math.abs(rightPower/leftPower)*Math.signum(rightPower);
			leftPower = Math.signum(leftPower);
		}
		else if(Math.abs(rightPower)>1) {
			leftPower = Math.abs(leftPower/rightPower)*Math.signum(leftPower);
			rightPower = Math.signum(rightPower);
		}
		setLeftPercent(leftPower);
		setRightPercent(rightPower);
	}
	public void Stop(){
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);

	}
	public void setLeftSpeed(double speed){
		SmartDashboard.putNumber("LeftSpeed", leftMainDrive.getVelocity());
		SmartDashboard.putNumber("RightSpeed", rightMainDrive.getVelocity());
		RobotMap.leftDriveLead.set(ControlMode.Velocity, leftMainDrive.convertftpersToNativeUnitsper100ms(speed));
	}
	public void setRightSpeed(double speed){
		RobotMap.rightDriveLead.set(ControlMode.Velocity, rightMainDrive.convertftpersToNativeUnitsper100ms(speed));

	}
	public void setLeftPercent(double percent){
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, percent);
	}
	public void setRightPercent(double percent){
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, percent);
	}
	public void stopDriveTrainMotors(){
		for(TalonSRX talon : RobotMap.driveMotorLeads){
			talon.set(ControlMode.PercentOutput, 0);
		}
	}
	
  @Override
  public void periodic() {
    if(RobotState.isOperatorControl()&&!RobotState.isDisabled()){
		if(ButtonMap.trackTarget()){
			RobotMap.visionRelay1.set(Value.kForward);
			try {
				SmartDashboard.putBoolean("targetLock", trackVisionTape());
			} catch (Exception e) {
			}
			if(ButtonMap.adjustTargetTrackingLeft()){
				shiftVisionLeft();
			}
			else if(ButtonMap.adjustTargetTrackingRight()){
				shiftVisionRight();
			}
		}
		else{
			RobotMap.visionRelay1.set(Value.kReverse);
			SmartDashboard.putBoolean("targetLock", false);
			arcadeDrive();
		}


	}
	
  }
}
