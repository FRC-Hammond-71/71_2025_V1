// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.MK4iSwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */


 //note: 0.343 meters
public class Robot extends TimedRobot {
		private Command m_autonomousCommand;
		//"Formatting"

		private RobotContainer m_robotContainer;

		private XboxController m_controller = new XboxController(0);

		private MK4iSwerveModule m_FrontLeftModule = new MK4iSwerveModule(10, 11, 0, 0);
		private MK4iSwerveModule m_FrontRightModule = new MK4iSwerveModule(12, 13, 1, 0);
		private MK4iSwerveModule m_BackLeftModule = new MK4iSwerveModule(14, 15, 2, 0);
		private MK4iSwerveModule m_BackRightModule = new MK4iSwerveModule(16, 17, 3, 0);

		private static final double kMaxMovingMagnitudeInMetersPerSecond = 4;

		/**		
		 * This function is run when the robot is first started up and should be used
		 * for any
		 * initialization code.
		 */
		@Override
		public void robotInit() {
				// Instantiate our RobotContainer. This will perform all our button bindings,
				// and put our
				// autonomous chooser on the dashboard.
				m_robotContainer = new RobotContainer();
		}

		/**
		 * This function is called every 20 ms, no matter the mode. Use this for items
		 * like diagnostics
		 * that you want ran during disabled, autonomous, teleoperated and test.
		 *
		 * <p>
		 * This runs after the mode specific periodic functions, but before LiveWindow
		 * and
		 * SmartDashboard integrated updating.
		 */
		@Override
		public void robotPeriodic() {
				// Runs the Scheduler. This is responsible for polling buttons, adding
				// newly-scheduled
				// commands, running already-scheduled commands, removing finished or
				// interrupted commands,
				// and running subsystem periodic() methods. This must be called from the
				// robot's periodic
				// block in order for anything in the Command-based framework to work.
				CommandScheduler.getInstance().run();
		}

		/** This function is called once each time the robot enters Disabled mode. */
		@Override
		public void disabledInit() {
				this.m_FrontLeftModule.Stop();
		}

		@Override
		public void disabledPeriodic() {
		}

		/**
		 * This autonomous runs the autonomous command selected by your
		 * {@link RobotContainer} class.
		 */
		@Override
		public void autonomousInit() {
				m_autonomousCommand = m_robotContainer.getAutonomousCommand();

				// schedule the autonomous command (example)
				if (m_autonomousCommand != null) {
						m_autonomousCommand.schedule();
				}
		}

		/** This function is called periodically during autonomous. */
		@Override
		public void autonomousPeriodic() {
		}

		@Override
		public void teleopInit() {
				// This makes sure that the autonomous stops running when
				// teleop starts running. If you want the autonomous to
				// continue until interrupted by another command, remove
				// this line or comment it out.
				if (m_autonomousCommand != null) {
						m_autonomousCommand.cancel();
				}
		}

		/** This function is called periodically during operator control. */
		@Override
		public void teleopPeriodic() {

				// Rotation2d movingDirection = Rotation2d.fromRadians(Math.atan2(this.m_controller.getLeftY(), this.m_controller.getLeftX()));

				// double movingMagnitude = (Math.sqrt(Math.pow(this.m_controller.getLeftX(), 2) + Math.pow(this.m_controller.getLeftY(), 2)) / 1.41) * kMaxMovingMagnitudeInMetersPerSecond;
				
				// System.out.println(movingMagnitude);

				// if (movingMagnitude < 0.15)
				// {
				//     // this.m_FrontLeftModule.
				//     this.m_FrontLeftModule.Drive(new SwerveModuleState(0, this.m_FrontLeftModule.getAzimuthRotation()));
				// }

				// // Optional: smooths magnitude but check desired scaling effect
				// movingMagnitude = Math.pow(movingMagnitude, 3); 
				
				// movingMagnitude = Math.min(movingMagnitude, kMaxMovingMagnitudeInMetersPerSecond);

				//"Formatting"

				if (this.m_controller.getYButton())
				{
					this.Drive(new ChassisSpeeds(1,0,Math.PI/4));
				}
				else if (this.m_controller.getAButton())
				{
					this.Drive(new ChassisSpeeds(-1,0,-Math.PI/4));
				}
				else if (this.m_controller.getXButton())
				{
					this.Drive(new ChassisSpeeds(0,1, 0));
				}
				else if (this.m_controller.getBButton())
				{
					this.Drive(new ChassisSpeeds(0,-1,0));
				}
				else
				{
					this.Stop();
				}				
		}

		public void Stop() //	"Formatting"
		{
			this.m_FrontLeftModule.Stop();
			this.m_FrontRightModule.Stop();
			this.m_BackLeftModule.Stop();
			this.m_BackRightModule.Stop();
		}

		public void Drive(ChassisSpeeds Speeds) { //"formatting"
				// Locations for the swerve drive modules relative to the robot center. | "formatting"
			Translation2d m_frontLeftLocation = new Translation2d(0.343, 0.343);
			Translation2d m_frontRightLocation = new Translation2d(0.343, -0.343);
			Translation2d m_backLeftLocation = new Translation2d(-0.343, 0.343);
			Translation2d m_backRightLocation = new Translation2d(-0.343, -0.343);

			

			// Creating my kinematics object using the module locations
			SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation); //"even more formatting"
			SwerveModuleState[] swerveStateArray = m_kinematics.toSwerveModuleStates(Speeds);
			this.m_FrontLeftModule.Drive(swerveStateArray[0]);
			this.m_FrontRightModule.Drive(swerveStateArray[1]);
			this.m_BackLeftModule.Drive(swerveStateArray[2]);
			this.m_BackRightModule.Drive(swerveStateArray[3]);

			//"Formatting" and "Lists"
			//FL = 0
			//FR = 1
			//BL = 2
			//BR = 3

			}
	

		@Override
		public void testInit() {
				// Cancels all running commands at the start of test mode.
				CommandScheduler.getInstance().cancelAll();
		}

		/** This function is called periodically during test mode. */
		@Override
		public void testPeriodic() {
		}

		/** This function is called once when the robot is first started up. */
		@Override
		public void simulationInit() {
		}

		/** This function is called periodically whilst in simulation. */
		@Override
		public void simulationPeriodic() {
		}
}        

