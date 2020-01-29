package org.usfirst.frc6647.subsystems;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.loops.ILooper;
import org.usfirst.lib6647.loops.Loop;
import org.usfirst.lib6647.loops.LoopType;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.PIDSuperSubsystem;
import org.usfirst.lib6647.subsystem.SuperSubsystem;
import org.usfirst.lib6647.subsystem.hypercomponents.HyperAHRS;
import org.usfirst.lib6647.subsystem.hypercomponents.HyperPIDController;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;
import org.usfirst.lib6647.wpilib.LooperRobot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Example implementation of a Chassis {@link SuperSubsystem Subsystem}, with
 * angle-arcade control.
 */
public class Chassis extends PIDSuperSubsystem implements SuperAHRS, SuperTalon, SuperVictor {
	/** {@link DifferentialDrive} used by this {@link SuperSubsystem Subsystem}. */
	private DifferentialDrive drive;
	/** {@link HyperAHRS} instance of the Robot's NavX. */
	private HyperAHRS navX;

	/**
	 * A lambda of every {@link SuperSubsystem Subsystem} must be provided to the
	 * {@link Robot}'s {@link LooperRobot super}'s constructor.
	 */
	public Chassis() {
		super("chassis");

		// All SuperComponents must be initialized like this. The 'robotMap' Object is
		// inherited from the SuperSubsystem class, while the second argument is simply
		// this Subsystem's name.
		initAHRS(robotMap, getName());
		initTalons(robotMap, getName());
		initVictors(robotMap, getName());

		// Additional initialization can be done here.
		getVictor("backLeft").follow(getTalon("frontLeft"));
		getVictor("backRight").follow(getTalon("frontRight"));

		drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
		drive.setDeadband(0);

		navX = ahrsDevices.get("navX");

		Robot.getInstance().getJoystick("driver1").get("X").whenPressed(new InstantCommand(() -> {
			getTalon("frontLeft").setSelectedSensorPosition(0, 0, 10);
			getTalon("frontRight").setSelectedSensorPosition(0, 0, 10);
		}));
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putNumber("l_encoder_pos", getTalon("frontLeft").getSelectedSensorPosition(0));
		SmartDashboard.putNumber("l_encoder_vel", getTalon("frontLeft").getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("r_encoder_pos", getTalon("frontRight").getSelectedSensorPosition(0));
		SmartDashboard.putNumber("r_encoder_vel", getTalon("frontRight").getSelectedSensorPosition(0));
	}

	@Override
	public void registerLoops(ILooper looper) {
		looper.register(new Loop() {
			private JController joystick;
			private HyperPIDController pidGyro;

			@Override
			public void onFirstStart(double timestamp) {
			}

			@Override
			public synchronized void onStart(double timestamp) {
				joystick = Robot.getInstance().getJoystick("driver1");
				pidGyro = getPIDController("gyro");

				navX.reset();
				setSetpoint("gyro", navX.getYaw());
				System.out.println("Started angle-arcade drive at: " + timestamp + "!");
			}

			@Override
			public synchronized void onLoop(double timestamp) {
				setSetpoint("gyro",
						Math.abs(joystick.getRawAxis(5)) > 0.15 || Math.abs(joystick.getRawAxis(2)) > 0.15
								? Math.toDegrees(Math.atan2(joystick.getRawAxis(5), joystick.getRawAxis(2)))
								: navX.getYaw());
				double output = pidGyro.calculate(navX.getYaw());
				drive.arcadeDrive(joystick.getRawAxis(1), output, false);
			}

			@Override
			public void onStop(double timestamp) {
				System.out.println("Stopped angle-arcade drive at: " + timestamp + ".");
			}

			@Override
			public LoopType getType() {
				return LoopType.TELEOP;
			}
		});
	}
}