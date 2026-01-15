package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveM2D {
	private Mechanism2d layout;

	private MechanismRoot2d frontRightRoot;
	private MechanismLigament2d frontRight;

	private MechanismRoot2d frontLeftRoot;
	private MechanismLigament2d frontLeft;

	private MechanismRoot2d backLeftRoot;
	private MechanismLigament2d backLeft;

	private MechanismRoot2d backRightRoot;
	private MechanismLigament2d backRight;

	public SwerveM2D() {
		layout = new Mechanism2d(6, 6);

		frontRightRoot = layout.getRoot("root1", 4.5, 4.5);
		frontRight = frontRightRoot.append(new MechanismLigament2d("motor1", 1, 0));

		frontLeftRoot = layout.getRoot("root2", 1.5, 4.5);
		frontLeft = frontLeftRoot.append(new MechanismLigament2d("motor2", 1, 0));

		backLeftRoot = layout.getRoot("root3", 1.5, 1.5);
		backLeft = backLeftRoot.append(new MechanismLigament2d("motor3", 1, 0));

		backRightRoot = layout.getRoot("root4", 4.5, 1.5);
		backRight = backRightRoot.append(new MechanismLigament2d("motor4", 1, 0));

		SmartDashboard.putData("Swerve Drive State", layout);
	}

	public void update(SwerveModuleState[] states) {
		frontRight.setAngle(states[0].angle.getDegrees() + 90.0d);
		frontRight.setLength(states[0].speedMetersPerSecond);

		frontLeft.setAngle(states[1].angle.getDegrees() + 90.0d);
		frontLeft.setLength(states[1].speedMetersPerSecond);

		backLeft.setAngle(states[2].angle.getDegrees() + 90.0d);
		backLeft.setLength(states[2].speedMetersPerSecond);

		backRight.setAngle(states[3].angle.getDegrees() + 90.0d);
		backRight.setLength(states[3].speedMetersPerSecond);

		SmartDashboard.putData("Swerve Drive State", layout);
	}
}