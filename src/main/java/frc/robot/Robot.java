package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.SwerveChassis;

public class Robot extends TimedRobot {
    private final Joystick xyStick = new Joystick(Constants.XY_STICK_ID);
    private final Joystick zStick = new Joystick(Constants.Z_STICK_ID);
    private final XboxController controller =
            new XboxController(Constants.CONTROLLER_ID);

    private SwerveChassis chassis;
    private Gyro gyro;

    @Override
    public void robotInit() {
        this.chassis = new SwerveChassis();
        this.gyro = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        double x = xyStick.getX();
        double y = xyStick.getY();
        double z = zStick.getX() * 2 * MATH.PI ;

        Rotation2d rotation = gyro.getRotation2d();

        SmartDashboard.putNumber("X Input", x);
        SmartDashboard.putNumber("Y Input", y);
        SmartDashboard.putNumber("Z Input", z);
        SmartDashboard.putNumber("Gyro Angle", rotation.getDegrees());

        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, rotation);

        chassis.drive(speeds);
    }

    @Override
    public void testPeriodic() {

    }
}
