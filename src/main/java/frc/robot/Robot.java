package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.SwerveChassis;

import static frc.robot.Constants.*;

public class Robot extends TimedRobot {
    private final Joystick xyStick = new Joystick(Constants.XY_STICK_ID);
    private final Joystick zStick = new Joystick(Constants.Z_STICK_ID);
    private final XboxController controller =
            new XboxController(Constants.CONTROLLER_ID);
    private boolean leftHandedMode = false;

    private SwerveChassis chassis;
    //private Gyro gyro;

    @Override
    public void robotInit() {
        this.chassis = new SwerveChassis();
        //this.gyro = new AHRS(SPI.Port.kMXP);
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
        double x = adjustJoystickValues(xyStick.getX(), 0.05);
        double y = adjustJoystickValues(xyStick.getY(), 0.05);
        double z = adjustJoystickValues(zStick.getTwist(), 0.05);
        //Rotation2d rotation = gyro.getRotation2d();

        SmartDashboard.putNumber("X Input", x);
        SmartDashboard.putNumber("Y Input", y);
        SmartDashboard.putNumber("Z Input", z);
        //SmartDashboard.putNumber("Gyro Angle", rotation.getDegrees());

        // If the X button was pressed, change the orientation.
        if (xyStick.getRawButtonPressed(LEFT_HANDED_BUTTON)) {
            if (leftHandedMode) {
                leftHandedMode = false;
            } else { 
                leftHandedMode = true; 
            }
        }

        if (leftHandedMode) {
            // If left-handed mode is enabled, use the opposite Joystick for controlling.
            x = adjustJoystickValues(zStick.getX(), DEADZONE);
            y = adjustJoystickValues(zStick.getY(), DEADZONE);
            z = adjustJoystickValues(xyStick.getTwist(), DEADZONE);
        } else {
            x = adjustJoystickValues(xyStick.getX(), DEADZONE);
            y = adjustJoystickValues(xyStick.getY(), DEADZONE);
            z = adjustJoystickValues(zStick.getTwist(), DEADZONE);
        }

        
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, z, Rotation2d.fromDegrees(0));

        chassis.drive(speeds);
    }

    //Adds a deadzone
    public double adjustJoystickValues(double value, double deadzone){
        if(Math.abs(value) > deadzone){
            return value;
        }
        else{
            return 0;
        }
    }

    @Override
    public void testPeriodic() {

    }
}
