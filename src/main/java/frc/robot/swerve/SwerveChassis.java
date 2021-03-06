package frc.robot.swerve;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveChassis {
    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(-Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, -Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, -Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    SWERVE_FR_POSITION,
                    SWERVE_FL_POSITION,
                    SWERVE_BR_POSITION,
                    SWERVE_BL_POSITION
            );

    private static final int FR_DRIVE_ID = 4;
    private static final int FL_DRIVE_ID = 2;
    private static final int BR_DRIVE_ID = 8;
    private static final int BL_DRIVE_ID = 6;

    private static final int FR_TURN_ID = 3;
    private static final int FL_TURN_ID = 1;
    private static final int BR_TURN_ID = 7;
    private static final int BL_TURN_ID = 5;

    private static final int FR_DIO_ENCODER_PORT = 1;
    private static final int FL_DIO_ENCODER_PORT = 0;
    private static final int BR_DIO_ENCODER_PORT = 3;
    private static final int BL_DIO_ENCODER_PORT = 2;

    private static final String NAME_FR = "FR";
    private static final String NAME_FL = "FL";
    private static final String NAME_BR = "BR";
    private static final String NAME_BL = "BL";

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule backRight;
    private final SwerveModule backLeft;



    public SwerveChassis() {
        this(
                new SwerveModule(FR_DRIVE_ID, FR_TURN_ID, FR_DIO_ENCODER_PORT, Constants.FR_OFFSET),
                new SwerveModule(FL_DRIVE_ID, FL_TURN_ID, FL_DIO_ENCODER_PORT, Constants.FL_OFFSET),
                new SwerveModule(BR_DRIVE_ID, BR_TURN_ID, BR_DIO_ENCODER_PORT, Constants.BR_OFFSET),
                new SwerveModule(BL_DRIVE_ID, BL_TURN_ID, BL_DIO_ENCODER_PORT, Constants.BL_OFFSET)
        );
    }

    public SwerveChassis(SwerveModule frontRight,
                         SwerveModule frontLeft,
                         SwerveModule backRight,
                         SwerveModule backLeft) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    private void updateDashboard() {
        frontRight.updateDashboard(NAME_FR);
        frontLeft.updateDashboard(NAME_FL);
        backRight.updateDashboard(NAME_BR);
        backLeft.updateDashboard(NAME_BL);
    }

    public SwerveModule getFrontRight() {
        return frontRight;
    }

    public SwerveModule getFrontLeft() {
        return frontLeft;
    }

    public SwerveModule getBackRight() {
        return backRight;
    }

    public SwerveModule getBackLeft() {
        return backLeft;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return SWERVE_KINEMATICS;
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
                SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveModuleState frontRightState = states[0];
        SwerveModuleState frontLeftState = states[1];
        SwerveModuleState backRightState = states[2];
        SwerveModuleState backLeftState = states[3];

        frontRight.setState(frontRightState);
        frontLeft.setState(frontLeftState);
        backRight.setState(backRightState);
        backLeft.setState(backLeftState);

        updateDashboard();
    }
}
