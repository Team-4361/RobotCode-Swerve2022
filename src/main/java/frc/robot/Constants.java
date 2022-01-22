package frc.robot;

public class Constants {
    public static final int XY_STICK_ID = 0;
    public static final int Z_STICK_ID = 1;
    public static final int CONTROLLER_ID = 2;


    // public static final double FR_OFFSET = 0.790 - ((3/2)* Math.PI);
    // public static final double FL_OFFSET = -2.95 - ((1/2) * Math.PI);
    // public static final double BR_OFFSET = -1.7 - (Math.PI/2);
    // public static final double BL_OFFSET = 0.111 - ((3/2)*Math.PI);

    public static final double FR_OFFSET = 0.790 + (-(2* Math.PI)- (Math.PI/2));
    public static final double FL_OFFSET = -2.95 + (-(Math.PI/2) - (Math.PI));
    public static final double BR_OFFSET = -1.7 - (Math.PI/2);
    public static final double BL_OFFSET = 0.111 + (-(2*Math.PI) - (Math.PI/2) - (Math.PI));

    // public static final double FR_OFFSET = 0.790 - (2* Math.PI)- (Math.PI/2);
    // public static final double FL_OFFSET = -2.95 - (Math.PI/2) - (Math.PI);
    // public static final double BR_OFFSET = -1.7 - (Math.PI/2);
    // public static final double BL_OFFSET = 0.111 - (2*Math.PI) - (Math.PI/2) - (Math.PI);

    public static final double DEADZONE = 0.05;
    public static final int LEFT_HANDED_BUTTON = 9;

    //In meters
    public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

    private Constants() {

    }
}
