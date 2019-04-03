package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class CatzPreSets
{
    //Values for ball and hatch pickup
    private static double NEUTRAL_PIVOT_ANGLE = 2.0;
    private static double NEUTRAL_ARM_EXT     = 12.844;
    private static double NEUTRAL_WRIST_ANGLE = -48.6;
    private static double NEUTRAL_LIFT_HEIGHT = 0.0;

    private static double HATCH_PICKUP_PIVOT_ANGLE = 30.0;
    private static double HATCH_PICKUP_ARM_EXT     = 8;
    private static double HATCH_PICKUP_WRIST_ANGLE = -105.0;
    private static double HATCH_PICKUP_LIFT_HEIGHT = 0.0;


    /**
     * 
     *  Values for stowed
     * 
     */
    private static double PRESTOWED_PIVOT_ANGLE = 0.0;
    private static double PRESTOWED_ARM_EXT     = 0.0;
    private static double PRESTOWED_WRIST_ANGLE = 0.0;
    private static double PRESTOWED_LIFT_HEIGHT = 0.0;
    
    private static double STOWED_PIVOT_ANGLE = 28.98;
    private static double STOWED_ARM_EXT     = 0.0;
    private static double STOWED_WRIST_ANGLE = 65.09;
    private static double STOWED_LIFT_HEIGHT = 0.0;

    /**
     * 
     *  Traveled Preset
     * 
     */
    private static double TRAVEL_PIVOT_ANGLE = 80.0;
    private static double TRAVEL_ARM_EXT     = 1.0;
    private static double TRAVEL_WRIST_ANGLE = 47.0;

    //Values for scoring in the cargo bay
    private static double CARGO_BAY_BALL_PIVOT_ANGLE = 99.166;
    private static double CARGO_BAY_BALL_ARM_EXT     = 12.844;
    private static double CARGO_BAY_BALL_WRIST_ANGLE = 76.035;
    private static double CARGO_BAY_BALL_LIFT_HEIGHT = 0.0;

    private static double CARGO_BAY_HATCH_PIVOT_ANGLE = 21.0;
    private static double CARGO_BAY_HATCH_ARM_EXT     = 3.0;
    private static double CARGO_BAY_HATCH_WRIST_ANGLE = -100.0;
    private static double CARGO_BAY_HATCH_LIFT_HEIGHT = 0.0;

    
    //Values for scoring in the rocket
    private static double LVL_3_BALL_PIVOT_ANGLE = 90;
    private static double LVL_3_BALL_ARM_EXT = 15;
    private static double LVL_3_BALL_WRIST_ANGLE = 15; 
    private static double LVL_3_BALL_LIFT_HEIGHT = 0.0; //TBD

    private static double LVL_2_BALL_PIVOT_ANGLE = 101.4;
    private static double LVL_2_BALL_ARM_EXT = 16.4;
    private static double LVL_2_BALL_WRIST_ANGLE = 29.5;
    private static double LVL_2_BALL_LIFT_HEIGHT = 0.0;

    private static double LVL_1_BALL_PIVOT_ANGLE = 53.0;
    private static double LVL_1_BALL_ARM_EXT     = 0.0;
    private static double LVL_1_BALL_WRIST_ANGLE = -5.0;
    private static double LVL_1_BALL_LIFT_HEIGHT = 0.0;

    private static double LVL_3_HATCH_PIVOT_ANGLE = 104.0;
    private static double LVL_3_HATCH_ARM_EXT     = 15;
    private static double LVL_3_HATCH_WRIST_ANGLE = -38.6; 
    private static double LVL_3_HATCH_LIFT_HEIGHT = 0.0; //TBD

    private static double LVL_2_HATCH_PIVOT_ANGLE = 96.0;
    private static double LVL_2_HATCH_ARM_EXT = 13.1;
    private static double LVL_2_HATCH_WRIST_ANGLE = -42.4;
    private static double LVL_2_HATCH_LIFT_HEIGHT = 0.0;

    private static double LVL_1_HATCH_PIVOT_ANGLE = 21.0;
    private static double LVL_1_HATCH_ARM_EXT = 3.0;
    private static double LVL_1_HATCH_WRIST_ANGLE = -100.0;
    private static double LVL_1_HATCH_LIFT_HEIGHT = 0.0;


    /** 
     *  Values for scoring in the cargo bay on the back side of the robot
    */
    private static double CARGO_BAY_REV_BALL_PIVOT_ANGLE = 155.0;
    private static double CARGO_BAY_REV_BALL_WRIST_ANGLE = -48.6;
    private static double CARGO_BAY_REV_BALL_ARM_EXT     = 9.844;

    public static void setPosition(double pivotTargetAngle, double armTargetExtension, 
                                   double wristTargetAngle, double liftTargetHeight)
    {
        Robot.arm.setPivotTargetAngle(pivotTargetAngle);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(armTargetExtension);
        
        Robot.intake.setWristTargetAngle(wristTargetAngle);
        Robot.lift.setLiftTargetHeight(liftTargetHeight);
    }

    public static void cargoPickUp()
    {
        Robot.intake.setWristTargetAngle(NEUTRAL_WRIST_ANGLE);
        Timer.delay(0.2); //Wait for wrist to be positioned not to hit the ground 
        
        Robot.arm.setPivotTargetAngle(NEUTRAL_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(NEUTRAL_ARM_EXT);
        
    }

    public static void hatchPickUp()
    {
        Robot.arm.setPivotTargetAngle(HATCH_PICKUP_PIVOT_ANGLE);
        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(HATCH_PICKUP_ARM_EXT);
     
        Robot.intake.setWristTargetAngle(HATCH_PICKUP_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(HATCH_PICKUP_LIFT_HEIGHT);
    }


    /**
     * 
     *  Positions for Scoring in Cargo Bay
     * 
     */
    public static void cargoBayBall()
    {
        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(CARGO_BAY_BALL_ARM_EXT);
        Timer.delay(0.200);

        Robot.arm.setPivotTargetAngle(CARGO_BAY_BALL_PIVOT_ANGLE);
        Timer.delay(0.100);

        Robot.intake.setWristTargetAngle(CARGO_BAY_BALL_WRIST_ANGLE);
    }

    public static void cargoBayHatch()
    {
        //Hatch spot is close to hatch pickup so use the same values
        hatchPickUp();

        /*Robot.arm.setPivotTargetAngle(CARGO_BAY_HATCH_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(CARGO_BAY_HATCH_ARM_EXT);
        
        Robot.intake.setWristTargetAngle(CARGO_BAY_HATCH_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(CARGO_BAY_HATCH_LIFT_HEIGHT);*/
    }

    /*
     * 
     *  Positions for Cargo in Rocket
     * 
     */

    public static void lvl3RocketBall()
    {
        Robot.arm.setPivotTargetAngle(LVL_3_BALL_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(LVL_3_BALL_ARM_EXT);

        Robot.intake.setWristTargetAngle(LVL_3_BALL_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(LVL_3_BALL_LIFT_HEIGHT);
    }

    public static void lvl2RocketBall()
    {
        Robot.arm.setPivotTargetAngle(LVL_2_BALL_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(LVL_2_BALL_ARM_EXT);

        Robot.intake.setWristTargetAngle(LVL_2_BALL_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(LVL_2_BALL_LIFT_HEIGHT);
    }

    public static void lvl1RocketBall()
    {
        Robot.arm.setPivotTargetAngle(LVL_1_BALL_PIVOT_ANGLE);
        Timer.delay(0.2);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(LVL_1_BALL_ARM_EXT);

        Robot.intake.setWristTargetAngle(LVL_1_BALL_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(LVL_1_BALL_LIFT_HEIGHT);
    }


    /*
     * 
     *  Positions for Hatch in Rocket
     * 
     */

    public static void lvl3RocketHatch()
    {
        Robot.arm.setPivotTargetAngle(LVL_3_HATCH_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(LVL_3_HATCH_ARM_EXT);

        Robot.intake.setWristTargetAngle(LVL_3_HATCH_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(LVL_3_HATCH_LIFT_HEIGHT);
    }

    public static void lvl2RocketHatch()
    {
        Robot.arm.setPivotTargetAngle(LVL_2_HATCH_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(LVL_2_HATCH_ARM_EXT);

        Robot.intake.setWristTargetAngle(LVL_2_HATCH_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(LVL_2_HATCH_LIFT_HEIGHT);
    }

    public static void lvl1RocketHatch()
    {
        //Hatch spot is close to hatch pickup so use the same values
        hatchPickUp();
    }

    //TODO
    public static void stowed()
    {
        Robot.arm.setPivotTargetAngle(STOWED_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(STOWED_ARM_EXT);
        
        Robot.intake.setWristTargetAngle(STOWED_WRIST_ANGLE);
        Robot.lift.setLiftTargetHeight(STOWED_LIFT_HEIGHT);
    }
 
    //also for scoring in rocket lvl 1
    public static void transport()
    {
        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(Robot.arm.getArmTargetExt() - 1);
        
        Robot.arm.setPivotTargetAngle(TRAVEL_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(TRAVEL_ARM_EXT);

        Timer.delay(0.15);
        Robot.intake.setWristTargetAngle(TRAVEL_WRIST_ANGLE);
    }

    public static void cargoBayReversed()
    {
        Robot.arm.setPivotTargetAngle(CARGO_BAY_REV_BALL_PIVOT_ANGLE);

        Robot.arm.setArmTargetHit(false);
        Robot.arm.setArmTargetExt(CARGO_BAY_REV_BALL_ARM_EXT);

        Robot.intake.setWristTargetAngle(CARGO_BAY_BALL_WRIST_ANGLE);
    }

}