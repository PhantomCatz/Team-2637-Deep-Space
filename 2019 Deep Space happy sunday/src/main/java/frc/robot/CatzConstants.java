package frc.robot;
/*
 *  Author : Derek Duenas

 *  Methods : 
 *  Functionality : Holds all global data values
 *   
 *  Revision History : 
 *  2-1-19 Added robot dimension constants DD
 * 
 */
public class CatzConstants
{
    public static final int NAVX_RESET_WAIT_TIME = 1;
    public static final int PCM_CANBUS_ID = 1;
    
    public static final double ROBOT_LENGTH = 0; //TBD
    public static final double ROBOT_WIDTH = 0; //
    public static final double HALF_ROBOT_LENGTH = ROBOT_LENGTH/2;
    public static final double HALF_ROBOT_WIDTH = ROBOT_WIDTH/2;

    public static boolean USING_COMPETITION_ROBOT = true;

    public static final double CONTROLLER_INPUT_WAIT_TIME = 0.005;

    public static final double INVALID_ANGLE = 999.9; 
    public static final double INVALID_EXT =   999.9; 

    public static final boolean USING_SOFT_LIMITS = false;

    public static final double NEO_MOTOR_TEMPERATURE_LIMIT = 65.0;
    public static final double REDLINE_MOTOR_TEMPERATURE_LIMIT = 65.0;
    public static final double PRO_775_MOTOR_TEMPERATURE_LIMIT = 65.0;

    public static final int SPARK_MAX_FIRMWARE = 0;

    public static final int CANBUS_ISSUES = 0;
    public static final int PDP_ISSUES = 1;
    public static final int MOTOR_ISSUES = 2;

    public static final int ARM_PIVOT_LT = 0;
    public static final int DRV_TRAIN_LT_BACK = 1;
    public static final int DRV_TRAIN_LT_MIDL = 2;
    public static final int DRV_TRAIN_LT_FRNT = 3;
    public static final int LIFT_RT = 4;
    public static final int LIFT_LT = 5;
    public static final int ARM_EXTENSION_A = 8;
    public static final int ARM_EXTENSION_B = 9;
    public static final int INTAKE_WRIST = 10;
    public static final int INTAKE_ROLLER = 11;
    public static final int DRV_TRAIN_RT_BACK = 12;
    public static final int DRV_TRAIN_RT_MIDL = 13;
    public static final int DRV_TRAIN_RT_FRNT = 14;
    public static final int ARM_PIVOT_RT = 15;



}