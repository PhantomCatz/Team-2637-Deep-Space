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

    public static boolean USING_COMPETITION_ROBOT = false;

    public static final double CONTROLLER_INPUT_WAIT_TIME = 0.005;

    public static final double INVALID_ANGLE = 999.9; 

    public static final boolean USING_SOFT_LIMITS = false;

}