package frc.Autonomous;

public class CatzAutonomousPaths 
{
    
    final double turn_R_90 = 90.0;
    final double turn_L_90 = -90.0;
    final double middle_to_cargoship = 150.0;
    final double to_R_L_hatch =11.75; 
    final double approach_hatch = 51.13;
    final double backup_from_hatch = -40.0;
    double hatch_to_reload = 121.815;
    final double approach_reload = 180.25;
    final double backup_from_reload = -170.0;
    double to_other_hatch = 145.315;
    final double approach_to_other_hatch = 50.25;

    public void middleFrontCargoPath(String hatchLocation, String reloadLocation)
    {
        placeFirstHatch(hatchLocation);
        toReload(hatchLocation,reloadLocation);
        placeSecondHatch(hatchLocation,reloadLocation);
    }

    public void placeFirstHatch(String hatchLocationR_L) 
    {
        // starts from level 1, then it can go eitherside and place the hatch

        CatzDriveStraight.PIDDriveNoTrig(0.0, middle_to_cargoship, 0.0);
            
        if (hatchLocationR_L.equalsIgnoreCase("left"))
        {
            CatzTurn.PIDturn(turn_L_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, to_R_L_hatch, 0.0);
            CatzTurn.PIDturn(turn_R_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, approach_hatch, 0.0);
            //placeHatch();
        }
        else if(hatchLocationR_L.equalsIgnoreCase("right"))
        {
            CatzTurn.PIDturn(turn_R_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, to_R_L_hatch, 0.0);
            CatzTurn.PIDturn(turn_L_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, approach_hatch, 0.0); 
            //placeHatch();
        }
    }


    public void toReload(String hatchLocationR_L,String reloadLocation)
    {
        if (hatchLocationR_L.equalsIgnoreCase("left"))
        {
            if(reloadLocation.equalsIgnoreCase("right"))
            {
                hatch_to_reload += 11.75;
            }
        }   
        else if(hatchLocationR_L.equalsIgnoreCase("right"))
        {
            if(reloadLocation.equalsIgnoreCase("left"))
            {
                hatch_to_reload += 11.75;
            }
        }
        // after placeing the hatch, the robot moves to the hatch reload station
        CatzDriveStraight.PIDDriveNoTrig(0.0, backup_from_hatch, 0.0);
        
        if(reloadLocation.equalsIgnoreCase("left"))
        {
            CatzTurn.PIDturn(turn_L_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, hatch_to_reload, 0.0);
            CatzTurn.PIDturn(turn_L_90,0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, approach_reload, 0.0);
            //getHatch()
        }
        else if(reloadLocation.equalsIgnoreCase("right")) //right
        {
            CatzTurn.PIDturn(turn_R_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, hatch_to_reload, 0.0);
            CatzTurn.PIDturn(turn_R_90, 0.0);
            CatzDriveStraight.PIDDriveNoTrig(0.0, approach_reload, 0.0);
            //getHatch()
        }
    }

    public void placeSecondHatch(String hatchLocationR_L, String reloadLocation)
    {
        if (reloadLocation.equalsIgnoreCase("left"))
        {
            if(hatchLocationR_L.equalsIgnoreCase("right"))
            {
                to_other_hatch += 11.75;
            }
        }
        else if(reloadLocation.equalsIgnoreCase("right"))
        {
            if(hatchLocationR_L.equalsIgnoreCase("left"))
            {
                to_other_hatch += 11.75;
            }
        
        }
        CatzDriveStraight.PIDDriveNoTrig(0.0, backup_from_reload, 0.0);

        if (reloadLocation.equalsIgnoreCase("left"))
        {
                CatzTurn.PIDturn(turn_L_90, 0.0);
                CatzDriveStraight.PIDDriveNoTrig(0.0, to_other_hatch, 0.0);
                CatzTurn.PIDturn(turn_L_90, 0.0);
                CatzDriveStraight.PIDDriveNoTrig(0.0, approach_to_other_hatch, 0.0);
                //placeHatch();
        }
        else if(reloadLocation.equalsIgnoreCase("right"))
        {
                CatzTurn.PIDturn(turn_R_90, 0.0);
                CatzDriveStraight.PIDDriveNoTrig(0.0, to_other_hatch, 0.0);
                CatzTurn.PIDturn(turn_R_90, 0.0);
                CatzDriveStraight.PIDDriveNoTrig(0.0, approach_to_other_hatch, 0.0);
                //placeHatch();
        }
    }
}