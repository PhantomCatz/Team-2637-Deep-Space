package frc.Autonomous;

public class CatzAutonomousPaths 
{
    public void middleAutonPath(int rightOrLeftHatch, int rightOrLeftCargo)
    { 
        //change to enum
        final double DISTANCE_TO_CARGOSHIP = 0;
        final double TURN_ANGLE_TO_CARGO_HATCH = 0;
        final double TURN_ANGLE_TO_LOAD_FROM_CARGO = 0;
        final double DISTANCE_TO_LOAD_FROM_CARGOSHIP = 0;
        final double TURN_ANGLE_TO_LOAD_FROM_PATH_TO_LOAD = 0;
        final double DISTANCE_TO_LOAD_FROM_PATH_TO_LOAD = 0;
        final double DISTANCE_BACK_FROM_LOAD = 0;
        final double DISTANCE_BACK_FROM_HATCH = 0;
        final double ADDED_DISTANCE_FROM_CARGO = 0;
        
        final int right = 1;
        final int left = 0;
        
        CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_CARGOSHIP, 0);

        if(rightOrLeftCargo == right)
        {
            CatzTurn.PIDturn(TURN_ANGLE_TO_CARGO_HATCH, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP, 0);

            CatzTurn.PIDturn(-TURN_ANGLE_TO_CARGO_HATCH, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_HATCH, 0);
        }
        else
        {
            CatzTurn.PIDturn(-TURN_ANGLE_TO_CARGO_HATCH, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP, 0);

            CatzTurn.PIDturn(TURN_ANGLE_TO_CARGO_HATCH, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_HATCH, 0);
        }

        if(rightOrLeftCargo == right && rightOrLeftHatch == right)
        {
            CatzTurn.PIDturn(TURN_ANGLE_TO_LOAD_FROM_CARGO, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP, 0);

            CatzTurn.PIDturn(TURN_ANGLE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_LOAD, 0);
        }
        else if(rightOrLeftCargo == left && rightOrLeftHatch == left)
        {
            CatzTurn.PIDturn(-TURN_ANGLE_TO_LOAD_FROM_CARGO, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP, 0);

            CatzTurn.PIDturn(-TURN_ANGLE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_LOAD, 0);
        }
        else if(rightOrLeftCargo == right && rightOrLeftHatch == left)
        {
            CatzTurn.PIDturn(TURN_ANGLE_TO_LOAD_FROM_CARGO, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP+ADDED_DISTANCE_FROM_CARGO, 0);

            CatzTurn.PIDturn(TURN_ANGLE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_LOAD, 0);
        }
        else if(rightOrLeftCargo == left && rightOrLeftHatch == right)
        {
            CatzTurn.PIDturn(-TURN_ANGLE_TO_LOAD_FROM_CARGO, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_CARGOSHIP+ADDED_DISTANCE_FROM_CARGO, 0);

            CatzTurn.PIDturn(-TURN_ANGLE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_TO_LOAD_FROM_PATH_TO_LOAD, 0);

            CatzDriveStraight.PIDDriveNoTrig(0, DISTANCE_BACK_FROM_LOAD, 0);
        }
    }
}