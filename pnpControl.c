/*
 *
 * pnpControl.c - the controller for the pick and place machine in manual and autonomous mode
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "pnpControl.h"

// state names and numbers
#define HOME                  0
#define MOVE_TO_FEEDER        1
#define WAIT_1                2
#define PICKING_PART          3
#define WAIT_2                4
#define MOVE_TO_LOOKUP        5
#define LOOKUP_PHOTO          6
#define MOVE_TO_NOM_POS       7
#define LOOKDOWN_PHOTO        8
#define WAIT_3                9
#define ROTATIONAL_CORRECT    10
#define WAIT_4                11
#define AMEND_POS             12
#define WAIT_5                13
#define PLACING_PART          14
#define MOVE_TO_HOME          15

/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[16][21] = {"HOME                ",
                                "MOVE TO FEEDER       ",
                                "WAIT 1               ",
                                "PICKING PART         ",
                                "WAIT 2               ",
                                "MOVE TO LOOKUP       ",
                                "LOOKUP PHOTO         ",
                                "MOVE TO NOM POS      ",
                                "LOOKDOWN PHOTO       ",
                                "WAIT 3               ",
                                "ROTATIONAL CORRECT   ",
                                "WAIT 4               ",
                                "AMEND POS            ",
                                "WAIT_5               ",
                                "PLACING PART         ",
                                "MOVE_TO_HOME         "};

const double TAPE_FEEDER_X[NUMBER_OF_FEEDERS] = {FDR_0_X, FDR_1_X, FDR_2_X, FDR_3_X, FDR_4_X, FDR_5_X, FDR_6_X, FDR_7_X, FDR_8_X, FDR_9_X};
const double TAPE_FEEDER_Y[NUMBER_OF_FEEDERS] = {FDR_0_Y, FDR_1_Y, FDR_2_Y, FDR_3_Y, FDR_4_Y, FDR_5_Y, FDR_6_Y, FDR_7_Y, FDR_8_Y, FDR_9_Y};
double theta_requested;
double theta_pick_error[3];
double x_preplace_error;
double y_preplace_error;
int placing_counter = 0;        // used for incrementing picking_part state sub modes
int picking_counter = 0;        // used for incrementing placing_oart state sub modes
int component_count = 0;

const char nozzle_name[3][10] = {"left", "centre", "right"};

int main()
{
    pnpOpen();

    int operation_mode, number_of_components_to_place, res;
    PlacementInfo pi[MAX_NUMBER_OF_COMPONENTS_TO_PLACE];

    /*
     * read the centroid file to obtain the operation mode, number of components to place
     * and the placement information for those components
     */
    res = getCentroidFileContents(&operation_mode, &number_of_components_to_place, pi);

    if (res != CENTROID_FILE_PRESENT_AND_READ)
    {
        printf("Problem with centroid file, error code %d, press any key to continue\n", res);
        getchar();
        exit(res);
    }

    /* state machine code for manual control mode */
    if (operation_mode == MANUAL_CONTROL)
    {
        /* initialization of variables and controller window */
        int state = HOME, finished = FALSE;

        char c;
        printf("Time: %7.2f  Initial state: %.15s  Operating in manual control mode, there are %d parts to place\n\n", getSimTime(), state_name[HOME], number_of_components_to_place);
        /* print details of part 0 */
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               pi[0].component_designation, pi[0].component_footprint, pi[0].component_value, pi[0].x_target, pi[0].y_target, pi[0].theta_target, pi[0].feeder);

        /* loop until user quits */
        while(!isPnPSimulationQuitFlagOn())
        {

            c = getKey();

            switch (state)
            {
                case HOME:

                    // component_count variable is used to increment and identify the current part and associated feeder number etc.
                    // number_of_components_to_place variable is total number of parts from 0-n

                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        // the expression (c - '0') obtains the integer value of the number key pressed above
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimTime(), state_name[state], c);
                    }

                    break;

                case MOVE_TO_FEEDER:    // state for confirming function is completed

                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s  Arrived at feeder, waiting for next instruction\n", getSimTime(), state_name[state]);
                    }
                    break;

/**********************************************    Moved to feeder position awaiting next command to commence picking states ****************************************************/

                case WAIT_1:    // First 'if' evaluation is incase the user wishes to change feeder and revert back through the initial sequence.
                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        // the expression (c - '0') obtains the integer value of the number key pressed
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimTime(), state_name[state], c);
                    }
                    else if (finished == FALSE && (c == 'p')){ // Command 'p' to commence picking issued, change state for picking - 'PICKING_PART'
                        lowerNozzle(CENTRE_NOZZLE);
                        state = PICKING_PART;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to lower nozzle\n", getSimTime(), state_name[state]);
                    }
                    break;

/*************************************************** PICKING_PART STATE ***************************************************************************/

                    case PICKING_PART:    // 'picking_counter' increment progresses picking state through 3 sub modes (lower/suction/raise)
                    if (isSimulatorReadyForNextInstruction() && picking_counter == 0)  // If nozzle lowering complete apply vacuum and change to second picking sub mode
                    {
                        printf("Time: %7.2f  Nozzle lowered\n", getSimTime());
                        applyVacuum(CENTRE_NOZZLE);
                        picking_counter++;      // increment to progress to raise nozzle sub mode
                        printf("Time: %7.2f  Current state: %.20s  Issued instruction to apply suction\n", getSimTime(), state_name[state]);
                    }

                    else if (isSimulatorReadyForNextInstruction() && picking_counter == 1)  // If suction applied (part picked) then raise nozzle back up, change to final sub mode
                    {
                        printf("Time: %7.2f  Suction applied\n", getSimTime());
                        raiseNozzle(CENTRE_NOZZLE);
                        picking_counter++;
                        printf("Time: %7.2f  Current state: %.20s  Issued instruction to raise nozzle\n", getSimTime(), state_name[state]);
                    }

                     else if (isSimulatorReadyForNextInstruction() && picking_counter == 2)  // Nozzle raised and ready for next command to move to lookup camera, state to 'WAIT_2'
                    {
                        printf("Time: %7.2f  Nozzle raised\n", getSimTime());
                        picking_counter = 0;                                                 // reset picking_counter for next full cycle
                        state = WAIT_2;
                        printf("Time: %7.2f  New state: %.20s  Picking complete, awaiting next instruction\n", getSimTime(), state_name[state]);
                    }
                    break;

    /********************************   Picking now complete next states are for cameras, alignment and placement    *************************************/
                case WAIT_2:
                    if (finished == FALSE && (c == 'c')) // Command 'c' to commence dual camera alignment checks issued, move to lookup camera and change state - 'MOVE_TO_LOOKUP'
                       {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(LOOKUP_CAMERA_X, LOOKUP_CAMERA_Y);    // Move to lookup camera position
                        state = MOVE_TO_LOOKUP;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to lookup camera %c\n", getSimTime(), state_name[state], c);
                       }
                    break;

                case MOVE_TO_LOOKUP:
                    if (isSimulatorReadyForNextInstruction())  // Gantry head at lookup camera position ready for photo to check rotation, change state  - 'LOOKUP_PHOTO'
                    {
                        printf("Time: %7.2f  Gantry at Lookup Camera position\n", getSimTime());
                        takePhoto(PHOTO_LOOKUP);    // Take photo to check rotational error for placement correction
                        state = LOOKUP_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Taking Lookup Photo\n", getSimTime(), state_name[state]);
                    }
                    break;

                case LOOKUP_PHOTO:
                    if (isSimulatorReadyForNextInstruction())
                        //  Lookup photo taken, error stored in >>> double theta_pick_error[nozzle] = getPickErrorTheta(nozzle); <<< state to - 'MOVE_TO_NOM_POS'
                    {
                        printf("Time: %7.2f  Lookup photo taken\n", getSimTime());
                        setTargetPos(pi[component_count].x_target, pi[component_count].y_target);    // Move to nominal board position for placement
                        state = MOVE_TO_NOM_POS;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to nominal board position %c\n", getSimTime(), state_name[state], c);
                    }

                    break;

                case MOVE_TO_NOM_POS:
                    if (isSimulatorReadyForNextInstruction()) // Moved to nom position, change state - 'LOOKDOWN_PHOTO'
                    {
                        printf("Time: %7.2f  Moved to nominal board position\n", getSimTime());
                        takePhoto(PHOTO_LOOKDOWN); // Take lookdown photo to check for translational error in X/Y planes
                        state = LOOKDOWN_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to complete Lookdown photo \n", getSimTime(), state_name[state] );
                    }

                    break;

                case LOOKDOWN_PHOTO:
                    if (isSimulatorReadyForNextInstruction()) // Lookdown photo taken
                        {
                            state = WAIT_3;
                            printf("Time: %7.2f  New state: %.20s  Lookdown photo taken, waiting for next instruction\n", getSimTime(), state_name[state]);
                        }
                    break;

   /*******************************************   Camera checks finished, in position, awaiting Theta and Translation corrections   *********************************************/

                case WAIT_3:
                    if (finished == FALSE && (c == 'r')) // Begin rotational alignment
                    {
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to correct rotational error \n", getSimTime(), state_name[state] );
                        theta_pick_error[CENTRE_NOZZLE] = getPickErrorTheta(CENTRE_NOZZLE);                    // recover error theta value
                        theta_requested = pi[component_count].theta_target-theta_pick_error[CENTRE_NOZZLE];    // adjust nominal value with error
                        rotateNozzle(CENTRE_NOZZLE, theta_requested);                                          // rotate to new requested value
                        state = ROTATIONAL_CORRECT;     // change state
                    }

                    break;

                case ROTATIONAL_CORRECT:
                    if (isSimulatorReadyForNextInstruction()) // Rotation completed
                    {
                        state = WAIT_4;
                        printf("Time: %7.2f  New state: %.20s  Rotation correction completed, waiting for next instruction\n", getSimTime(), state_name[state]);
                    }

                    break;

                case WAIT_4:
                    if (finished == FALSE && (c == 'a')) // Begin translational alignment
                    {
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to correct translational error %c\n", getSimTime(), state_name[state], c);
                        x_preplace_error = getPreplaceErrorX();     // store x error value in this variable
                        y_preplace_error = getPreplaceErrorY();     // store yerror value in this variable
                        amendPos(-x_preplace_error, -y_preplace_error);     // carry out correction using both variables above
                        state = AMEND_POS;               // change state
                    }

                    break;

                case AMEND_POS:
                    if (isSimulatorReadyForNextInstruction()) // Translation correction completed
                    {
                        state = WAIT_5;
                        printf("Time: %7.2f  New state: %.20s  Translational correction completed, waiting for next instruction\n", getSimTime(), state_name[state]);
                    }

                    break;
  /************************************************  Placement of part and return home  *********************************************/
                case WAIT_5:
                     if (finished == FALSE && (c == 'p')) // Begin placement
                     {
                        lowerNozzle(CENTRE_NOZZLE);
                        state = PLACING_PART;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to begin placement\n", getSimTime(), state_name[state]);
                    }
                    break;

                case PLACING_PART:      // 'placing_counter' increments to progress through 3 placing sub modes (lower/releaseSuction/raise)
                    if (isSimulatorReadyForNextInstruction() && placing_counter == 0)  // If nozzle lowering complete apply vacuum and change to next placing sub mode
                    {
                        printf("Time: %7.2f  Nozzle lowered\n", getSimTime());
                        releaseVacuum(CENTRE_NOZZLE);
                        placing_counter++;              // increments to move to next placing sub mode
                        printf("Time: %7.2f  Current state: %.20s  Releasing suction\n", getSimTime(), state_name[state]);
                    }

                   else if (isSimulatorReadyForNextInstruction() && placing_counter == 1)  // If suction released raise nozzle back up, change to final placing sub mode
                    {
                        printf("Time: %7.2f  Suction released\n", getSimTime());
                        raiseNozzle(CENTRE_NOZZLE);
                        placing_counter++;
                        printf("Time: %7.2f  Current state: %.20s  Raising  nozzle\n", getSimTime(), state_name[state]);
                    }

                     else if (isSimulatorReadyForNextInstruction() && placing_counter == 2)  // Nozzle raised and ready to move to home position
                    {
                        printf("Time: %7.2f  Nozzle raised\n", getSimTime());
                        state = MOVE_TO_HOME;
                        placing_counter = 0;
                        printf("Time: %7.2f  New state: %.20s  Placement complete\n", getSimTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_HOME:
                    if(component_count <  (number_of_components_to_place-1))
                    {
                       component_count++;
                       setTargetPos(HOME_X, HOME_Y);        // If continuing then carry on as normal
                       state = HOME;
                       printf("Time: %7.2f  New state: %.20s  Move to home \n", getSimTime(), state_name[state]);
                       printf("Part %d details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               component_count,pi[component_count].component_designation, pi[component_count].component_footprint, pi[component_count].component_value, pi[component_count].x_target, pi[component_count].y_target, pi[component_count].theta_target, pi[component_count].feeder);
                    }
                    else if(component_count ==  (number_of_components_to_place-1))
                    {
                       setTargetPos(HOME_X, HOME_Y);        // if last part has been placed give the below message to user
                       state = HOME;
                       printf("Time: %7.2f  New state: %.20s  Moving to home\n", getSimTime(), state_name[state]);

                       printf("Time: %7.2f  All %d Parts have been placed\n", getSimTime(), number_of_components_to_place);
                       finished = TRUE;

                    }

                break;



            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }
    }
    /* *****************************************   state machine code for autonomous control mode ******************************************************/

    else        // *********************************   AUTOMATIC OPERATION MODE  **************************************
    {
        /* initialization of variables and controller window */
        int state = HOME, finished = FALSE;
        int partsCount;     // variable used for sorting 'for' loop

       // component_count variable is used to increment and identify the current part and associated feeder number etc.
       // number_of_components_to_place variable is total number of parts from 0-n

        printf("Time: %7.2f  Initial state: %.15s  Operating in auto control mode, there are %d parts to place\n\n", getSimTime(), state_name[HOME], number_of_components_to_place);

/************************************************ Section to order parts according to feeder locations ***************************************/

            int feedCount = 0;                                              // used to increment feederarray position below
            int feederNo;                                                   // variable for feeder number in 'for' loop below
            // feeder array 'fa' struct array for reorder below
            PlacementInfo fa[number_of_components_to_place];     // mirror struct array for sorting feeder bins together in ascending order

       // nested 'for' loops to check each parts feeder number and place into struct array in position based on feeder number

            for(feederNo = 0; feederNo <= 9;feederNo++){                                            // iterate loop for each feeder number

                for(partsCount = 0; partsCount < number_of_components_to_place; partsCount++){    // iterate loop for each part to be placed

                    if(pi[partsCount].feeder == feederNo){                          // if part has current feeder number queue in feederarray
                         fa[feedCount] = pi[partsCount];
                        feedCount++;                                                // increment 'feederarray' array position by 1
                    }
                    else{}

                    }
                }
        printf("The below pick and place part list has been ordered by feeder number\n\n");

        // Prints new order of parts with designation and feeder number for reference
        for (partsCount = 0; partsCount < number_of_components_to_place; partsCount++){
                printf("Part %d details: Designation: %s Feeder: %d\n", partsCount, fa[partsCount].component_designation, fa[partsCount].feeder);
        }
        // Copies new ordered list back into pi[] stuct array for use in program below
        for(partsCount = 0; partsCount < number_of_components_to_place; partsCount++){
            pi[partsCount] = fa[partsCount];
        }
/******************************************** Main program state code section below ***************************************************/

        /* print details of part 0 */
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               pi[component_count].component_designation, pi[component_count].component_footprint, pi[component_count].component_value, pi[component_count].x_target, pi[component_count].y_target, pi[component_count].theta_target, pi[component_count].feeder);
        /* loop until user quits */

        while(!isPnPSimulationQuitFlagOn())
        {
            switch (state)
            {
                case HOME:

                    if (finished == FALSE && isSimulatorReadyForNextInstruction())
                    {
                        int feedNumber = (pi[component_count].feeder);
                        setTargetPos(TAPE_FEEDER_X[feedNumber], TAPE_FEEDER_Y[feedNumber]); // automatically loads first required feeder number to go to
                        printf("Time: %7.2f  New state: %.20s  Moving to tape feeder\n", getSimTime(), state_name[state]);
                        state = MOVE_TO_FEEDER;
                        }


                    break;

/**********************************************    Moved to feeder position commencing picking state ****************************************************/

                case MOVE_TO_FEEDER:
                    if (isSimulatorReadyForNextInstruction()){
                       lowerNozzle(CENTRE_NOZZLE);
                       printf("Time: %7.2f  New state: %.20s Arrived at feeder, Lowering nozzle\n", getSimTime(), state_name[state]);
                       state = PICKING_PART;

                    }

                    break;

/*************************************************** PICKING_PART STATE ***************************************************************************/

                    case PICKING_PART:    // 'picking_counter' increment progresses picking state through 3 sub modes (lower/suction/raise)
                    if (isSimulatorReadyForNextInstruction() && picking_counter == 0)  // If nozzle lowering complete apply vacuum and change to second picking sub mode
                    {
                        printf("Time: %7.2f  Nozzle lowered\n", getSimTime());
                        applyVacuum(CENTRE_NOZZLE);
                        picking_counter++;      // increment to progress to raise nozzle sub mode
                        printf("Time: %7.2f  New state: %.20s  Applying suction\n", getSimTime(), state_name[state]);
                    }

                    else if (isSimulatorReadyForNextInstruction() && picking_counter == 1)  // If suction applied (part picked) then raise nozzle back up, change to final sub mode
                    {
                        printf("Time: %7.2f  Suction applied\n", getSimTime());
                        raiseNozzle(CENTRE_NOZZLE);
                        picking_counter++;
                        printf("Time: %7.2f  Current state: %.20s  Raising nozzle\n", getSimTime(), state_name[state]);
                    }

                     else if (isSimulatorReadyForNextInstruction() && picking_counter == 2)  // Nozzle raised and ready for next command to move to lookup camera, state to 'WAIT_2'
                    {
                        printf("Time: %7.2f  Nozzle raised, Picking complete, Moving to lookup camera\n", getSimTime());
                        picking_counter = 0;    // reset picking_counter for next full cycle
                        setTargetPos(LOOKUP_CAMERA_X, LOOKUP_CAMERA_Y);    // Move to lookup camera position
                        state = MOVE_TO_LOOKUP;
                    }
                    break;

    /********************************   Picking now complete next states are for cameras, alignment and placement    *************************************/

                case MOVE_TO_LOOKUP:
                    if (isSimulatorReadyForNextInstruction())  // Gantry head at lookup camera position ready for photo to check rotation, change state  - 'LOOKUP_PHOTO'
                    {
                        printf("Time: %7.2f  Gantry at Lookup Camera position\n", getSimTime());
                        takePhoto(PHOTO_LOOKUP);    // Take photo to check rotational error for placement correction
                        state = LOOKUP_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Taking Lookup Photo\n", getSimTime(), state_name[state]);
                    }
                    break;

                case LOOKUP_PHOTO:
                    if (isSimulatorReadyForNextInstruction())
                        //  Lookup photo taken, error stored in >>> double theta_pick_error[nozzle] = getPickErrorTheta(nozzle); <<< state to - 'MOVE_TO_NOM_POS'
                    {
                        printf("Time: %7.2f  Lookup photo taken\n", getSimTime());
                        setTargetPos(pi[component_count].x_target, pi[component_count].y_target);    // Move to nominal board position for placement
                        state = MOVE_TO_NOM_POS;
                        printf("Time: %7.2f  New state: %.20s  Moving to nominal board position \n", getSimTime(), state_name[state]);
                    }

                    break;

                case MOVE_TO_NOM_POS:
                    if (isSimulatorReadyForNextInstruction()) // Moved to nom position, change state - 'LOOKDOWN_PHOTO'
                    {
                        printf("Time: %7.2f  Moved to nominal board position\n", getSimTime());
                        takePhoto(PHOTO_LOOKDOWN); // Take lookdown photo to check for translational error in X/Y planes
                        state = LOOKDOWN_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Taking Lookdown photo \n", getSimTime(), state_name[state] );
                    }

                    break;

   /*******************************************   Camera checks finished, in position, awaiting Theta and Translation corrections   *********************************************/

                case LOOKDOWN_PHOTO:
                    if (isSimulatorReadyForNextInstruction()) // Lookdown photo taken Begin rotational alignment
                    {
                        printf("Time: %7.2f  Lookdown photo taken correcting rotational error \n", getSimTime( ));
                        theta_pick_error[CENTRE_NOZZLE] = getPickErrorTheta(CENTRE_NOZZLE);                    // recover error theta value
                        theta_requested = pi[component_count].theta_target-theta_pick_error[CENTRE_NOZZLE];    // adjust nominal value with error
                        rotateNozzle(CENTRE_NOZZLE, theta_requested);                                          // rotate to new requested value
                        state = ROTATIONAL_CORRECT;     // change state
                    }

                    break;

                case ROTATIONAL_CORRECT:
                    if (isSimulatorReadyForNextInstruction()) // Rotation completed now Begin translational alignment
                    {
                        printf("Time: %7.2f  New state: %.20s  Rotation correction completed correcting translational error \n", getSimTime(), state_name[state]);
                        x_preplace_error = getPreplaceErrorX();
                        y_preplace_error = getPreplaceErrorY();
                        amendPos(-x_preplace_error, -y_preplace_error);
                        state = AMEND_POS;               // change state
                    }

                    break;


  /************************************************  Placement of part and return home  *********************************************/
                case AMEND_POS:
                     if (isSimulatorReadyForNextInstruction()) // Translation correction completed, Begin placement
                     {
                        lowerNozzle(CENTRE_NOZZLE);
                        printf("Time: %7.2f  New state: %.20s  Translational correction complete, Beginning placement-Lowering Nozzle\n", getSimTime(), state_name[state]);
                        state = PLACING_PART;

                    }
                    break;

                case PLACING_PART:      // 'placing_counter' increments to progress through 3 placing sub modes (lower/releaseSuction/raise)
                    if (isSimulatorReadyForNextInstruction() && placing_counter == 0)  // If nozzle lowering complete apply vacuum and change to next placing sub mode
                    {
                        printf("Time: %7.2f  Nozzle lowered\n", getSimTime());
                        releaseVacuum(CENTRE_NOZZLE);
                        placing_counter++;              // increments to move to next placing sub mode
                        printf("Time: %7.2f  Current state: %.20s  Releasing suction\n", getSimTime(), state_name[state]);
                    }

                   else if (isSimulatorReadyForNextInstruction() && placing_counter == 1)  // If suction released raise nozzle back up, change to final placing sub mode
                    {
                        printf("Time: %7.2f  Suction released\n", getSimTime());
                        raiseNozzle(CENTRE_NOZZLE);
                        placing_counter++;
                        printf("Time: %7.2f  Current state: %.20s  Raising  nozzle\n", getSimTime(), state_name[state]);
                    }

                     else if (isSimulatorReadyForNextInstruction() && placing_counter == 2)  // Nozzle raised and ready to move to home position
                    {
                        printf("Time: %7.2f  Nozzle raised\n", getSimTime());
                        state = MOVE_TO_HOME;
                        placing_counter = 0;
                        printf("Time: %7.2f  New state: %.20s  Placement complete\n", getSimTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_HOME:
                    if(component_count <  (number_of_components_to_place-1))
                    {
                       component_count++;
                       setTargetPos(HOME_X, HOME_Y);
                       state = HOME;
                       printf("Time: %7.2f  New state: %.20s  Move to home \n", getSimTime(), state_name[state]);
                       printf("Part %d details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               component_count,pi[component_count].component_designation, pi[component_count].component_footprint, pi[component_count].component_value, pi[component_count].x_target, pi[component_count].y_target, pi[component_count].theta_target, pi[component_count].feeder);
                    }
                    else if(component_count ==  (number_of_components_to_place-1))
                    {
                       setTargetPos(HOME_X, HOME_Y);
                       state = HOME;
                       printf("Time: %7.2f  New state: %.20s All %d Parts have been placed, Moving to Home \n", getSimTime(),state_name[state], number_of_components_to_place);
                       finished = TRUE;
                    }

                break;
            }
             sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
    }
        }
    pnpClose();
    return 0;
}

