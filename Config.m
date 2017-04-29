% -----------------------------------------------
% Configuration file for Robotarium experiments -
% -----------------------------------------------
clear all %#ok<CLALL>
global  AGENT_COUNT     TIMEOUT         INFORMED_PRC    TARGET_COUNT    ATT_WEIGHT          ...
        ORI_WEIGHT      GOAL_ATT_WEIGHT VISUAL_DIST     BLINDSPOT       SENSE_GOAL_RAD      ...
        FOUND_GOAL_RAD  GOAL_TOLERANCE  START_TOLERANCE ENABLE_NBOR_VIS ENABLE_RADII_VIS    ...
        ENABLE_ROBOT_LBL                WALL_REP_RAD    SWARM_SPEED
 
AGENT_COUNT         =   8;
TIMEOUT             =   1500;

% Task specific parameters
% RALLY
INFORMED_PRC        =   0.6000;
TARGET_COUNT        =   4;



% Boid parameters
ATT_WEIGHT          =   0.5000;
ORI_WEIGHT          =   0.5000;
GOAL_ATT_WEIGHT     =   0.6000;
VISUAL_DIST         =   0.6708;
BLINDSPOT           =   pi/3;

SENSE_GOAL_RAD      =   0.15;
FOUND_GOAL_RAD      =   0.15;

GOAL_TOLERANCE      =   0.0500;
START_TOLERANCE     =   0.2500;

ENABLE_NBOR_VIS     =   0;
ENABLE_RADII_VIS    =   0;
ENABLE_ROBOT_LBL    =   0;

WALL_REP_RAD        =   0.1250;
SWARM_SPEED         =   0.0500;