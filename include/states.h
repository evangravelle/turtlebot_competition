#define 	INIT 						0
#define 	MANUAL						1
#define 	START						3
#define 	END							4

#define 	CONFIG						20
#define 		CONFIG_GOAL_COLOR		21	
#define 		CONFIG_CEILING			22
#define 		CONFIG_BALL_COLOR		23

#define 	FIND_GOAL					40
#define         SEARCH_FOR_GOAL         41
#define         GOAL_FOUND              42

#define 	MOVE_TO_GOAL				50
#define         MOVING_TO_GOAL          51
#define         AT_GOAL                 52
#define         MOVE_TO_GOAL_FAILED     53

#define 	FIND_BALL					60
#define         SEARCH_FOR_BALL         61
#define         BALL_FOUND              62
#define         GREEN_BALL_FOUND        63
#define         ORANGE_BALL_FOUND       64

#define 	MOVE_TO_BALL				70
#define         MOVING_TO_GREEN         71
#define         AT_GREEN                72
#define         MOVE_TO_GREEN_FAILED    73
#define         CENTER_ON_GREEN         74
#define         MOVING_TO_ORANGE        75
#define         AT_ORANGE               76
#define         MOVE_TO_ORANGE_FAILED   77
#define         CENTER_ON_ORANGE        78
#define         MOVE_TO_BALL_FAILED     79
#define         CENTER_ON_BALL          170
#define         MOVING_TO_BALL          171
#define         AT_BALL                 172

#define 	PICK_UP_BALL				80	
#define			ATTEMPT_PICK_UP_GREEN	81
#define			CHECK_GREEN				82
#define			ATTEMPT_PICK_UP_ORANGE	83
#define			CHECK_ORANGE			84
#define         GOT_BALL                85 
#define         GOT_BALL_FAILED         86 
#define         ATTEMPT_PICK_UP         87 
#define         CHECK_BALL              88 

#define 	DROP_BALL					90
#define 	    BALL_DROPPED			91
#define 	    DROP_BALL_FAILED		92

#define     DEFAULT_SUB_STATE           100
