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

#define 	MOVE_TO_BALL				70
#define         MOVING_TO_BALL          71
#define         AT_BALL                 72
#define         MOVE_TO_BALL_FAILED     73

#define 	PICK_UP_BALL				80	
#define         GOT_BALL                81 
#define         LOST_BALL               82 
#define         GOT_BALL_FAILED         83 

#define 	DROP_BALL					90
#define 	    BALL_DROPPED			91
#define 	    DROP_BALL_FAILED		92

#define     DEFAULT_SUB_STATE           100
