
#include <stdint.h>


typedef struct JointCmd{
  int select;
  int set_point;
  uint8_t wait;
  JointCmd(){select=-1; set_point=-1; wait=0;}
};


#define Q_SIZE 128

/*
Simple circular buffer queue of arm commands

*/
class CommandQueue{
public:
	CommandQueue();
	~CommandQueue();

	int size();

	void enq(JointCmd);
	JointCmd dq();



private:
	int _front; 	//dq pulls out stuff from here
	int _back; 		//New stuff goes here
	JointCmd *_Q;

};

