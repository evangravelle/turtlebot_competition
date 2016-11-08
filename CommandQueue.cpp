
#include "Arduino.h"
#include "CommandQueue.h"

// #define VERBOSE

CommandQueue::CommandQueue(){
	_Q = new JointCmd[Q_SIZE];
	_front = 0;
	_back = 0;
}

CommandQueue::~CommandQueue(){
	delete[] _Q;
}

int CommandQueue::size(){
	int s = _back - _front;
	s = (s + Q_SIZE) % Q_SIZE;
	return s;
}

void CommandQueue::enq(JointCmd newcmd){
	if(size() >= Q_SIZE){

#ifdef VERBOSE
		Serial.println("Queue is full");
#endif

		return;
	}
	_Q[_back] = newcmd;

#ifdef VERBOSE
	Serial.print("ENQ : ");
	Serial.print(_Q[_back].select);
	Serial.print(" 		");
	Serial.print(_Q[_back].set_point);
	Serial.print(" 		");
	Serial.println(_Q[_back].wait);
#endif	


	_back = (_back + 1) % Q_SIZE;
}


JointCmd CommandQueue::dq(){
	JointCmd r;
	if(size()==0)
		return r;
	r = _Q[_front];
	_front = (_front + 1) % Q_SIZE;

#ifdef VERBOSE
	Serial.print("DQ : ");
	Serial.print(r.select);
	Serial.print("		");
	Serial.print(r.set_point);
	Serial.print("		");
	Serial.println(r.wait);
#endif

	return r;
}



