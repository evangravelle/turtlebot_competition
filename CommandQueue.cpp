#include "CommandQueue.h"

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
	if(size() >= Q_SIZE)
		return;
	_Q[_back] = newcmd;
	_back = (_back + 1) % Q_SIZE;
}

JointCmd CommandQueue::dq(){
	JointCmd r;
	if(size()==0)
		return r;
	r = _Q[_front];
	_front = (_front + 1) % Q_SIZE;
	return r;
}

