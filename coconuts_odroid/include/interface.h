#include <serial/serial.h>

// Packets sizes for POT status
#define OI_TRIM_POT_0_SIZE						26
#define OI_TRIM_POT_1_SIZE						26
#define OI_TRIM_POT_2_SIZE						26
#define OI_TRIM_POT_3_SIZE						26
#define OI_TRIM_POT_4_SIZE						26

// TRIM POT Value range?

// Motors and Constraints
#define OI_MOTOR_0							0
#define OI_MOTOR_0_MIN							-255
#define OI_MOTOR_0_MAX							255
#define OI_MOTOR_1							1
#define OI_MOTOR_1_MIN							-255
#define OI_MOTOR_1_MAX							255
#define OI_MOTOR_2							2
#define OI_MOTOR_2_MIN							-255
#define OI_MOTOR_2_MAX							255
#define OI_MOTOR_3							3
#define OI_MOTOR_3_MIN							-255
#define OI_MOTOR_3_MAX							255
#define OI_MOTOR_4							4
#define OI_MOTOR_4_MIN							-255
#define OI_MOTOR_4_MAX							255



#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

namespace interface
{
	//! TRIM POT IO packet id -- may be overkill
	typedef enum _OI_Packet_ID {
	
		// Sensor Packets
		OI_PACKET_GROUP_0 = 0,			//! OI packets 7-26
		OI_PACKET_GROUP_1 = 1,			//! OI packets 7-16
		OI_PACKET_GROUP_2 = 2,			//! OI packets 17-20
		OI_PACKET_GROUP_3 = 3,			//! OI packets 21-26
		OI_PACKET_GROUP_4 = 4,			//! OI packets 27-34
		OI_PACKET_GROUP_5 = 5,			//! OI packets 35-42

	} OI_Packet_ID;

	class Interface
	{
		public:
	
		//! Constructor
		OpenInterface(const char * new_serial_port);

		~OpenInterface();
	
		int openSerialPort();
		int startOI();
		int closeSerialPort();
	
		int initialize();

		int getTrimPotStatus(int timeout);
		
		

		private:

		// Send voltage to a motor
		int moveMotor(int motor, float voltage);	

 		//! Serial port to which the robot is connected
		std::string port_name_;
		//! Serial port object
		serial::Serial serial_port_;
	
	};

}

// EOF
