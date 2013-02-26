// Arduino MAVLink test code.

#include <FastSerial.h>
#include "../mavlink/include/mavlink.h"        // Mavlink interface


FastSerialPort0(Serial);
FastSerialPort2(Serial2);

void setup() {
	Serial.begin(115200);
	Serial2.begin(57600);
}

void loop() {
	// Define the system type (see mavlink_types.h for list of possible types)
	int system_type = MAV_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type);

	// Copy the message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	// Send the message (.write sends as bytes)
	Serial2.write(buf, len);
	comm_receive();
}

void comm_receive() {
	mavlink_message_t msg;
	mavlink_status_t status;

	//receive data over serial
	while(Serial2.available() > 0) {
		uint8_t c = Serial2.read();

		//try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
                                        Serial.print("Received heatbeat packet!\n");
				/*
					mavbeat = 1;
					apm_mav_system    = msg.sysid;
					apm_mav_component = msg.compid;
					apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);
					lastMAVBeat = millis();
					if(waitingMAVBeats == 1){
					  enable_mav_request = 1;
					}
				*/
				}
				break;
				case MAVLINK_MSG_ID_SYS_STATUS:
				{
                                        Serial.print("Received status packet!\n");
				/*
					osd_vbat = (mavlink_msg_sys_status_get_vbat(&msg) / 1000.0f);
					osd_mode = mavlink_msg_sys_status_get_mode(&msg);
					osd_nav_mode = mavlink_msg_sys_status_get_nav_mode(&msg);
					osd_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
					//osd_mode = apm_mav_component;//Debug
					//osd_nav_mode = apm_mav_system;//Debug
				*/
				}
				break;
				case MAVLINK_MSG_ID_GPS_RAW:
				{
                                        Serial.print("Received GPS packet!\n");
				/*
					osd_lat = mavlink_msg_gps_raw_get_lat(&msg);
					osd_lon = mavlink_msg_gps_raw_get_lon(&msg);
					//osd_alt = mavlink_msg_gps_raw_get_alt(&msg);
					osd_fix_type = mavlink_msg_gps_raw_get_fix_type(&msg);
				*/
				}
				break;
				case MAVLINK_MSG_ID_GPS_STATUS:
				{
                                        Serial.print("Received GPS status packet!\n");
				/*
					osd_satellites_visible = mavlink_msg_gps_status_get_satellites_visible(&msg);
				*/
				}
				break;
				case MAVLINK_MSG_ID_ATTITUDE:
				{
                                        Serial.print("Received attitude packet!\n");
				/*
					osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
					osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
					osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
				*/
				}
				break;
				default:
				  //Do nothing
				break;
			}
		}
		// read until the serial buffer is empty
	}
} 
