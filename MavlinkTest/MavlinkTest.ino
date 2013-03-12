// Mavlink test code
#include <FastSerial.h>
#include <BetterStream.h>
#include <ftoa_engine.h>
#include <ntz.h>
#include <xtoa_fast.h>
#include <GCS_MAVLink.h>

// old implementation
//#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
//#include "../mavlink/include/mavlink.h"

// Globals
const int  maxStreams = 6; // number of streams to request
const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RC_CHANNELS,
                                        MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2}; // the streams requested
const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02}; // data rates for the streams
boolean enable_mav_request = false; // keep track of when to request new data
static int packet_drops = 0; // keep track of dropped packets
static uint8_t      apm_mav_system; // system ID
static uint8_t      apm_mav_component; // component ID
FastSerialPort0(Serial); // for printing debug information to serial monitor
FastSerialPort3(Serial3); // connected to the APM


// Initializes the serial ports
void setup() {
	Serial.begin(115200);
	Serial3.begin(57600);
}

void loop() {
        //requestData();
	comm_receive();
        delay(100);
}

// Request data streams from the APM
void requestData(){
     if(enable_mav_request){//Request rate control
        Serial.print("Rquesting data from APM!\n");
        for(int n = 0; n < 3; n++){
            for (int i=0; i < maxStreams; i++) {
              mavlink_msg_request_data_stream_send(MAVLINK_COMM_0, apm_mav_system, apm_mav_component, MAVStreams[i], MAVRates[i], 1);
            } 
            delay(50);
        }
        enable_mav_request = false;
        delay(2000);
        //lastMAVBeat = millis();//Preventing error from delay sensing
    }
}

// Decodes mavlink messsages sent by the APM
void comm_receive() {
	mavlink_message_t msg;
	mavlink_status_t status;

	//receive data over serial
	while(Serial3.available() > 0) {
		uint8_t c = Serial3.read();

		//try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
                                        Serial.print("Received heatbeat packet!\n");
                                        apm_mav_system    = msg.sysid;
					apm_mav_component = msg.compid;
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
                                         enable_mav_request = true;
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
                                case MAVLINK_MSG_ID_VFR_HUD:
                                {
                                  Serial.print("Received VFR Hud!\n");
                                  /*
                                    osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                                    osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                                    osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                                    //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
                                    //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
                                    osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                                    osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                                    */
                                    Serial.print("\tHeading: ");
                                    Serial.print(mavlink_msg_vfr_hud_get_heading(&msg));
                                    Serial.print("\n");
                                }
                                break;
				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
                                        Serial.print("Received GPS packet!\n");
                                        Serial.print("\tLatitude: ");
                                        Serial.print(mavlink_msg_gps_raw_int_get_lat(&msg));
                                        Serial.print("\n");
                                        Serial.print("\tLongitude: ");
                                        Serial.print(mavlink_msg_gps_raw_int_get_lon(&msg));
                                        Serial.print("\n");
                                        Serial.print("\tFix: ");
                                        Serial.print(mavlink_msg_gps_raw_int_get_fix_type(&msg));
                                        Serial.print("\n");
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
                                        Serial.print("\tRoll: ");
                                        Serial.print(ToDeg(mavlink_msg_attitude_get_roll(&msg)));
                                        Serial.print("\n");
                                        Serial.print("\tPitch: ");
                                        Serial.print(ToDeg(mavlink_msg_attitude_get_pitch(&msg)));
                                        Serial.print("\n");
                                        Serial.print("\tYaw: ");
                                        Serial.print(ToDeg(mavlink_msg_attitude_get_yaw(&msg)));
                                        Serial.print("\n");
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
        // update the number of dropped packets
        packet_drops += status.packet_rx_drop_count;
} 

// Attempts to send a heartbeat packet to the APM
void sendHeartbeat(){
  	// Define the system type (see mavlink_types.h for list of possible types)
	int system_type = MAV_TYPE_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
/**
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param system_status System status flag, see MAV_STATE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);

	// Copy the message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	// Send the message (.write sends as bytes)
	Serial3.write(buf, len);
}

