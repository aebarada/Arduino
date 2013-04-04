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
int DegreeFinder[8][4];
float lat = 0;
float lon = 0;
float z = 0;
float yaw = 0;
int degree = 0;
int check[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// Initializes the serial ports
void setup() {
	Serial.begin(115200);
	Serial3.begin(57600);

        pinMode(49, INPUT); //B4
        pinMode(48, INPUT); //B3
        pinMode(51, INPUT); //B2

        pinMode(46, INPUT); //Q2
        pinMode(47, INPUT); //Q3
        pinMode(44, INPUT); //Q4
        pinMode(45, INPUT); //Q5
  
        //RSSI
        pinMode(53 , INPUT); //Digital Output; Not Used
        BuildDegree();
}

void loop() {
        requestData();
	comm_receive();
        getDegree();
        doMath();
        checkCollision();
        setWaypoint();
        //delay(100);
}

void checkCollision(){
      if(degree > 315 && degree < 45){
        // check sensors on front arm
      } else if(degree > 45 && degree < 135){
        // check sensors on right arm
      } else if(degree > 135 && degree < 225){
        // check sensors on back arm
      } else {
        //check sensor on left arm
      }
}

void getDegree(){ 
    int Y = 0;
    int Q = 0;
    int greatest = 0;
    memset(check, 0, sizeof(check)/sizeof(check[0]));
    for(int i =0; i<=5; i++){
       Y = findY();
       Q = findQ();
       check[DegreeFinder[Y][Q]]++;
    }
    for(int i = 0; i <=32; i++){
        if(check[i] > greatest){
           greatest = check[i]; 
        }
    }
    degree = greatest;
}

void doMath(){
     int dir = yaw + degree;
     float dx = 1*cos(dir); // needs to be changed from a 1 becuase 1 in lat/lon is a very large distance
     float dy = 1*sin(dir);
	 float delta_lon = dx/(111320*cos(lat))
	 float delta_lat = dy/110540
	 lat = lat + delta_lat;
	 lon = lon + delta_lon;
}

void setWaypoint(){
/**
 * @brief Send a mission_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 mavlink_msg_mission_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
 */
      mavlink_msg_mission_item_send(MAVLINK_COMM_0,
      apm_mav_system,
      apm_mav_component,
      0, // sequence number of waypoint
      MAV_FRAME_GLOBAL, // frame
      MAV_CMD_NAV_WAYPOINT, // command
      1, // current waypoint
      0, // autocontinue
      1, // param1
      1000, // param2
      0, // param3
      0, // param4
      lat,
      lon,
      z);
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
                                    yaw = mavlink_msg_vfr_hud_get_heading(&msg);
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
                                        lat = mavlink_msg_gps_raw_int_get_lat(&msg);
                                        lon = mavlink_msg_gps_raw_int_get_lon(&msg);
                                        z = mavlink_msg_gps_raw_int_get_alt(&msg);
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

//Finds the Y part to find degree

int findY(){

  int A2 = digitalRead(49); //B4
  int A1 = digitalRead(48); //B3
  int A0 = digitalRead(51); //B2
  
  Serial.print(A2); //B4
  Serial.print(A1); //B3
  Serial.print(A0); //B2
  Serial.println();
  //Serial.print((A2 << 2) | (A1 << 1) | A0);
  
  return (A2 << 2) | (A1 << 1) | A0;
}

//Finds the Q for finding degree

int findQ(){
  int x = 0, q0 =0, q1 = 0, q2 = 0, q3 = 0;
  q0 = digitalRead(46); //Q2
  q1 = digitalRead(47); //Q3
  q2 = digitalRead(44); //Q4
  q3 = digitalRead(45); //Q5
  
  if(q0){
    x = 0;
  }
  else if(q1){
    x = 1;
  }
  else if(q2){
    x = 2;
  }
  else if(q3){
    x = 3;
  }
  Serial.print(x);
  Serial.println();
   return x;
}

//Builds 2 dimensional array holding degree values

void BuildDegree(){
 
 DegreeFinder[0][0] = 0;
 DegreeFinder[0][1] = 90;
 DegreeFinder[0][2] = 180;
 DegreeFinder[0][3] = 270;
 DegreeFinder[1][0] = 11;
 DegreeFinder[1][1] = 101;
 DegreeFinder[1][2] = 191;
 DegreeFinder[1][3] = 281;
 DegreeFinder[2][0] = 22;
 DegreeFinder[2][1] = 112;
 DegreeFinder[2][2] = 202;
 DegreeFinder[2][3] = 292;
 DegreeFinder[3][0] = 34;
 DegreeFinder[3][1] = 124;
 DegreeFinder[3][2] = 214;
 DegreeFinder[3][3] = 304;
 DegreeFinder[4][0] = 45;
 DegreeFinder[4][1] = 135;
 DegreeFinder[4][2] = 225;
 DegreeFinder[4][3] = 315;
 DegreeFinder[5][0] = 56;
 DegreeFinder[5][1] = 146;
 DegreeFinder[5][2] = 236;
 DegreeFinder[5][3] = 326;
 DegreeFinder[6][0] = 67;
 DegreeFinder[6][1] = 157;
 DegreeFinder[6][2] = 247;
 DegreeFinder[6][3] = 337;
 DegreeFinder[7][0] = 79;
 DegreeFinder[7][1] = 169;
 DegreeFinder[7][2] = 259;
 DegreeFinder[7][3] = 349;

}
