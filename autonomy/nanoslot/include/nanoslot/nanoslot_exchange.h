/*
 Plain-old-bytes structs used to exchange data with the Arduino nanoslots.
 These structs are sent on the serial connection between PC and Arduino,
 connecting slot_ID program swith the firmware_ID Arduino programs.
 
 Structs:
    - command: raw data sent to Arduino, like the autonomy mode and motor power.
    - sensor: raw data sent from Arduino, like heartbeat and raw encoder counts.
    - state: parsed data about the machine state, like connected data and encoder counts with wraparound.
    - debug: debug data received from Arduino.

 Command and sensor data is sent over serial to/from the Arduino, so it must be compact and match Arduino struct layout byte for byte.

 State and debug data is only used on the PC side and is hence less size critical.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-23 (Public Domain)
*/
#ifndef NANOSLOT_EXCHANGE_H
#define NANOSLOT_EXCHANGE_H 1

/* Datatypes used */
typedef uint8_t nanoslot_byte_t; ///< generic data byte
typedef uint8_t nanoslot_heartbeat_t; ///< heartbeat (watchdog-type counter)
typedef int8_t nanoslot_motorpercent_t; ///< -100 for full reverse, 0 for stop, +100 for full forward
typedef int8_t nanoslot_padding_t[3]; ///< padding to avoid false sharing between slots

// Packed bitfield struct holding 10-bit XYZ values.
//   Used for gyro and accelerometer data.  Will be padded to 4-byte alignment.
struct nanoslot_xyz10_t {
	int32_t x:10;
	int32_t y:10;
	int32_t z:10;
	uint32_t type:2; // Round out to 32 bits with scaling/valid flag
	enum {
	    type_1x = 0, // scale 1x
	    type_2x = 1, // scale 2x
	    type_4x = 2, // scale 4x
	    type_invalid = 3
	};
	
	void invalidate(void) {
	    x=y=z=0;
	    type=type_invalid;
	}
	bool valid(void) {
	    return type!=type_invalid;
	}
	
#if _STDIO_H
    void print(const char *name) {
        printf("%s %4d %4d %4d (%d) ",
            name,x,y,z,type);
    }
#endif
};

// 3D vector type
typedef nanoslot_xyz10_t nanoslot_vec3_t;

// Inertial measurement unit (IMU) data
//  Will be padded to 4-byte alignment.
struct nanoslot_IMU_t {
    nanoslot_vec3_t acc; /// Accelerometer down vector (gravity)
    nanoslot_vec3_t gyro; /// Gyro rotation rates
    
    void invalidate(void) {
        acc.invalidate(); gyro.invalidate();
    }
};

/** Generic firmware state */
struct nanoslot_state {
    nanoslot_byte_t connected; // 0 if not connected, 1 if connected
};


/** Info about autonomous operation shared with all firmware */
struct nanoslot_autonomy {
    /**
        Autonomous operation mode:
          mode==0 is STOP, safe mode, all actuators off.
          mode==1 or 2 is manual driving
          mode>2 is autonomous driving
    */
    nanoslot_byte_t mode;
};


/* 
 General per-slot format:
    NANOSLOT_MY_ID is my hex ID, with leading "0x".
 nanoslot_command_<ID> is the command data sent by the PC to the Arduino.
 nanoslot_sensor_<ID> is the sensor data reported back by the Arduino.
*/

/** slot ID 0xA0: arm motor controllers (in arm electronics box) */
struct nanoslot_command_0xA0 {
    nanoslot_autonomy autonomy; 
    
    enum {n_motors=4};
    nanoslot_motorpercent_t motor[n_motors]; // brushed DC linear actuator motors
};
struct nanoslot_sensor_0xA0 {
    nanoslot_heartbeat_t heartbeat; // increments
    nanoslot_byte_t stop; // 1 == stop requested
};
struct nanoslot_state_0xA0 : public nanoslot_state {
    
};

/** slot ID 0xA1: arm IMUs (in arm electronics box) */
struct nanoslot_command_0xA1 {
    nanoslot_autonomy autonomy; 
};
struct nanoslot_sensor_0xA1 {
    // IMU needs to be listed first in the struct, for alignment
    enum {n_imu=2}; 
    enum {imu_tool=0};
    enum {imu_stick=1};
    nanoslot_IMU_t imu[n_imu];
    
    // Single-byte fields go after IMU data
    nanoslot_heartbeat_t heartbeat; // increments
    // need a multiple of 4 bytes for Arduino and PC to agree on struct padding
    nanoslot_byte_t spare[3];
};
struct nanoslot_state_0xA1 : public nanoslot_state {
    // FIXME: filtered IMU values (smoothed, vibration estimate, de-vertigo etc)
};


/** slot ID 0xD0: drive motor controllers (in big back box) */
struct nanoslot_command_0xD0 {
    nanoslot_autonomy autonomy; 
    enum {n_motors=4};
    nanoslot_motorpercent_t motor[n_motors]; // brushed DC drive motors
};
struct nanoslot_sensor_0xD0 {
    nanoslot_heartbeat_t heartbeat;
    nanoslot_byte_t raw; // raw bit version of sensors
    nanoslot_byte_t stall; // raw bit version of sensors
    enum {n_sensors=2};
    nanoslot_byte_t counts[n_sensors]; // counts for each sensor channel
};
struct nanoslot_state_0xD0 : public nanoslot_state {
    
};

/** slot ID 0xF0: forward motor controllers (in big back box) */
struct nanoslot_command_0xF0 {
    nanoslot_autonomy autonomy; 
    
    enum {n_motors=4};
    nanoslot_motorpercent_t motor[n_motors]; // brushed DC linear actuator motors
};
struct nanoslot_sensor_0xF0 {
    nanoslot_heartbeat_t heartbeat; // increments
    nanoslot_byte_t stop; // 1 == stop requested
};
struct nanoslot_state_0xF0 : public nanoslot_state {
    
};


/** slot ID 0xF1: forward IMUs (in dedicated mini box in bottom front of robot) */
struct nanoslot_command_0xF1 {
    nanoslot_autonomy autonomy; 
};
struct nanoslot_sensor_0xF1 {
    // IMU needs to be listed first in the struct, for alignment
    enum {n_imu=4};
    enum {imu_frame=0};
    enum {imu_boom=1};
    enum {imu_fork=2};
    enum {imu_dump=3};
    nanoslot_IMU_t imu[n_imu];
    
    // Single-byte fields go after IMU data
    nanoslot_heartbeat_t heartbeat; // increments
    // need a multiple of 4 bytes for Arduino and PC to agree on struct padding
    nanoslot_byte_t spare[3];
};
struct nanoslot_state_0xF1 : public nanoslot_state {
    // FIXME: filtered IMU values (smoothed, vibration estimate, de-vertigo etc)
};


/** slot ID 0xEE: example nano (debug / dev only) */
struct nanoslot_command_0xEE {
    nanoslot_autonomy autonomy; 
    nanoslot_motorpercent_t LED; // pin 13 debug
};
struct nanoslot_sensor_0xEE {
    nanoslot_heartbeat_t heartbeat;
    nanoslot_byte_t latency;
};
struct nanoslot_state_0xEE : public nanoslot_state {
    
};

/** Debug data kept per slot */
struct nanoslot_debug_t {
    nanoslot_byte_t flags; // 0: no extra debug info.  Bits request various debug features (TBD)
    nanoslot_byte_t packet_count; // serial packets recv'd (like a heartbeat)
    
    
};

/** Each slot keeps this data on the exchange.
    The idea is we can send commands like nano.slot_A0.command.motor[1]=100;
*/
template <typename command_t, typename sensor_t, typename state_t>
struct nanoslot_exchange_slot 
{
    command_t command; ///< Commands to send to Arduino
    sensor_t sensor; ///< Sensor data received back from Arduino
    state_t state; ///< Persistent state data
    nanoslot_debug_t debug; ///< Debug data
};


/** One struct with all nano slot data, 
   for example to live in the data exchange, 
   or for logging & debugging. */
struct nanoslot_exchange {
    uint16_t size; // size, in bytes, of this struct (exit early if mismatch here)
    void sanity_check_size(void);
    
    // The backend increments this every time it writes commands
    nanoslot_heartbeat_t backend_heartbeat;
    // Autonomy mode is shared by all slots.  This value is published by the backend.
    nanoslot_autonomy autonomy;
    
    nanoslot_padding_t pad_0; ///<- padding prevents false sharing slowdown
    
    // Each slot stores its data here:
    nanoslot_exchange_slot<nanoslot_command_0xA0, nanoslot_sensor_0xA0, nanoslot_state_0xA0> slot_A0;
    nanoslot_padding_t pad_A0;
    
    nanoslot_exchange_slot<nanoslot_command_0xA1, nanoslot_sensor_0xA1, nanoslot_state_0xA1> slot_A1;
    nanoslot_padding_t pad_A1;
    
    nanoslot_exchange_slot<nanoslot_command_0xD0, nanoslot_sensor_0xD0, nanoslot_state_0xD0> slot_D0;
    nanoslot_padding_t pad_D0;
    
    nanoslot_exchange_slot<nanoslot_command_0xF0, nanoslot_sensor_0xF0, nanoslot_state_0xF0> slot_F0;
    nanoslot_padding_t pad_F0;
    
    nanoslot_exchange_slot<nanoslot_command_0xF1, nanoslot_sensor_0xF1, nanoslot_state_0xF1> slot_F1;
    nanoslot_padding_t pad_F1;
    
    nanoslot_exchange_slot<nanoslot_command_0xEE, nanoslot_sensor_0xEE, nanoslot_state_0xEE> slot_EE;
    nanoslot_padding_t pad_EE;
};


#ifdef NANOSLOT_MY_ID

#define NANOSLOT_TOKENPASTE(a,b) a##b
#define NANOSLOT_TOKENPASTE2(a,b) NANOSLOT_TOKENPASTE(a,b)
#define NANOSLOT_COMMAND_MY  NANOSLOT_TOKENPASTE2(nanoslot_command_,NANOSLOT_MY_ID)
#define NANOSLOT_SENSOR_MY  NANOSLOT_TOKENPASTE2(nanoslot_sensor_,NANOSLOT_MY_ID)
#define NANOSLOT_STATE_MY  NANOSLOT_TOKENPASTE2(nanoslot_state_,NANOSLOT_MY_ID)

#endif

#endif

