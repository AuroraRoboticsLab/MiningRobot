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

#include "nanoslot_IMU.h"

/* Datatypes used */
typedef uint8_t nanoslot_byte_t; ///< generic data byte
typedef uint8_t nanoslot_heartbeat_t; ///< heartbeat (watchdog-type counter)
typedef int8_t nanoslot_motorpercent_t; ///< -100 for full reverse, 0 for stop, +100 for full forward
typedef int16_t nanoslot_voltage_t; ///< Arduino A/D voltage reading
typedef int16_t nanoslot_actuator_angle_t; // 1/4096 angle reading
typedef uint8_t nanoslot_counter_t; ///< Counter, like an encoder
typedef int8_t nanoslot_padding_t[7]; ///< padding to avoid false sharing between slots

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

/** slot ID 0x7x: slender arm motor controllers on each actuator */
struct nanoslot_command_0x70 {
    nanoslot_autonomy autonomy; 
    
    enum {n_motors=1};
    nanoslot_motorpercent_t torque[n_motors]; // brushless motor power, torque control
    nanoslot_actuator_angle_t target[n_motors]; // angle control (autonomous modes)
};
struct nanoslot_sensor_0x70 {
    nanoslot_heartbeat_t heartbeat; // increments when connected

    nanoslot_byte_t mag[1]; // magnet strength 
    nanoslot_actuator_angle_t angle[1]; // read-back angle
};
struct nanoslot_state_0x70 : public nanoslot_state {
    float angle[1]; // read-back angle, in degrees
};

/* 
 General per-slot format:
    NANOSLOT_MY_ID is my hex ID, with leading "0x".
 nanoslot_command_<ID> is the command data sent by the PC to the Arduino.
 nanoslot_sensor_<ID> is the sensor data reported back by the Arduino.
*/

/** slot ID 0xA0: wide permanent arm motor controllers (in arm electronics box) */
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
    nanoslot_byte_t read_L; // if 1, read from left load cell channel
};
struct nanoslot_sensor_0xA1 {
    // IMU needs to be listed first in the struct, for alignment
    enum {n_imu=2}; 
    enum {imu_tool=0};
    enum {imu_stick=1};
    nanoslot_IMU_t imu[n_imu];
    
    // Load cell left and right (default) values
    int32_t load_L, load_R;
    
    // Single-byte fields go after IMU data
    nanoslot_heartbeat_t heartbeat; // increments
    // need a multiple of 4 bytes for Arduino and PC to agree on struct padding
    nanoslot_byte_t spare[3];
};
struct nanoslot_state_0xA1 : public nanoslot_state {
    nanoslot_IMU_state stick; ///< Arm stick frame
    nanoslot_IMU_state tool; ///< Tool coupler (tilt + spin)
    
    // Load cell kilogram-force, negative = down
    float load_L, load_R;
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
    nanoslot_voltage_t cell1; ///< first cell of drive battery pack
};
struct nanoslot_state_0xF0 : public nanoslot_state {
    float cell; ///< voltage (V) on drive battery's first cell
    float charge; ///< Estimated percent charge for battery, normally between 20 and 80
    
};


/** slot ID 0xF1: forward IMUs (in dedicated mini box in bottom front of robot) */
struct nanoslot_command_0xF1 {
    nanoslot_autonomy autonomy; 
    nanoslot_byte_t read_L; // if 1, read from left load cell channel
};
struct nanoslot_sensor_0xF1 {
    // IMU needs to be listed first in the struct, for alignment
    enum {n_imu=4};
    enum {imu_frame=0};
    enum {imu_boom=1};
    enum {imu_fork=2};
    enum {imu_dump=3};
    nanoslot_IMU_t imu[n_imu];
    
    // Load cell left and right (default) values
    int32_t load_L, load_R;
    
    // Single-byte fields go after IMU data
    nanoslot_heartbeat_t heartbeat; // increments
    // need a multiple of 4 bytes for Arduino and PC to agree on struct padding
    nanoslot_byte_t spare[3];
};

struct nanoslot_state_0xF1 : public nanoslot_state {
    nanoslot_IMU_state frame; ///< Drive frame
    nanoslot_IMU_state boom; ///< Robot arm boom
    nanoslot_IMU_state fork; ///< Front scoop fork
    nanoslot_IMU_state dump; ///< Front scoop dump
    
    // Load cell kilogram-force, negative = down
    float load_L, load_R;
};


/** slot ID 0xC0: cutter in rockgrinder head (tool, pluggable) */
struct nanoslot_command_0xC0 {
    nanoslot_autonomy autonomy; 
    nanoslot_motorpercent_t mine; // run mining head
};
struct nanoslot_sensor_0xC0 {
    nanoslot_heartbeat_t heartbeat;
    nanoslot_counter_t spincount; ///< mining head spin count
    nanoslot_voltage_t cell0; ///< ground of battery pack
    nanoslot_voltage_t cell1; ///< first cell of rockgrinder battery pack
};
struct nanoslot_state_0xC0 : public nanoslot_state {
    float spin; ///< Last spin count per second
    float load; ///< scaled from voltage delta on ground line (always zero?)
    float cell; ///< voltage (V) on mine battery's first cell
    float charge; ///< Estimated percent charge for battery, normally between 20 and 80
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
    
    nanoslot_padding_t pad; ///<- padding prevents false sharing slowdown (separate programs on separate cores may be updating each slot's data)
};

/* These typedefs relate the slot structs above to the slot ID */
typedef nanoslot_exchange_slot<nanoslot_command_0x70, nanoslot_sensor_0x70, nanoslot_state_0x70> nanoslot_slot_0x70;
typedef nanoslot_exchange_slot<nanoslot_command_0x70, nanoslot_sensor_0x70, nanoslot_state_0x70> nanoslot_slot_0x71;
typedef nanoslot_exchange_slot<nanoslot_command_0x70, nanoslot_sensor_0x70, nanoslot_state_0x70> nanoslot_slot_0x72;
typedef nanoslot_exchange_slot<nanoslot_command_0x70, nanoslot_sensor_0x70, nanoslot_state_0x70> nanoslot_slot_0x73;

typedef nanoslot_exchange_slot<nanoslot_command_0xA0, nanoslot_sensor_0xA0, nanoslot_state_0xA0> nanoslot_slot_0xA0;
typedef nanoslot_exchange_slot<nanoslot_command_0xA1, nanoslot_sensor_0xA1, nanoslot_state_0xA1> nanoslot_slot_0xA1;

typedef nanoslot_exchange_slot<nanoslot_command_0xC0, nanoslot_sensor_0xC0, nanoslot_state_0xC0> nanoslot_slot_0xC0;

typedef nanoslot_exchange_slot<nanoslot_command_0xD0, nanoslot_sensor_0xD0, nanoslot_state_0xD0> nanoslot_slot_0xD0;

typedef nanoslot_exchange_slot<nanoslot_command_0xF0, nanoslot_sensor_0xF0, nanoslot_state_0xF0> nanoslot_slot_0xF0;
typedef nanoslot_exchange_slot<nanoslot_command_0xF1, nanoslot_sensor_0xF1, nanoslot_state_0xF1> nanoslot_slot_0xF1;

typedef nanoslot_exchange_slot<nanoslot_command_0xEE, nanoslot_sensor_0xEE, nanoslot_state_0xEE> nanoslot_slot_0xEE;

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
    nanoslot_slot_0x70 slot_70;
    nanoslot_slot_0x71 slot_71;
    nanoslot_slot_0x72 slot_72;
    nanoslot_slot_0x73 slot_73;
    
    nanoslot_slot_0xA0 slot_A0;
    nanoslot_slot_0xA1 slot_A1;
    
    nanoslot_slot_0xC0 slot_C0;
    
    nanoslot_slot_0xD0 slot_D0;
    
    nanoslot_slot_0xF0 slot_F0;
    nanoslot_slot_0xF1 slot_F1;
    
    nanoslot_slot_0xEE slot_EE;
};


#ifdef NANOSLOT_MY_ID
/* Used by slot programs, with a defined hex value */

#define NANOSLOT_TOKENPASTE(a,b) a##b
#define NANOSLOT_TOKENPASTE2(a,b) NANOSLOT_TOKENPASTE(a,b)
#define NANOSLOT_SLOT_FROM_ID() NANOSLOT_TOKENPASTE2(nanoslot_slot_,NANOSLOT_MY_ID)
#define NANOSLOT_COMMAND_MY decltype(((NANOSLOT_SLOT_FROM_ID() *)0)->command)
#define NANOSLOT_SENSOR_MY  decltype(((NANOSLOT_SLOT_FROM_ID() *)0)->sensor)
#define NANOSLOT_STATE_MY   decltype(((NANOSLOT_SLOT_FROM_ID() *)0)->state)

#endif

#endif

