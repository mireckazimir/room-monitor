/**
 * @file main.cpp
 * @author Miroslav Kazimir (xkazim00@fit.vutbr.cz)
 * @brief Room monitor main service
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022 Miroslav Kazimir
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include "DFRobot_VEML7700.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "MCP342x.h"
#include <mosquitto.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <time.h>
#include <pigpio.h>

#define MOTION_SENSOR_GPIO 23
#define DUST_SENSOR_GPIO 24

DFRobot_VEML7700 als;
uint8_t mcp3422_address = 0x6E;
MCP342x adc = MCP342x(mcp3422_address);
int fd_i2c;
struct mosquitto *mosq = NULL;

// Timestamp in milliseconds
uint64_t millis()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    uint64_t ms = (((uint64_t)ts.tv_sec)*1000) + (((uint64_t)ts.tv_nsec)/1000000);
    return ms;
}

// Mosquitto error logging
void mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    switch(level){
        case MOSQ_LOG_WARNING:
        case MOSQ_LOG_ERR: {
        printf("%i:%s\n", level, str);
        }
    }
}

// MQTT initial setup
void mqtt_setup(){

	char *host = "test.mosquitto.org";
	int port = 1883;
	int keepalive = 60;
	bool clean_session = true;
  
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, clean_session, NULL);
    if(!mosq){
        fprintf(stderr, "Error: Out of memory.\n");
        exit(1);
    }
    
    mosquitto_log_callback_set(mosq, mosq_log_callback);
    
    if(mosquitto_connect(mosq, host, port, keepalive)){
        fprintf(stderr, "Unable to connect.\n");
        exit(1);
    }
    int loop = mosquitto_loop_start(mosq);
    if(loop != MOSQ_ERR_SUCCESS){
        fprintf(stderr, "Unable to start loop: %i\n", loop);
        exit(1);
    }
}

// Send MQTT message
int mqtt_send(std::string topic, std::string msg){
    return mosquitto_publish(mosq, NULL, topic.c_str(), msg.size(), msg.c_str(), 0, 0);
}

// MQ-2 gas / smoke measurement
void mq2_measurement(){
    long value = 0;
    MCP342x::Config status;

    // Initiate a conversion; convertAndRead() will wait until it can be read
    uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
				   MCP342x::resolution12, MCP342x::gain1,
				   1000000, value, status);

    if(err){
        printf("MQ-2 measurement MCP3422 convert error\n");
    }

    // MQ-2 A/D output to voltage conversion
    float mq2 = (((float) value) / 2047.0) * 5.0;

    // Send MQ-2 MQTT message
    char buff[100];
    sprintf(buff, "%.3f", mq2);
    std::string msg = buff;
    int snd = mqtt_send("room_monitor/mq2", msg);
    if(snd != 0) printf("mqtt_send error=%i\n", snd);
}

// Dust measurement
void dust_measurement(){
    long value = 0;
    MCP342x::Config status;

    gpioWrite(DUST_SENSOR_GPIO, 1); // Set infrared LED on
    usleep(280); // wait 0,28ms

    // Initiate a conversion; convertAndRead() will wait until it can be read
    uint8_t err = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
				   MCP342x::resolution12, MCP342x::gain1,
				   1000000, value, status);
    if(err){
        printf("Dust measurement MCP3422 convert error\n");
    }

    gpioWrite(DUST_SENSOR_GPIO, 0);// Set infrared LED off

    // Dust sensor A/D output to voltage conversion
    float dust_voltage = (((float) value) / 2047.0) * 5.0;

    dust_voltage *= 11.0; // voltage divider

    float density = (dust_voltage - 0.4) * 2.0;

    // Send dust sensor MQTT message
    char buff[100];
    sprintf(buff, "%.2f", density);
    std::string msg = buff;
    int snd = mqtt_send("room_monitor/dust", msg);
    if(snd != 0) printf("mqtt_send error=%i\n", snd);
}

// Light measurement
void VEML7700_measurement(){
    float lux;

    // Measure ALS Lux with automatic gain and time adjustment
    als.getAutoALSLux(lux);

    // Send MQ-2 MQTT message
    char buff[100];
    sprintf(buff, "%.5f", lux);
    std::string msg = buff;
    int snd = mqtt_send("room_monitor/light", msg);
    if(snd != 0) printf("mqtt_send error=%i\n", snd);
}

void motion_interrupt(int gpio, int level, uint32_t tick){
    std::string msg = "1";
    int snd = mqtt_send("room_monitor/dust", msg);
    if(snd != 0) printf("mqtt_send error=%i\n", snd);
}

int main(void){

    // Open I2C file descriptor
    fd_i2c = open("/dev/i2c-1", O_RDWR);
    if(fd_i2c < 0){
        printf("Failed to open I2C interface!\n");
        return 1;
    }

    // Initialize VEML7700 sensor
    als.init(fd_i2c);
    als.begin();

    // Setup MQTT
    mqtt_setup();
    
    // Initialize MCP3422 ADC
    adc.setI2CFd(fd_i2c);

    // Initialise Pigpio
    gpioInitialise();

    // Set motion sensor interrupt
    gpioSetMode(MOTION_SENSOR_GPIO, PI_INPUT);
    gpioSetPullUpDown(MOTION_SENSOR_GPIO, PI_PUD_OFF);
    gpioSetISRFunc(MOTION_SENSOR_GPIO, RISING_EDGE, 10, motion_interrupt);

    // Initialize dust sensor infrared LED output pin
    gpioSetMode(DUST_SENSOR_GPIO, PI_OUTPUT);
    
    while(1){
        mq2_measurement();
        VEML7700_measurement();
        // dust_measurement(); // only if connected

        time_sleep(5);
    }
    
    return 0;
}