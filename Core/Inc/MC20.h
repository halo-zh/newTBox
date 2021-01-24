#ifndef _MC20_H
#define _MC20_H

extern void MC20_Init();
extern int MQTT_Start(void);
extern int MQTT_Pub_Topic(uint8_t *topic,uint8_t len);
extern int MQTT_Pub_Data();
extern uint16_t len;
extern uint8_t mqttData[500];
extern uint8_t gpsInitDone;
extern uint8_t mqttStatus;

extern void pubLiveData();
extern uint16_t packLiveData();
extern uint16_t packGPSData();
extern void pubGPSData();
extern char checkBatErr();
extern char checkInverterErr();

extern uint16_t packInverterErrData();
extern uint16_t packBatErrData();
extern void pubBatErrData();
extern void pubInverterErrData();
extern void readGnssData();
extern enum MC20_Status;
extern void GPSInit();




#define AT_SUCCESS 0
#define AT_FAILED  -1

#endif