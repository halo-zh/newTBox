#include "usart.h"
#include "MC20.h"
#include "cmsis_os.h"
#include "string.h"
#include "can.h"
#include "adc.h"
#include <stdio.h>


#define ProductKey     "a1fkV4RfRSP"             //2ú?·KEY
#define DeviceName     "AQ_TEST1"      //
#define DeviceSecret    "7be2672204c0051eb271dfe09999b3d9"  //
#define PeriodicTopic         "/sys/a1fkV4RfRSP/AQ_TEST1/thing/event/property/post"
#define BatEventTopic        "/sys/a1fkV4RfRSP/AQ_TEST1/thing/event/BatErr/post"
#define InverterEventTopic        "/sys/a1fkV4RfRSP/AQ_TEST1/thing/event/InverterErr/post"
uint8_t temp;

uint8_t mc20Status=0;

uint8_t gprsStatus = 0;
uint8_t gnssStatus = 0;
uint8_t mqttStatus=0;

uint8_t mqttData[500];
uint8_t gpsInitDone =0;
extern uint8_t gnssDataAval;
extern uint32_t jd;
extern uint32_t wd;



void mc20PowerOn()
{
   HAL_GPIO_WritePin(MC20_EN_GPIO_Port,MC20_EN_Pin,SET);
   osDelay(1200);
   //HAL_GPIO_WritePin(MC20_EN_GPIO_Port,MC20_EN_Pin,RESET);
   printf("MC20 Power ON\r\n");
}


void mc20PowerOff()
{
   HAL_GPIO_WritePin(MC20_EN_GPIO_Port,MC20_EN_Pin,RESET);
   HAL_Delay(100);
   
   HAL_GPIO_WritePin(MC20_EN_GPIO_Port,MC20_EN_Pin,SET);
   HAL_Delay(1200);
   HAL_GPIO_WritePin(MC20_EN_GPIO_Port,MC20_EN_Pin,RESET);
   printf("MC20 Power OFF\r\n");
}



void closeGNSS()
{
   uint8_t iter=0;
   int ret = AT_FAILED;

    while(iter++ <10) {
        ret = mc20_normal_check("AT+QGNSSC=0\r\n");
        if(ret == AT_SUCCESS) {
            break;
        }
        osDelay(10);
    } 
}

void closePower()
{
   uint8_t iter=0;
   int ret = AT_FAILED;

    while(iter++ <10) {
        ret = mc20_normal_check("AT+QPOWD=1\r\n");
        if(ret == AT_SUCCESS) {
            break;
        }
        osDelay(10);
    } 
}





void MC20_Init()
{
    
   mc20PowerOn();
   
START:  
    HAL_UART_Receive_IT(&huart3, &temp, 1);
    int iter = 0;
    int ret = AT_FAILED;
    while(1) {
      ret = mc20_normal_check("AT\r\n");
      if(ret == AT_SUCCESS) {
          break;
      }
      osDelay(10);
    }
    printf("MC20 COM OK\r\n");
    
    mc20Status = 1;
  


    iter = 0;
    while(iter++ <10) {
        ret = mc20_normal_check("ATE1\r\n");
        if(ret == AT_SUCCESS) {
            break;
        }
        osDelay(10);
    }
    if(ret == AT_FAILED) {
         goto START ;
    }
  
		
     iter = 0;
    while(iter++ <10) {
        ret = mc20_normal_check("AT+CMEE=2\r\n");
        if(ret == AT_SUCCESS) {
            break;
        }
        osDelay(10);
    }
    if(ret == AT_FAILED) {
         goto START ;
    }
    
    

    iter = 0;
    while(iter++ <10) {
        ret = mc20_check_sim_card();
        if(ret == AT_SUCCESS) {
            break;
        }
        osDelay(10);
    }
    if(ret == AT_FAILED) {
      printf("no sim card\r\n");
         goto START ;
    }
    printf("SIM card detected\r\n");
    iter = 0;
    while(1) {
        ret = mc20_check_creg();
        if(ret == AT_SUCCESS) {
            gprsStatus =1;
            break;
        }
        osDelay(10);
    }
    printf("Reg OK\r\n");
		////printf("creg check ret is %d\n", ret);
    if(ret == AT_FAILED) 
    {
       gprsStatus = 0;
       gnssStatus =0;
       goto START ;
    }
    osDelay(5);
    
    
  
}


void GPSInit()
{
  
    uint8_t  ret = 0;
    
    printf("GNSS Init Start\r\n");
    ret = mc20_normal_check("AT+QIFGCNT=2\r\n");
    if(ret != AT_SUCCESS) {
       printf("PDP context Fail\r\n");
        return;
    }
    

    ret = mc20_normal_check("AT+QICSGP=1,\"CMNET\"\r\n");
    if(ret != AT_SUCCESS) {
       printf("SET APN FAIL\r\n");
        return;
    }
    

    
    mc20_check_timesync();
    if(ret != AT_SUCCESS) 
    {
       printf("GNSS TimeSync Fail\r\n");
       return;      
    }
    osDelay(10);
     printf("GNSS TimeSync OK\r\n");          
    
    

    ret = mc20_normal_check("AT+QGNSSEPO=1\r\n");
    if(ret != AT_SUCCESS) {
        return;
    }
    osDelay(10);

    
 /*    
    ret = mc20_normal_check("AT+QGEPOAID\r\n");
    if(ret != AT_SUCCESS) 
    {
        return;
    }
 */   
    
    
    ret = mc20_normal_check("AT+QGNSSC=1\r\n");
    if(ret != AT_SUCCESS) {
       printf("GNSS Enable Failed\r\n");
        return;
    }
    osDelay(10);
    printf("GNSS Enable Success\r\n");
        
    gpsInitDone =1;
    printf("GNSS Init Done\r\n");

}
         



void send(char* buffer)
{
  user_send_data_with_delay(buffer);
}


void readGnssData()
{
   mc20_read_gnss("AT+QGNSSRD=\"NMEA/RMC\"\r\n");
}



//提交的JASON数据
float Longitude=0;
float Latitude=0;
float Altitude =0;

void calculateGPS()
{
    uint32_t tempjd ;
    uint32_t tempwd ;
    if(gnssDataAval == 1)
    {
       tempjd = jd/1000000;
       tempwd = wd/1000000;       
       Longitude = tempjd;
       Latitude =  tempwd;
       
       tempjd = (jd%1000000);
       tempwd = (wd%1000000);
       
       Longitude += (float)tempjd/60/10000;
       Latitude  += (float)tempwd/60/10000;

       printf("Longitude =%f,Latitude =%f\r\n",Longitude,Latitude);
    }
    else
    {
      gnssDataAval =0;
      printf("GNSS Data Not Ready\r\n");
      Longitude =0;
      Latitude =0;    
    }
}


uint32_t packID=0;
uint16_t packGPSData(char *t_payload)
{  
      //uint16_t pkt_id = 1;
    char json[]="{\"id\":\"%d\",\"params\":{\"GeoLocation\":{\"Longitude\":%3.10f,\"Latitude\":%3.10f,\"Altitude\":%f,\"CoordinateSystem\":1}}}";	 
    char t_json[500];

    unsigned short json_len;
    calculateGPS();
    
    sprintf(t_json, json,packID,Longitude,Latitude,Altitude);

    json_len = strlen(t_json)/sizeof(char);
    memcpy(t_payload, t_json, json_len);
    packID++;
    return json_len;
}



uint16_t packLiveData(char *t_payload)
{
    
      //uint16_t pkt_id = 1;
    char json[]="{\"id\":\"%d\",\"params\":{\"BatDischargeCurrent\":%d,\"BatVolt\":%d,\"batSOC\":%d,\"batSOH\":%d,\"throttlePercent\":%d,\"motorRPM\":%d}}";	 
    char t_json[500];

    unsigned short json_len;
    sprintf(t_json, json,packID, batCurrent/10, batVolt/10,batSOC,batSOH,throttlePercent,motorRPM);

    json_len = strlen(t_json)/sizeof(char);
    memcpy(t_payload, t_json, json_len);
    packID++;
    return json_len;
}

uint8_t bHallFault=0;
uint8_t bThrottleFault=0;
uint8_t bMOSFETFault=0;
uint8_t bInverterOverVolt=0;
uint8_t bInverterLowVolt=0;
uint8_t bInverterOverCurrent=0;
uint8_t bInverterOverTemp=0;
uint8_t bMotorOverTemp=0;
uint8_t bRotorLocked=0;
uint8_t bInverterFault=0;
uint8_t bInverterPhaseLossFault=0;


char inverterErrorInfo[500];


char checkInverterErr()
{
  if(!((bHallFault)||\
          (bThrottleFault)||\
            (bMOSFETFault)||\
              (bInverterOverVolt)||\
                (bInverterLowVolt)||\
                  (bInverterOverCurrent)||\
                    (bInverterOverTemp)||\
                      (bMotorOverTemp)||\
                        (bRotorLocked)||\
                          (bInverterFault)||\
                            (bInverterPhaseLossFault)))
    {
      return 0;
    }
    else
    {
      return 1;
    }
  
}


uint16_t packInverterErrData(char *t_payload)
{
    

    memset(inverterErrorInfo,0,500);
    //uint16_t pkt_id = 1;
    char json[]="{\"id\":\"%d\",\"params\":{\"InverterFaultInfo\":\"%s\"}}";	 
#if 0   
    if( bHallFault ==1 )
    {    
        strcat(inverterErrorInfo,"HallFault;");
    }
    if( bThrottleFault ==1 )
    {    
        strcat(inverterErrorInfo,"ThrottleFault;");
    }
    if( bMOSFETFault ==1 )
    {    
         strcat(inverterErrorInfo,"MOSFETFault;");
    }
    if( bInverterOverVolt ==1 )
    {    
         strcat(inverterErrorInfo,"InverterOverVolt;");
    }
    if( bInverterLowVolt ==1 )
    {    
         strcat(inverterErrorInfo,"InverterLowVolt;");
    }
     if( bInverterOverCurrent ==1 )
    {    
         strcat(inverterErrorInfo,"InverterOverCurrent;");
    }
     if( bInverterOverTemp ==1 )
    {    
         strcat(inverterErrorInfo,"InverterOverTemp;");
    }
    if( bMotorOverTemp ==1 )
    {    
         strcat(inverterErrorInfo,"MotorOverTemp;");
    }
    if( bRotorLocked ==1 )
    {    
         strcat(inverterErrorInfo,"RotorLocked;");
    }
    if( bInverterFault ==1 )
    {    
         strcat(inverterErrorInfo,"InverterFault;");
    }
    if( bInverterPhaseLossFault ==1 )
    {    
         strcat(inverterErrorInfo,"InverterPhaseLossFault;");
    }
#endif 
    
    if( bHallFault ==1 )
    {    
        strcat(inverterErrorInfo,"HF;");
    }
    if( bThrottleFault ==1 )
    {    
        strcat(inverterErrorInfo,"TF;");
    }
    if( bMOSFETFault ==1 )
    {    
         strcat(inverterErrorInfo,"MF;");
    }
    if( bInverterOverVolt ==1 )
    {    
         strcat(inverterErrorInfo,"IOV;");
    }
    if( bInverterLowVolt ==1 )
    {    
         strcat(inverterErrorInfo,"ILV;");
    }
     if( bInverterOverCurrent ==1 )
    {    
         strcat(inverterErrorInfo,"IOC;");
    }
     if( bInverterOverTemp ==1 )
    {    
         strcat(inverterErrorInfo,"IOT;");
    }
    if( bMotorOverTemp ==1 )
    {    
         strcat(inverterErrorInfo,"MOT;");
    }
    if( bRotorLocked ==1 )
    {    
         strcat(inverterErrorInfo,"RL;");
    }
    if( bInverterFault ==1 )
    {    
         strcat(inverterErrorInfo,"IF;");
    }
    if( bInverterPhaseLossFault ==1 )
    {    
         strcat(inverterErrorInfo,"IPLF;");
    }
    char t_json[500];
 
    unsigned short json_len;
    sprintf(t_json, json,packID, inverterErrorInfo);
    json_len = strlen(t_json)/sizeof(char);
    memcpy(t_payload, t_json, json_len);
    packID++;
    return json_len;
}


uint8_t bDischargeTempLow=0;
uint8_t bDischargeTempHigh=0;
uint8_t bChargeTempLow=0;
uint8_t bChargeTempHigh=0;
uint8_t bBatVoltHigh=0;
uint8_t bBatVoltLow=0;
uint8_t bCellVoltLow=0;
uint8_t bCellVoltHigh=0;
uint8_t bSOCLow=0;
uint8_t bSOCHigh=0;
uint8_t bDischargeCurrentHigh=0;
uint8_t bChargeCurrentHigh=0;
uint8_t bBatTempHigh=0;
uint8_t bBatBalcanceTempHigh=0;
uint8_t bBatDiffTempHigh=0;
uint8_t bBatVoltDiffHigh=0;
uint8_t bBatSampleLineFault=0;
uint8_t bBatTempLineFault=0;




char checkBatErr()
{
  if(((bDischargeTempLow)||\
       ( bDischargeTempHigh)||\
        (  bChargeTempLow)||\
          ( bChargeTempHigh)||\
          (  bBatVoltHigh)||\
            (  bBatVoltLow)||\
            (  bCellVoltLow)||\
            (  bCellVoltHigh)||\
              ( bSOCLow)||\
               (  bSOCHigh)||\
               (  bDischargeCurrentHigh)||\
               (  bChargeCurrentHigh)||\
               (  bBatTempHigh)||\
               (  bBatBalcanceTempHigh)||\
               (  bBatDiffTempHigh)||\
               (  bBatVoltDiffHigh)||\
                ( bBatSampleLineFault)||\
                  ( bBatTempLineFault)))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  
}

char batErrorInfo[500];
uint16_t packBatErrData(char *t_payload)
{
    
   // uint16_t pkt_id = 1;
    char json[]="{\"id\":\"%d\",\"params\":{\"BatErrorInfo\":\"%s\"}}";  	 
    char t_json[500];
   
    unsigned short json_len;
     
    memset(batErrorInfo,0,500);
#if 0
    if( bDischargeTempLow ==1 )
    {    
         strcat(batErrorInfo,"DischargeTempLow;");
    }
    if( bDischargeTempHigh ==1 )
    {    
         strcat(batErrorInfo,"DischargeTempHigh;");
    }
    if( bChargeTempLow ==1 )
    {    
         strcat(batErrorInfo,"ChargeTempLow;");
    }
    if( bChargeTempHigh ==1 )
    {    
         strcat(batErrorInfo,"ChargeTempHigh;");
    }
    if( bBatVoltHigh ==1 )
    {    
         strcat(batErrorInfo,"BatVoltHigh;");
    }
    if( bBatVoltLow ==1 )
    {    
         strcat(batErrorInfo,"BatVoltLow;");
    }
    if( bSOCLow ==1 )
    {    
         strcat(batErrorInfo,"SOCLow;");
    }
    if( bSOCHigh ==1 )
    {    
         strcat(batErrorInfo,"SOCHigh;");
    }
    if( bDischargeCurrentHigh ==1 )
    {    
         strcat(batErrorInfo,"DischargeCurrentHigh;");
    }
    if( bChargeCurrentHigh ==1 )
    {    
         strcat(batErrorInfo,"ChargeCurrentHigh;");
    }
    if( bBatTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BatTempHigh;");
    }
    if( bBatBalcanceTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BatBalcanceTempHigh;");
    }
    if( bBatDiffTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BatDiffTempHigh;");
    }
    if( bBatVoltDiffHigh ==1 )
    {    
         strcat(batErrorInfo,"BatVoltDiffHigh;");
    }
    if( bBatSampleLineFault ==1 )
    {    
         strcat(batErrorInfo,"BatSampleLineFault;");
    }
    if( bBatTempLineFault ==1 )
    {    
         strcat(batErrorInfo,"BatTempLineFault;");
    }
#endif   
    
    
    if( bDischargeTempLow ==1 )
    {    
         strcat(batErrorInfo,"DTL;");
    }
    if( bDischargeTempHigh ==1 )
    {    
         strcat(batErrorInfo,"DTH;");
    }
    if( bChargeTempLow ==1 )
    {    
         strcat(batErrorInfo,"CTL;");
    }
    if( bChargeTempHigh ==1 )
    {    
         strcat(batErrorInfo,"CTH;");
    }
    if( bBatVoltHigh ==1 )
    {    
         strcat(batErrorInfo,"BVH;");
    }
    if( bBatVoltLow ==1 )
    {    
         strcat(batErrorInfo,"BVL;");
    }
    if( bSOCLow ==1 )
    {    
         strcat(batErrorInfo,"SL;");
    }
    if( bSOCHigh ==1 )
    {    
         strcat(batErrorInfo,"SH;");
    }
    if( bDischargeCurrentHigh ==1 )
    {    
         strcat(batErrorInfo,"DCH;");
    }
    if( bChargeCurrentHigh ==1 )
    {    
         strcat(batErrorInfo,"CCH;");
    }
    if( bBatTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BTH;");
    }
    if( bBatBalcanceTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BBTH;");
    }
    if( bBatDiffTempHigh ==1 )
    {    
         strcat(batErrorInfo,"BDTH;");
    }
    if( bBatVoltDiffHigh ==1 )
    {    
         strcat(batErrorInfo,"BVDH;");
    }
    if( bBatSampleLineFault ==1 )
    {    
         strcat(batErrorInfo,"BSLF;");
    }
    if( bBatTempLineFault ==1 )
    {    
         strcat(batErrorInfo,"BTLF;");
    }
    
    sprintf(t_json, json, packID,batErrorInfo);  
    json_len = strlen(t_json)/sizeof(char);
    memcpy(t_payload, t_json, json_len);
    packID++;
    return json_len;
}



int MQTT_CFG()
{
  char txbuffer[300];
  sprintf(txbuffer,"AT+QMTCFG=\"ALIAUTH\",0,\"%s\",\"%s\",\"%s\"\r\n",ProductKey,DeviceName,DeviceSecret);
  //printf("%s",txbuffer);
  user_send_data_with_delay(txbuffer);  
 #define BUF_LEN 100
  char buffer[BUF_LEN] = {0};

  int ret = user_get_data_with_delay(buffer, BUF_LEN,1000);
  if(strstr(buffer, "OK") != NULL) {     // CFG SUCCESS
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_OPEN()
{
   char buffer[100]="AT+QMTOPEN=0,\"iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\r\n";
//   printf("%s",buffer);
   user_send_data_with_delay(buffer);  
  #define BUF_LEN 100

  int ret = user_get_data_with_delay(buffer, BUF_LEN,100);
  if((strstr(buffer, "QMTOPEN: 0,0") != NULL)||(strstr(buffer, "QMTOPEN: 0,2"))) {     // CFG SUCCESS
    printf("MQTT OPEN SUCCESS\r\n");  
    return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_OPEN_CHECK()
{
   char buffer[100]="AT+QMTOPEN?\r\n";
//   printf("%s",buffer);
   user_send_data_with_delay(buffer);  
  #define BUF_LEN 100

  int ret = user_get_data_with_delay(buffer, BUF_LEN,5000);
  if(strstr(buffer, "QMTOPEN: 0") != NULL) {     // CFG SUCCESS
    printf("MQTT OPEN CHECK SUCCESS\r\n"); 
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_Connect()
{
  
  char buffer[100] = "AT+QMTCONN=0,\"scootertest1\"\r\n";
//   printf("%s",buffer);
  user_send_data_with_delay(buffer);  

  int ret = user_get_data_with_delay(buffer, BUF_LEN,3000);
  if(strstr(buffer, "QMTCONN: 0,0,0") != NULL) {     // CFG SUCCESS
    printf("MQTT CONNECT SUCCESS\r\n"); 
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_Pub_Topic(uint8_t *topic,uint8_t len)
{
    char buffer[100];
    sprintf(buffer,"AT+QMTPUB=0,0,0,0,\"%s\",%d\r\n",topic,len);
 //   printf("%s",buffer);
    
    user_send_data_with_delay(buffer);  

  int ret = user_get_data_with_delay(buffer, BUF_LEN,20);
  if(strstr(buffer, ">") != NULL) {     // CFG SUCCESS
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


uint8_t DataPaly;
char mqtt_pub_data[500];
int MQTT_Pub_Data()
{
  
  char buffer[100];
  user_send_data_with_delay(mqtt_pub_data);  

  int ret = user_get_data_with_delay(buffer, BUF_LEN,200);
  if(strstr(buffer, "QMTPUB: 0,0,0") != NULL) {     // CFG SUCCESS
      printf("MQTT Public Data SUCCESS\r\n"); 
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_Disconnect()
{
  
  char buffer[100]="AT+QMTDISC=0\r\n";
  user_send_data_with_delay(buffer);  
  

  int ret = user_get_data_with_delay(buffer, BUF_LEN,100);
  if(strstr(buffer, "CREG: 0,1") != NULL) {     // CFG SUCCESS
      return AT_SUCCESS;
  }
  return AT_FAILED;
}


int MQTT_Start(void)
{
  mqttStatus = 0;
  int ret = AT_FAILED;
  while(ret != AT_SUCCESS)
  {
    ret = MQTT_CFG();
    printf("MQTT CFG FAILED\r\n");
    osDelay(10);
  }
  printf("MQTT CFG SUCCESS\r\n");
  
  
OPEN_LOOP:  
  ret = MQTT_OPEN();
  osDelay(10);
  ret = MQTT_OPEN_CHECK();
  if(ret!= AT_SUCCESS )
    goto OPEN_LOOP;
  
  ret = MQTT_Connect();
    if(ret!= AT_SUCCESS )
    {
      printf("MQTT Connect ALIYUN FAILED\r\n");
       goto OPEN_LOOP;
    }
    printf("MQTT Connect ALIYUN SUCCESS\r\n");
    mqttStatus = 1;
    return 0;
  
}

uint16_t len=0;
void pubLiveData()
{
     int ret;
     memset(mqtt_pub_data,0,500);
     len = packLiveData(mqtt_pub_data);
     if(len > 0)
    {
        ret = MQTT_Pub_Topic(PeriodicTopic,len);   
        ret = MQTT_Pub_Data();
    }
}



void pubGPSData()
{
     int ret;
     memset(mqtt_pub_data,0,500);
     len = packGPSData(mqtt_pub_data);
     if(len >0)
    {
      ret = MQTT_Pub_Topic(PeriodicTopic,len);   
      ret = MQTT_Pub_Data();
    }
}

void pubInverterErrData()
{

     int ret;
     memset(mqtt_pub_data,0,500);
     len = packInverterErrData(mqtt_pub_data);
     if(len >0)
     {    
       ret = MQTT_Pub_Topic(InverterEventTopic,len);   
       ret = MQTT_Pub_Data();
     }
}

void pubBatErrData()
{
     int ret;
     memset(mqtt_pub_data,0,500);
     len = packBatErrData(mqtt_pub_data);
     if(len >0)
     {
       ret = MQTT_Pub_Topic(BatEventTopic,len);   
       ret = MQTT_Pub_Data();    
     }

}


