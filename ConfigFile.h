#pragma once
// -----------------------------  COORDINATOR DEFAULTS --------------------
#define COORDINATOR_HW_VERSION 1
#define COORDINATOR_SW_VERSION_MAJOR 0
#define COORDINATOR_SW_VERSION_MINOR 10
#define COORDINATOR_SERIAL_MSB 0
#define COORDINATOR_SERIAL_LSB 1
#define COORDINATOR_RADIO_TX_POWER 13
#define def_WPS_LOST_COUNT 3 // ASSUME WPS LOST AFTER NUMBER TRANSMIT COUNTS

#define MAX_WPS_NUMBER 127 // 0, 128 COODINATOR ADDRESLERİ
#define def_INSTALLED_WPS_COUNT 16 //TEST İÇİN 16 OLARAK BELİRLENDİ

#define TESTSERVER 	"ec2-54-245-35-126.us-west-2.compute.amazonaws.com"
#define def_MQTT_PASSWORD "Alp2013er"
#define def_MQTT_USERNAME "wpsdevices"
#define def_MQTT_TOPIC "coordinator"
#define def_MQTT_PORT 1883

#define BASE_FREQ433 433.15
#define BASE_FREQ865 865.20
#define BASE_CHANNEL 0

// -----------------------------  WPS  DEFAULTS --------------------
#define def_WPS_ADDRESS 60
#define WPS_SERIAL 7  //////  SERİ NUMARASINA DİKKAT EDİLECEK.... 48bit


#define HW_VERSION 6//BUNA DA DİKKAT EDİLECEK. AÇIKLAMA AŞAĞIDA

#define SW_VERSION_MAJOR 3
#define SW_VERSION_MINOR 06


#define def_BLE_TIMEOUT 2
#define def_VECTOR_VARIANCE_LIMIT  5
#define def_VECTOR_VARIANCE_BASE 3 // MINIMUM VECTOR VARIANCE

#define def_MIN_RSSI_DIFF 3
#define def_MAX_RSSI_DIFF 10

#define def_VEHICLE_DETECTION_COUNT 1 //+1 
#define def_RADIO_TX_POWER  13
#define def_WPS_SLEEP_COUNT 30
#define def_CHECK_RSSI false
#define def_MAG_FRQ 3
#define def_MAG_OMZ 3
#define def_MAG_OMXY 3

#define STATE_BUSY true
#define STATE_EMPTY false

#define VARIANCECHK true
#define DIFFCHK false

#define STATE_BUSY true
#define STATE_EMPTY false

#define MAG_ON true
#define MAG_OFF false

#define defDIFFMULTIPLIER 2

#define SleepCycleFactor 5 // SLEEP TIMEOUT = SLEEP_COUNT*SleepCycleFactor 

#define WPS_BROADCAST_ADDRESS 0xFE
#define def_NETWORK_ID 0xAE
#define def_COORDINATOR_ADDRESS 0
#define def_SECRET_CODE 38013


#define VoltageMeasurePIN A2




#define WPS_PACKET_LENGHT 32 // PACKET LENGHT FOR WPS PACKET
// **************************** WPS->COORDINATOR PAKET VERİ YAPISI **************************
// 00 - NW ID
// 01 - FLAG TYPE

// VARIABLE VALUES ///////////////////////////////////////////////////////////////////////////////////

// 02 - SEQUENCE 3rd
// 03 - SEQUENCE 2nd
// 04 - SEQUENCE 1st
// 05 - SENSÖR SICAKLIK MSB
// 06 - SENSÖR SICAKLIK LSB
// 07 - PİL SEVİYESİ MSB
// 08 - PİL SEVİYESİ LSB
// 09 - LAST RSSI FROM WPS SIDE 
// 10 - ERROR CODE & VEHICLE STATUS
//			BIT 0 - LSM303C INIT ERROR
//			BIT 1 - BLUETOOTH ERROR
//			BIT 2 - LoRA PREVIOUS TRANSMISSION FAILED
//			BIT 3-6 BOŞ
//			BIT 7 - VEHICLE STATUS
// 11 - LAST TRANSMISSION RETRY COUNT & VECTOR VARIANCE
//			BIT 0-3 - LAST TRANSMISSION RETRY COUNT
//			BIT 4-7	- VEKTÖR SAPMASI
// 12 - Vector 4th
// 13 - Vector 3rd
// 14 - Vector 2nd
// 15 - Vector 1st


// STATIC VALUES ///////////////////////////////////////////////////////////////////////////////////

// 16 - WPS HW & WPS SW MAJOR Version Number
//			BIT 0-3 - WPS SW MAJOR
//			BIT 4-7 - WPS HW VERSION
// 17 - WPS SW Minor Version
//			BIT 0-5 - WPS SW MINOR
//			BIT 6-7 - BOŞ
// 18 - WPS SERIAL 6th
// 19 - WPS SERIAL 5th
// 20 - WPS SERIAL 4th
// 21 - WPS SERIAL 3rd
// 22 - WPS SERIAL 2nd
// 23 - WPS SERIAL 1st

// CONFIGRABLE PARAMETERS
// 24 - MAGNETOMETER PARAMETERS
//			BIT 0-1		MAG Z PARAMETERS 
//					00 =  0 = MAG_OMZ_LOW_PW					= 0x00
//					01 =  1 = MAG_OMZ_MEDIUM_PERFORMANCE		= 0x04
//					10 =  2 = MAG_OMZ_HIGH_PERFORMANCE			= 0x08
//					11 =  3 = MAG_OMZ_ULTRA_HIGH_PERFORMANCE	= 0x0C
//			BIT 2-3		MAG XY PARAMETERS
//					00 =  0 = MAG_OMXY_LOW_PW					= 0x00
//					01 =  1 = MAG_OMXY_MEDIUM_PERFORMANCE		= 0x20
//					10 =  2 = MAG_OMXY_HIGH_PERFORMANCE			= 0x40
//					11 =  3 = MAG_OMXY_ULTRA_HIGH_PERFORMANCE	= 0x60
//			BIT 4-5-6	READ FREQ
//					000=  0 = MAG_DO_0_625_Hz					= 0x00					
//					001=  1 = MAG_DO_1_25_Hz					= 0x04
//					010=  2 = MAG_DO_2_5_Hz						= 0x08
//					011=  3 = MAG_DO_5_Hz						= 0x0C
//					100=  4 = MAG_DO_10_Hz						= 0x10
//					101=  5 = MAG_DO_20_Hz						= 0x14
//					110=  6 = MAG_DO_40_Hz						= 0x18
//					111=  7 = MAG_DO_80_Hz						= 0x1C
//			BIT 7 VarianceOrDiff (true vector variance, false vector diff)
// 25 - VECTOR VARIANCE LIMITS
//			BIT 0-2 - VECTOR_VARIANCE_BASE
//			BIT 3-7 - VECTOR_VARIANCE_LIMIT
// 26 - RSSI DIFF LIMITS
//			BIT 0-2 - MIN_RSSI_DIFF 
//			BIT 3-7 - MAX_RSSI_DIFF
// 27 - BLE TX POWER & RADIO TX POWER (Bit 4-0) & CHECK_RSSI (Bit 7)
//			BIT 0-4 - RADIO TX POWER 
// 			BIT 5-6 - BLE TX POWER 
//			BIT 7   - CHECK_RSSI (Bit 7)
// 28 - VEHICLE_DETECTION_COUNT & WPS_SLEEP_COUNT
//			BIT 0-5 - WPS_SLEEP_COUNT 
//			BIT 6-7 - VEHICLE_DETECTION_COUNT
// 29 - LORA CHANNEL
// 30 - DIFF_MULTIPLIER & BLE_WAIT_TIMEOUT & SENSOR GAIN 
//			BIT 0-1 - DIFF_MULTIPLIER
//			BIT 2-3 - SENSOR GAIN 
//			BIT 4-7 - BLE_WAIT_TIMEOUT
// CHECKSUM
// (LEN-1) - CHECKSUM


// **************************** WPS EEPROM VERİ YAPISI **************************
// 0, NETWORK_ID
// 1, 1 //1 EEPROM'DAKİ VERİLERİ KULLAN DEMEK....
// 2, COORDINATOR_ADDRESS
// 3, WPS_ADDRESS
// 4, GAIN
// 5, WPS_SLEEP_COUNT
// 6, VECTOR_VARIANCE_LIMIT
// 7, VEHICLE_DETECTION_COUNT
// 8, RADIO_TX_POWER & BLE POWER
// 9, VOLTAGE_OFFSET MSB
// 10, VOLTAGE_OFFSET LSB
// 11, SERIAL 6rd 
// 12, SERIAL 5rd 
// 13, SERIAL 4rd 
// 14, SERIAL 3rd
// 15, SERIAL 2nd
// 16, SERIAL 1st
// 17, LoRA Channel 
// 18, HW VERSION
//				1 - ARDUINO PRO MINI 3V3, HMC5983
//				2 - 328P+HMC5983+HM10
//				3 - ARDUINO PRO MINI 3V3, LIS3MDL
//				4 - ARDUINO PRO MINI 3V3, LSM303C
//				5 - 328P+LSM303C
//				6 - 328P+LSM303C+HM10
// 19, BLE TIMEOUT
// 20, MAX_RSSI_DIFF
// 21, MAG_FRQ+MAG_OMXY,MAG_OMZ
// 22, Check RSSI 
// 23, VECTOR_VARIANCE_BASE
// 24, MIN_RSSI_DIFF
// 25-26-27-28 - Vector0
// 29 - VarianceOrDiff
// 30 - InitialState (Sadece Remote Reset için) 
// 31 - DIFFMULTIPLIER - DIFF ÇARPANI (0=x10, 1=x20, 2=x30, 3=40, 4=x50)


#define COORDINATOR_PACKET_LENGHT 16 // PACKET LENGHT FOR REPLY PACKET
// **************************** COORDINATOR->WPS PAKET VERİ YAPISI **************************
// 00 - NETWORK ID
// 01 - FLAG TYPE 
#define FLAG_PERIODICAL 1
#define FLAG_VEHICLE_STATUS_CHANGE 2
#define FLAG_NEW_CONFIGURATION 3
#define FLAG_CONFIGURATION_SET 4
#define FLAG_ACK_OK 5
#define FLAG_RETRANSMIT 6
#define FLAG_FIN 7
#define FLAG_PING 8 // VERSION 3 and ABOVE
#define FLAG_INTER 9

// 02 - COORDINATOR SIDE RECEIVED SNR 
// 03 - RESET VECTOR0, RESET WPS SOFTWARE, SET NEW CONFIG
//						0 = No change
//						BIT 0 = 1 = Reset VECTOR0  (Bit 0)
//						BIT 1 = 2 = Reset WPS SOFTWARE (Bit 1)
//						BIT 2 = 4 = CHANGE REQUEST
//						BIT 3 = 8 = InitialState=STATE_BUSY
//						BIT 4 = 16 = Change WPS address(0), Change NW_ID(1)
//						BIT 5-7 	BOŞ

// 04 - Change Flag
//						BIT 0	-	MAG Z,XY,FRQ,VarOrDiff
//						BIT 1	-	Vector Variance Limits
//						BIT 2	-	RSSI Diff limits
//						BIT 3	-	BLE PWR, RADIO PWR, Chck RSSI
//						BIT 4	-	VDC, SLEEPC
//						BIT 5	-	LORA CHANNEL
//						BIT 6	-	DIFF MULTIPLIER, BLE TO, GAIN
//						BIT 7	-	WPS Address

// 05 - MAGNETOMETER PARAMETERS
//			BIT 0-1		MAG Z PARAMETERS 
//					00 =  0 = MAG_OMZ_LOW_PW					= 0x00
//					01 =  1 = MAG_OMZ_MEDIUM_PERFORMANCE		= 0x04
//					10 =  2 = MAG_OMZ_HIGH_PERFORMANCE			= 0x08
//					11 =  3 = MAG_OMZ_ULTRA_HIGH_PERFORMANCE	= 0x0C
//			BIT 2-3		MAG XY PARAMETERS
//					00 =  0 = MAG_OMXY_LOW_PW					= 0x00
//					01 =  1 = MAG_OMXY_MEDIUM_PERFORMANCE		= 0x20
//					10 =  2 = MAG_OMXY_HIGH_PERFORMANCE			= 0x40
//					11 =  3 = MAG_OMXY_ULTRA_HIGH_PERFORMANCE	= 0x60
//			BIT 4-5-6	READ FREQ
//					000=  0 = MAG_DO_0_625_Hz					= 0x00					
//					001=  1 = MAG_DO_1_25_Hz					= 0x04
//					010=  2 = MAG_DO_2_5_Hz						= 0x08
//					011=  3 = MAG_DO_5_Hz						= 0x0C
//					100=  4 = MAG_DO_10_Hz						= 0x10
//					101=  5 = MAG_DO_20_Hz						= 0x14
//					110=  6 = MAG_DO_40_Hz						= 0x18
//					111=  7 = MAG_DO_80_Hz						= 0x1C
//			BIT 7 VarianceOrDiff (true vector variance, false vector diff)
// 06 - VECTOR VARIANCE LIMITS
//			BIT 0-2 - VECTOR_VARIANCE_BASE
//			BIT 3-7 - VECTOR_VARIANCE_LIMIT
// 07 - RSSI DIFF LIMITS
//			BIT 0-2 - MIN_RSSI_DIFF 
//			BIT 3-7 - MAX_RSSI_DIFF
// 08 - BLE TX POWER & RADIO TX POWER (Bit 4-0) & CHECK_RSSI (Bit 7)
//			BIT 0-4 - RADIO TX POWER 
// 			BIT 5-6 - BLE TX POWER 
//			BIT 7   - CHECK_RSSI (Bit 7)
// 09 - VEHICLE_DETECTION_COUNT & WPS_SLEEP_COUNT
//			BIT 0-5 - WPS_SLEEP_COUNT 
//			BIT 6-7 - VEHICLE_DETECTION_COUNT
// 10 - LORA CHANNEL
// 11 - DIFF_MULTIPLIER & BLE_WAIT_TIMEOUT & SENSOR GAIN 
//			BIT 0-1 - DIFF_MULTIPLIER
//			BIT 2-3 - SENSOR GAIN 
//			BIT 4-7 - BLE_WAIT_TIMEOUT
// 12 - WPS_ADDRESS






#define GAIN_4 0x00
#define GAIN_8 0x20
#define GAIN_12 0x40
#define GAIN_16 0x60


//HMC için tanımlamalar

#define HMC_Address 0x1E
#define GAIN_088 0x00
#define GAIN_130 0x20 //default value
#define GAIN_190 0x40
#define GAIN_250 0x60
#define GAIN_400 0x80
#define GAIN_470 0xA0
#define GAIN_560 0xC0
#define GAIN_810 0xE0

#define Temp_Sensor_Enable  0x80
#define Temp_Sensor_Disable  0x00
#define Sample_x8 0x60
#define Sample_x4 0x40
#define Sample_x2 0x20
#define Sample_x1 0x00

#define Rate_220 0x1C
#define Rate_75 0x18
#define Rate_30 0x14
#define Rate_15 0x10
#define Rate_7d5 0x0C
#define Rate_3 0x08
#define Rate_1d5 0x04
#define Rate_0d75 0x00


