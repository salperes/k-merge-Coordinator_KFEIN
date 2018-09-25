#pragma once
// -----------------------------  COORDINATOR DEFAULTS --------------------
#define COORDINATOR_HW_VERSION 1
#define COORDINATOR_SW_VERSION_MAJOR 0
#define COORDINATOR_SW_VERSION_MINOR 10
#define COORDINATOR_SERIAL_MSB 0
#define COORDINATOR_SERIAL_LSB 1
#define COORDINATOR_RADIO_TX_POWER 13
#define def_WPS_LOST_COUNT 3 // ASSUME WPS LOST AFTER NUMBER TRANSMIT COUNTS

#define MAX_WPS_NUMBER 127 // 0, 128 COODINATOR ADDRESLERÝ
#define def_INSTALLED_WPS_COUNT 16 //TEST ÝÇÝN 16 OLARAK BELÝRLENDÝ

#define TESTSERVER 	"amc-wps02.amcpro.com.tr"
//"78.187.41.206" 
//"37.77.17.149"
#define TESTPORT 1883
#define def_MQTT_PASSWORD "Alp2013er"
#define def_MQTT_USERNAME "wpsdevices"
#define def_MQTT_TOPIC "coordinator"
#define DEFAULT_FREQ 433.92

#define BASE_FREQ433 433.15
#define BASE_FREQ865 865.20
#define BASE_CHANNEL 0;

// -----------------------------  WPS  DEFAULTS --------------------
#define def_WPS_ADDRESS 60
#define WPS_SERIAL 7  //////  SERÝ NUMARASINA DÝKKAT EDÝLECEK.... 48bit


#define HW_VERSION 6//BUNA DA DÝKKAT EDÝLECEK. AÇIKLAMA AÞAÐIDA

#define SW_VERSION_MAJOR 3
#define SW_VERSION_MINOR 05


#define def_BLE_TIMEOUT 2
#define def_VECTOR_VARIANCE_LIMIT  5
#define def_MIN_RSSI_DIFF 3
#define def_MAX_RSSI_DIFF 10
#define def_VECTOR_VARIANCE_BASE 3 // MINIMUM VECTOR VARIANCE
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

#define SleepCycleFactor 5 // SLEEP TIMEOUT = SLEEP_COUNT*SleepCycleFactor 

#define WPS_BROADCAST_ADDRESS 0xFE
#define def_NETWORK_ID 0xAE
#define def_COORDINATOR_ADDRESS 0
#define def_SECRET_CODE 38013


#define VoltageMeasurePIN A2




#define WPS_PACKET_LENGHT 34 // PACKET LENGHT FOR WPS PACKET
// **************************** WPS->COORDINATOR PAKET VERÝ YAPISI **************************
// 00 - NW ID
// 01 - FLAG TYPE
// 02 - SEQUENCE 3rd
// 03 - SEQUENCE 2nd
// 04 - SEQUENCE 1st
// 05 - SENSÖR SICAKLIK MSB
// 06 - SENSÖR SICAKLIK LSB
// 07 - VEKTÖR SAPMASI
// 08 - ARAÇ VAR/YOK
// 09 - PÝL SEVÝYESÝ MSB
// 10 - PÝL SEVÝYESÝ LSB
// 11 - MAGNETOMETER PARAMETERS
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

// 12 - LAST RSSI FROM WPS SIDE MSB
// 13 - LAST RSSI FROM WPS SIDE LSB
// 14 - WPS SERIAL 6th
// 15 - WPS SERIAL 5th
// 16 - WPS SERIAL 4th
// 17 - WPS SERIAL 3rd
// 18 - WPS SERIAL 2nd
// 19 - WPS SERIAL 1st
// 20 - WPS HW Versiyon (Bit 7-4) and WPS SW Major (Bit 3-0)
// 21 - WPS SW Minor (BIT 0-4) & MIN_VARIANCE (5-7)
// 22 - VEKTÖR SAPMA LÝMÝTÝ (Bit 7-2) and VEHICLE_DETECTION_COUNT (Bit 1-0)
// 23 - BLE TX POWER (Bit 6-5) & RADIO TX POWER (Bit 4-0) & CHECK_RSSI (Bit 7)
// 24 - SENSOR GAIN (Bit 7-6) & WPS_SLEEP_COUNT (Bit 5-0)
// 25 - LoRa Channel
// 26 - ERROR CODE
//		BIT 0 - LSM303C INIT ERROR
//		BIT 1 - BLUETOOTH ERROR
//		BIT 2 - LoRA PREVIOUS TRANSMISSION FAILED
// 27 - BLE WAIT TIMEOUT (Bit 7-4) & LAST TRANSMISSION RETRY COUNT (Bit 3-0)
// 28 - Vector 4th
// 29 - Vector 3rd
// 30 - Vector 2nd
// 31 - Vector 1st
// 32 - MIN_RSSI_DIFF (BIT 0-2), MAX_RSSI_DIFF (3-7)
// (LEN-1) - CHECKSUM


// **************************** WPS EEPROM VERÝ YAPISI **************************
// 0, NETWORK_ID
// 1, 1 //1 EEPROM'DAKÝ VERÝLERÝ KULLAN DEMEK....
// 2, COORDINATOR_ADDRESS
// 3, WPS_ADDRESS
// 4, GAIN
// 5, WPS_SLEEP_COUNT
// 6, VECTOR_VARIANCE
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
// 23, MIN_VARIANCE
// 24, MIN_RSSI_DIFF
// 25-26-27-28 - Vector0
// 29 - VarianceOrDiff
// 30 - InitialState (Sadece Remote Reset için) 


#define COORDINATOR_PACKET_LENGHT 16 // PACKET LENGHT FOR REPLY PACKET
// **************************** COORDINATOR->WPS PAKET VERÝ YAPISI **************************
// 00 - NETWORK ID
// 01 - FLAG TYPE 
#define FLAG_PERIODICAL 1
#define FLAG_VEHICLE_STATUS_CHANGE 2
#define FLAG_CONFIGURATION_SET 4
#define FLAG_ACK_OK 8
#define FLAG_RETRANSMIT 16
#define FLAG_FIN 32
#define FLAG_PING 64 // VERSION 3 and ABOVE

// 02 - YENÝ MAX_RSSI_DIFF (Bit 0-4), YENÝ MAG_FRQ (BIT 5-7) 
// 03 - RESET VECTOR0 / WPS SOFTWARE ? (DEFAULT = 0, VECTOR0=1, WPS SOFTWARE=2), set InitialState on Byte06
// 04 - COORDINATOR SIDE RECEIVED SNR 
// 05 - SET NEW DATA 
					//0=NONE
					//BIT 0 = RADIO TX POWER & BLE TX POWER
					//BIT 1 = VEHICLE DETECTION COUNT & BLE TIME OUT
					//BIT 2 = VECTOR VARIANCE LIMIT & CHECK_RSSI & VECTOR_VARIANCE_BASE & MIN_RSSI_DIFF & MAX_RSSI_DIFF
					//BIT 3 = SLEEP COUNT
					//BIT 4 = GAIN, MAG_FRQ, MAG_OMXY, MAG_OMZ, VarianceOrDiff, InitialState
					//BIT 5 = COORDINATOR ADDRESS & LORA CHANNEL
					//BIT 6 = WPS ADDRESS
					//BIT 7 = NETWORK ID
// 06 - YENÝ GAIN (Bit 5-4), YENÝ MAG_OMXY (Bit 3-2), YENÝ MAG_OMZ (1-0), 
	 // VarianceOrDiff (Bit 6,  yüzde deðiþim mi yoksa fark mý), InitialState (Bit 7, araç var / yok)
// 07 - YENÝ NETWORK ID
// 08 - YENÝ WPS ID
// 09 - YENÝ COORDINATOR ID
// 10 - YENÝ WPS_SLEEP_COUNT (8s x WPS_SLEEP_COUNT)
// 11 - YENÝ VECTOR_VARIANCE LIMIT
// 12 - YENÝ VEHICLE_DETECTION_COUNT & BLE_TIMEOUT (kaç tane 8sn aralýk gözleyecek (0-1 Bit) -  BLE timeout (2-5 bit)) - Bit 6-7 Boþ 
// 13 - YENÝ RADIO_TX_POWER & BLE POWER
// 14 - YENÝ LORA CHANNEL
// 15 - YENÝ CHECK_RSSI (Bit 0) & VECTOR_VARIANCE_BASE (BIT 1-3), MIN_RSSI_DIFF (4-6), Bit 7 BOÞ !!!!!!!!!!!!!






#define GAIN_4 0x00
#define GAIN_8 0x20
#define GAIN_12 0x40
#define GAIN_16 0x60


//HMC için tanýmlamalar

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


