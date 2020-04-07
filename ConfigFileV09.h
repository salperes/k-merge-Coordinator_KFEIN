#pragma once
// -----------------------------  COORDINATOR DEFAULTS --------------------
#define COORDINATOR_HW_VERSION 1
#define COORDINATOR_SW_VERSION_MAJOR 0
#define COORDINATOR_SW_VERSION_MINOR 6
#define COORDINATOR_SERIAL_MSB 0
#define COORDINATOR_SERIAL_LSB 1
#define COORDINATOR_RADIO_TX_POWER 13
#define def_WPS_LOST_COUNT 3 // ASSUME WPS LOST AFTER NUMBER TRANSMIT COUNTS

#define MAX_WPS_NUMBER 80 // 0, 41, 82, 123, 184 COODINATOR ADDRESLERİ
#define def_INSTALLED_WPS_COUNT 16 //TEST İÇİN 16 OLARAK BELİRLENDİ

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
#define def_WPS_ADDRESS 44
#define WPS_SERIAL 7  //////  SERİ NUMARASINA DİKKAT EDİLECEK.... 48bit


#define HW_VERSION 6//BUNA DA DİKKAT EDİLECEK. AÇIKLAMA AŞAĞIDA

#define SW_VERSION_MAJOR 2
#define SW_VERSION_MINOR 5


#define def_BLE_TIMEOUT 3
#define def_VECTOR_VARIANCE_LIMIT  10
#define def_VEHICLE_DETECTION_COUNT  1
#define def_RADIO_TX_POWER  13
#define def_WPS_SLEEP_COUNT 30

#define SleepCycleFactor 5 // SLEEP TIMEOUT = SLEEP_COUNT*SleepCycleFactor 

#define WPS_BROADCAST_ADDRESS 0xFE
#define def_NETWORK_ID 0xAE
#define def_COORDINATOR_ADDRESS 0
#define def_SECRET_CODE 38013


#define VoltageMeasurePIN A2




#define WPS_PACKET_LENGHT 29 // PACKET LENGHT FOR WPS PACKET
// **************************** WPS->COORDINATOR PAKET VERİ YAPISI **************************
// 00 - NW ID
// 01 - FLAG TYPE
// 02 - SEQUENCE 3rd
// 03 - SEQUENCE 2nd
// 04 - SEQUENCE 1st
// 05 - SENSÖR SICAKLIK MSB
// 06 - SENSÖR SICAKLIK LSB
// 07 - VEKTÖR SAPMASI
// 08 - ARAÇ VAR/YOK
// 09 - PİL SEVİYESİ MSB
// 10 - PİL SEVİYESİ LSB
// 11 - PİL SEVİYESİ %
// 12 - LAST RSSI FROM WPS SIDE MSB
// 13 - LAST RSSI FROM WPS SIDE LSB
// 14 - WPS SERIAL 6th
// 15 - WPS SERIAL 5th
// 16 - WPS SERIAL 4th
// 17 - WPS SERIAL 3rd
// 18 - WPS SERIAL 2nd
// 19 - WPS SERIAL 1st
// 20 - WPS HW Versiyon (Bit 7-4) and WPS SW Major (Bit 3-0)
// 21 - WPS SW Minor
// 22 - VEKTÖR SAPMA LİMİTİ (Bit 7-2) and VEHICLE_DETECTION_COUNT (Bit 1-0)
// 23 - BLE TX POWER (Bit 6-5) & RADIO TX POWER (Bit 4-0)  --- BIT 7 BOŞTA
// 24 - SENSOR GAIN (Bit 7-6) & WPS_SLEEP_COUNT (Bit 5-0)
// 25 - LoRa Channel
// 26 - ERROR CODE
//		BIT 0 - LSM303C INIT ERROR
//		BIT 1 - BLUETOOTH INIT ERROR
//		BIT 2 - LoRA PREVIOUS TRANSMISSION FAILED
// 27 - BLE WAIT TIMEOUT (Bit 7-4) & LAST TRANSMISSION RETRY COUNT (Bit 3-0)
// (LEN-1) - CHECKSUM


// **************************** WPS EEPROM VERİ YAPISI **************************
// 0, NETWORK_ID
// 1, 1 //1 EEPROM'DAKİ VERİLERİ KULLAN DEMEK....
// 2, COORDINATOR_ADDRESS
// 3, WPS_ADDRESS
// 4, GAIN
// 5, WPS_SLEEP_COUNT
// 6, VECTOR_VARIANCE
// 7, VEHICLE_DETECTION_COUNT
// 8, RADIO_TX_POWER & BLE POWER
// 9, VOLTAGE_OFFSET MSB
// 10, VOLTAGE_OFFSET LSB
// 11, SERIAL 6rd - FREQUENCY 4th OCTET
// 12, SERIAL 5rd - FREQUENCY 3th OCTET
// 13, SERIAL 4rd - FREQUENCY 2th OCTET
// 14, SERIAL 3rd
// 15, SERIAL 2nd
// 16, SERIAL 1st
// 17, LoRA Channel 
// 18, HW VERSION
//				1 - ARDUINO PRO MINI 3V3, HMC5983
//				2 - MOTEINO, HMC5983
//				3 - ARDUINO PRO MINI 3V3, LIS3MDL
//				4 - ARDUINO PRO MINI 3V3, LSM303C
//				5 - 328P+LSM303C
//				6 - 328P+LSM303C+HM10
// 19, BLE TIMEOUT


#define COORDINATOR_PACKET_LENGHT 15 // PACKET LENGHT FOR REPLY PACKET
// **************************** COORDINATOR->WPS PAKET VERİ YAPISI **************************
// 00 - NETWORK ID
// 01 - FLAG TYPE 
#define FLAG_PERIODICAL 1
#define FLAG_VEHICLE_STATUS_CHANGE 2
#define FLAG_CONFIGURATION_SET 4
#define FLAG_ACK_OK 8
#define FLAG_RETRANSMIT 16
#define FLAG_FIN 32

// 02 - WPS ID

// 03 - RESET VECTOR0 / WPS SOFTWARE ? (DEFAULT = 0, VECTOR0=1, WPS SOFTWARE=2)
// 04 - COORDINATOR SIDE RECEIVED SNR 
// 05 - SET NEW DATA 
			//0=NONE
			//BIT 0 = RADIO TX POWER & BLE TX POWER
			//BIT 1 = VEHICLE DETECTION COUNT & BLE TIME OUT
			//BIT 2 = VECTOR VARIANCE LIMIT
			//BIT 3 = SLEEP COUNT
			//BIT 4 = GAIN
			//BIT 5 = COORDINATOR ADDRESS & LORA CHANNEL
			//BIT 6 = WPS ADDRESS
			//BIT 7 = NETWORK ID
// 06 - YENİ GAIN
// 07 - YENİ NETWORK ID
// 08 - YENİ WPS ID
// 09 - YENİ COORDINATOR ID
// 10 - YENİ WPS_SLEEP_COUNT (8s x WPS_SLEEP_COUNT)
// 11 - YENİ VECTOR_VARIANCE
// 12 - YENİ VEHICLE_DETECTION_COUNT & BLE_TIMEOUT (kaç tane 8sn aralİk gözleyecek (0-3 Bit) -  BLE timeout (4-7 bit))
// 13 - YENİ RADIO_TX_POWER & BLE POWER
// 14 - YENİ LORA CHANNEL





// HMC için GAIN DEĞERLERİ
#define GAIN_088 0x00 //LSM303C için 4 gauss
#define GAIN_130 0x20 //LSM303C için 8 gauss 
#define GAIN_190 0x40 //LM303C için 12 gauss
#define GAIN_250 0x60 //LSM303C için 16 gauss
#define GAIN_400 0x80 
#define GAIN_470 0xA0
#define GAIN_560 0xC0
#define GAIN_810 0xE0
#define GAIN_4 0x00
#define GAIN_8 0x20
#define GAIN_12 0x40
#define GAIN_16 0x60
