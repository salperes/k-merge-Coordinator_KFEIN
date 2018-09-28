#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include<time.h>
#include <thread>
#include <mutex>
#include <atomic>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <vector>

#include <string.h>
#include <stdlib.h>
#include <cstdlib>

#include<errno.h> //errno
#include<sys/socket.h>
#include<netdb.h>
#include<ifaddrs.h>
#include<unistd.h>
#include<netinet/in.h>//INADDR_ANY
#include<arpa/inet.h>


//#include <ads1115.h>
//#include <wiringPi.h>

#include <inttypes.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

//#include <python2.7/Python.h>

#include <RH_RF95.h>


#include "ConfigFile.h"

#include <mosquitto.h>

using namespace std;

// SPI üzerinden RFM95 kullanımı için tanımlar...
#define BOARD_LORASPI

#define RF_CS_PIN  RPI_V2_GPIO_P1_24 // Slave Select on CE0 so P1 connector pin #24
#define RF_IRQ_PIN RPI_V2_GPIO_P1_22 // IRQ on GPIO25 so P1 connector pin #22
#define RF_RST_PIN RPI_V2_GPIO_P1_15 // RST on GPIO22 so P1 connector pin #15

#define NOT_FIN_PACKET true
#define FIN_PACKET false

#define MSG_INITIAL 0
#define MSG_PERIODICAL 1
#define MSG_WPS_LOST 2
#define MSG_VEHICLE_STATUS 3
#define MSG_INTERRIM 4

char TIMESTMP_COMPUTER[19];							// zaman damgası değişken tanımı

// ----------------------------------------------------------------------------------------------
// ------------------------                               ---------------------------------------
//							WPS LOST VARIABLES AND CLASSES
// ------------------------                               ---------------------------------------
// ----------------------------------------------------------------------------------------------
uint8_t NUMBER_OF_LOST_WPS;
uint8_t DETECTED_WPS = 0;
int WPSSignalSending = 0; // Paket yollayan WPS sayısı
						  //uint8_t WPSRegistered = 0; // registered WPS dosyasından okunan WPS sayısı
unsigned long LostCheckTimer = 0;		// lost wps için timeout sayacı. olarak çalışıyor.
										//uint8_t WPS_ID_CORRECTED = 0; //COORDINATOR ADRESINE GÖRE DÜZELTİLMİŞ WPS ID ...LOST CHECK İŞLEMİ İÇİN


/////////////////////////////////////////////////////////////////////////
//							NEW DEFINITIONS 
/////////////////////////////////////////////////////////////////////////

#pragma pack(1)
struct sWPSData
{
	// DYNAMIC VALUES (Bytes 00-15)
	uint8_t nw_id;
	uint8_t flagtype;
	uint8_t sequence3;
	uint8_t sequence2;
	uint8_t sequence1;
	uint16_t temp;
	uint16_t voltage;
	uint8_t rssi;
	uint8_t error_vehicle;
	uint8_t retry_variance;
	float vector;

	// STATIC VALUES (Bytes 16-30)
	uint16_t versions;
	uint32_t serialble;
	uint16_t serialmsb;
	uint8_t mag_varordiff;
	uint8_t varlimits;
	uint8_t rssilimits;
	uint8_t blepw_radiopw_chkrssi;
	uint8_t vehicledc_sleepc;
	uint8_t lorachannel;
	uint8_t diffmultp_bleto_gain;
	uint8_t checksum;

	/*
	// LOST CHECK VARIABLES
	bool IsLost;
	bool WasLost;
	bool OnRegisteredWPSFile;
	bool SignalSending;
	uint64_t WPSSerial;
	unsigned long LostTimer;
	*/
}wpsdata;
#pragma pack()

#pragma pack(1)
struct sLostStruct
{
	// LOST CHECK VARIABLES
	bool IsLost;
	bool WasLost;
	bool OnRegisteredWPSFile;
	bool SignalSending;
	uint64_t WPSSerial;
	unsigned long LostTimer;
};
#pragma pack()

#pragma pack(1)
struct sCoordData
{
	// DYNAMIC VALUES (Bytes 00-03)
	uint8_t nw_id;
	uint8_t flagtype;
	uint8_t rssi;
	uint8_t changerq;
	uint8_t changeflag;
	// CHANGE PARAMETERS (Bytes 16-30)
	uint8_t mag_varordiff;
	uint8_t varlimits;
	uint8_t rssilimits;
	uint8_t blepw_radiopw_chkrssi;
	uint8_t vehicledc_sleepc;
	uint8_t lorachannel;
	uint8_t diffmultp_bleto_gain;
	uint8_t wpsaddress;
}coordinatordata;
#pragma pack()

#pragma pack(1)
struct sCoordinator_Config
{
	float frq;
	uint8_t registered_wps;
	uint8_t lost_wps_count; // LOST WPS SENSOR COUNT için initial değer....
	uint8_t max_wps;
	uint8_t address;
	uint8_t NW_id;
	uint8_t HWversion;
	uint8_t SWversion_Minor;
	uint8_t SWversion_Major;
	uint32_t serial;
	uint16_t secret;
	char ServerAddress[256];
	uint16_t ServerPort;
	uint8_t power;
	uint8_t wps_lost_cycle; //NUMBER OF TRANSMISION CYCLES TO ASSUME WPS LOST
	char mqtt_username[40];
	char mqtt_password[40];
	char mqtt_topic[40];
	uint32_t IPaddress;
};
#pragma pack()
sCoordinator_Config coordinator;


//////////////////////////////////////////////////////
//                  MQTT STRUCTS
//////////////////////////////////////////////////////
#pragma pack(1)
struct sMQTTPreamble
{
	//11 bytes
	uint8_t	MessageType;
	uint32_t Serial;
	// COORDINATOR TIME
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Min;
	uint8_t Sec;
}preamble;
#pragma pack()

#pragma pack(1)
struct sMQTTCoordinatorHealth
{
	//12 bytes
	float Voltage;
	float Temp;
	float Hum;
}coordhealth;
#pragma pack()

#pragma pack(1)
struct sMQTTCoordinatorConfig
{
	//15 bytes
	uint8_t Address;				// 09 - COORDINATOR_ADDRESS
	uint8_t NW_id;					// 10 - NETWORK ID
	uint8_t HWversion;				// 11 - HW_VERSION
	uint8_t SWversion_Major;			// 12 - SW_VERSION MAJOR
	uint8_t SWversion_Minor;			// 13 - SW_VERSION MINOR
	uint8_t Radiopower;					// 14 - RADIO POWER
	uint8_t max_wps;					// 15 - MAX WPS
	uint8_t wps_lost_cycle;			// 16 - WPS LOST CYCLE
	uint8_t LORAChannel;

	uint8_t lost_wps_count;			// 21 - LOST WPS COUNT
	uint8_t registered_wps;			// 22 - REGISTERED WPS COUNT
	uint32_t IPaddress;
};
#pragma pack()

#pragma pack(1)
struct sMQTTWPSinterrim
{
	// 19 Bytes
	uint8_t nw_id;
	uint32_t sequence;
	uint16_t temp;
	uint16_t voltage;
	uint8_t rssi;
	uint8_t error;
	uint8_t vehicle;
	uint8_t retrycount;
	uint8_t variance;
	float vector;
	uint8_t coordrssi;
};
#pragma pack()

#pragma pack(1)
struct sMQTTWPScfg
{
	// 28 bytes
	uint8_t hwvers;
	uint8_t swvermaj;
	uint8_t swvermin;
	uint64_t serial;
	uint8_t magomxy;
	uint8_t magomz;
	uint8_t magfrq;
	uint8_t maggain;
	uint8_t varordiff;
	uint8_t varlimit;
	uint8_t varbase;

	uint8_t rssilimit;
	uint8_t rssibase;
	uint8_t blepw;
	uint8_t radiopw;
	uint8_t chkrssi;
	uint8_t vehicledc;
	uint8_t sleepc;
	uint8_t lorachannel;
	uint8_t diffmultp;
	uint8_t bleto;

};
#pragma pack()

#pragma pack(1)
struct sMQTTinitial
{
	sMQTTPreamble preamble; //11 bytes
	sMQTTCoordinatorHealth coordhealth; //12 bytes
	//sMQTTCoordinatorConfig coordcfg; //15 bytes
	sCoordinator_Config coordcfg;
}mqttinitial;
#pragma pack()

#pragma pack(1)
struct sMQTTPeriodical
{
	sMQTTPreamble preamble;//11 bytes
	sMQTTCoordinatorHealth coordhealth; //12 bytes
	sMQTTWPSinterrim interrim;//19 bytes
	sMQTTWPScfg wpscfg;//28 bytes
}mqttperiodical;
#pragma pack()

#pragma pack(1)
struct sMQTTinterrim
{
	sMQTTPreamble preamble;//11 bytes
	sMQTTCoordinatorHealth coordhealth;//12 bytes
	sMQTTWPSinterrim interrim;//19 bytes
}mqttinterrim;
#pragma pack()

// ----------------------------------------------------------------------------------------------
// ------------------------                               ---------------------------------------
//							      RADIO DEFINITIONS
// ------------------------                               ---------------------------------------
// ----------------------------------------------------------------------------------------------
// Create an instance of a driver
RH_RF95 driver(RF_CS_PIN, RF_IRQ_PIN);


#pragma pack(1)
struct sLORAcomm
{
	uint8_t WPSAddress;
	bool ACTSend;
	bool FINReceived;
	bool MQTTmsgSend;
	sWPSData wpsdata;
	sLostStruct loststruct;
	sCoordData coordinatordata;
};
#pragma pack()

sLORAcomm LORAcomm[255];


// MQTT TANIMLAMASI
struct mosquitto *mosq = NULL;
char client_id[] = { 'c','o','o','r','d','-','0','0','0','0','0','0','\0' }; //MQTT Client ID ilk değeri
std::string MQTTtopic = "wpsdata";
std::string MQTTtopic2 = "mqttdata";
//Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;
void sig_handler(int sig)
{
	printf("\n%s Break received, exiting!\n", __BASEFILE__);
	force_exit = true;
}


// ----------------------------------------------------------------------------------------------
// ------------------------                               ---------------------------------------
//							  FONKSİYON DEKLARASYONLARI
// ------------------------                               ---------------------------------------
// ----------------------------------------------------------------------------------------------
char* datetime_f(char dates[]);						// zamanı string haline getirmek için
char* int_to_str(int i, char b[], int format_, int Base);		// datetime_f için kullanılan fonksiyn
bool LORARead(); //WPS üzerinden paket okumak ve sonlandırmak için
void SPI_setup(); // SPI setup işlemleri
void RF95_setup(uint8_t RadioPower, uint8_t Coordinator_Address, uint8_t Network_ID, float RFFrequency); //Radyo setup işlemleri
bool SendToWPS(uint8_t LORAWPSindex); // WPS üzerine veri yollama fonksiyonu
void WPS_DATA_print(uint8_t printindex, uint8_t len, uint8_t from, uint8_t to, uint8_t id, uint8_t flags, int16_t Rssi, uint8_t checksum, uint8_t checksumreceived); // debug print
void PrepareDataToWPS(uint8_t LORAWPSindex, uint8_t flagtype, int16_t Rssi); // WPS'e yollanacak verileri hazırlama fonksiyonu
bool PrepareCoordinator(); // Program çalıştırıldığında coordinator.conf dosyası okuma ve yazma
bool ReadCoordinatorConfig(); //coordinator.conf read için
bool WriteConfig(); //coordinator.conf write için... config değiştiğinde tüm dosyayı yeniden oluşturacak
void WPSLostCheckINIT(uint8_t wpscount);
void LostCheck();
bool RWRegisteredWPSCfg(char ReadWrite, uint8_t WPSAddress);
//newver uint8_t LORAPacketIndex(uint8_t from, bool recivedpacket);
bool SendMessageMQTT(uint8_t LORAWPSindexOnFIN, uint8_t MessageType);
void FindIPAddress();
uint32_t hex2dec(const char*hexvalue, int lenght);
void test_output();
void ReadMqtt();

uint16_t readvoltage();

uint8_t test_msqtt[54];

uint8_t WPSVoltageLevel(uint16_t voltage);
//Main Function
int main(int argc, const char* argv[])
{
	//cout << endl << " Battery Voltage = " << (float)readvoltage()*23.48 / 32768 << " V" << endl;
	//cout << endl << " Battery Voltage = " << (float)readvoltage()*23.48 / 32768 << " V" << endl;
	cout << endl << " Battery Voltage = " << (float)readvoltage()*23.48 / 32768 << " V" << endl;

	if (!PrepareCoordinator())
	{
		printf(" Coordinator Configuation File Error...");
		exit;
	}

	FindIPAddress();

	//LORAcomm.resize(coordinator.max_wps);

	//WPSLostCheckStruct.resize(coordinator.max_wps);
	//WPSLostCheckINIT(coordinator.max_wps);


	RWRegisteredWPSCfg('R', 0);

	//wpsnewconfig.resize(coordinator.max_wps);

	//MQTT ClientID, Coordinator.Serial ile birlikte oluşturulacak ..


	char stringnumber[6];
	int_to_str(coordinator.serial, stringnumber, 1, 16);

	for (int i = 0; i < 6; i++)
	{
		client_id[11 - i] = stringnumber[5 - i];
	}
	printf(" MQTT Client ID	: %s\n", client_id);
	printf(" MQTT Server	: %s\n", coordinator.ServerAddress);
	printf(" MQTT Port	: %d\n", coordinator.ServerPort);



	SendMessageMQTT(0, MSG_INITIAL);

	SPI_setup();
	mosquitto_disconnect(mosq);
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	//bool useRFO = true;
	//coordinator.power = 23;
	if (!driver.init())
	{
		fprintf(stderr, "\nRF95 module init failed, Please verify wiring/module\n");
	}
	else
	{
		RF95_setup(coordinator.power, coordinator.address, coordinator.NW_id, coordinator.frq);
		while (!force_exit)
		{

			// LostCheck işlemi için timer ayarı 60 sn
			if ((millis() - LostCheckTimer) > 60000) // 300 sn'de bir Lost_Check() yapılacak
			{
				LostCheck();
				LostCheckTimer = millis();
				//if (LostCheckTimer > 4294967295 4294900000) LostCheckTimer = 0;
			}



#ifdef RF_IRQ_PIN
			// We have a IRQ pin ,pool it instead reading
			// Modules IRQ registers from SPI in each loop
			// Rising edge fired ?
			if (bcm2835_gpio_eds(RF_IRQ_PIN))
			{
				// Now clear the eds flag by setting it to 1
				bcm2835_gpio_set_eds(RF_IRQ_PIN);
				//printf("Packet Received, Rising event detect for pin GPIO%d\n", RF_IRQ_PIN);
#endif

				if (driver.available())
				{
					LORARead();
				}

#ifdef RF_IRQ_PIN
			}
#endif

			// Let OS doing other tasks
			// For timed critical appliation you can reduce or delete
			// this delay, but this will charge CPU usage, take care and monitor
			bcm2835_delay(500);
		}
	}

	printf("\n%s Ending\n", __BASEFILE__);
	bcm2835_close();
	return 0;
}

uint16_t readvoltage()
{

	int fd;
	int ads_address = 0x48;
	uint8_t buf[10];
	int16_t val;

	// open device on /dev/i2c-1 the default on Raspberry Pi B
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		printf("Error: Couldn't open device! %d\n", fd);
		return 1;
	}
	usleep(10000);
	// connect to ads1115 as i2c slave
	if (ioctl(fd, I2C_SLAVE, ads_address) < 0) {
		printf("Error: Couldn't find device on address!\n");
		return 1;
	}
	usleep(10000);
	///////////////////////////////
	// set config register and start conversion
	// AIN0 and GND, 4.096v, 128s/s
	buf[0] = 1;    // config register is 1
	buf[1] = 0xc3;
	buf[2] = 0x85;
	if (write(fd, buf, 3) != 3) {
		perror("Write to register 1");
		return 1;//exit(-1);
	}
	usleep(10000);
	//////////////////////////////
	// wait for conversion complete
	do {
		if (read(fd, buf, 2) != 2) {
			perror("Read conversion");
			return 1;//exit(-1);exit(-1);
		}
	} while (!(buf[0] & 0x80));
	usleep(10000);
	//////////////////////////////
	// read conversion register
	buf[0] = 0;   // conversion register is 0
	if (write(fd, buf, 1) != 1) {
		perror("Write register select");
		return 1;//exit(-1);exit(-1);
	}
	usleep(10000);
	if (read(fd, buf, 2) != 2) {
		perror("Read conversion");
		return 1;//exit(-1);exit(-1);
	}

	//////////////////////////////
	// convert output and display results
	val = (int16_t)buf[0] * 256 + (uint16_t)buf[1];
	//printf("Conversion %02x %02x %d %f\n", buf[0], buf[1], val, (float)val*23.48 / 32768);

	close(fd);


	return val;


}

uint8_t WPSVoltageLevel(uint16_t voltage)
{
	//printf("MSB = %d , LSB = %d", MSB_, LSB_);

	float VoltageLevel = (float)(voltage*3.3 * 2 / 1024);
	//printf("MSB = %d , LSB = %d, Voltage=%f3.2", MSB_, LSB_,VoltageLevel);
	float delta = 0;


	if (VoltageLevel >= 6) delta = 100;

	if (VoltageLevel < 6 && VoltageLevel >= 5.6)
	{
		delta = 96 + (100 * 4 * (VoltageLevel - 5.6) / 40);
		if (delta > 100) delta = 100;
	}

	if (VoltageLevel < 5.6 && VoltageLevel >= 5.12)
	{
		delta = 83 + (100 * 13 * (VoltageLevel - 5.12) / 48);
		if (delta > 96) delta = 96;
	}

	if (VoltageLevel < 5.12 && VoltageLevel >= 4)
	{
		delta = 17 + (100 * 66 * (VoltageLevel - 4) / 112);
		if (delta > 83) delta = 83;

		//data_to_coordinator[11] = 17 + (uint8_t)delta;
	}

	if (VoltageLevel < 4 && VoltageLevel >= 3.2)
	{
		delta = 100 * 17 * (VoltageLevel - 3, 2) / 80;
		if (delta > 17) delta = 17;
		//data_to_coordinator[11] = (uint8_t)delta;
	}
	if (VoltageLevel < 3.2) delta = 0;
	round(VoltageLevel);
	return (uint8_t)delta;

}

bool SendMessageMQTT(uint8_t index, uint8_t MessageType)
{
	//return 0;

	// Initialize the Mosquitto library
	mosquitto_lib_init();

	mosq = mosquitto_new(client_id, true, NULL);
	mosquitto_username_pw_set(mosq, coordinator.mqtt_username, coordinator.mqtt_password);


	// Establish a connection to the MQTT server. Do not use a keep-alive ping
	int ret = mosquitto_connect(mosq, coordinator.ServerAddress, coordinator.ServerPort, 0);
	if (ret)
	{
		fprintf(stderr, "Can't connect to Mosquitto server\n");
		return false;
	}

	// MQTT Paket hazırlama
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);

	int MQTTMsgLen = 0;

	switch (MessageType)
	{
	case MSG_INITIAL:
	{
		MQTTMsgLen = 28;
		MQTTtopic = "wpsdata";
		break;
	}
	case MSG_PERIODICAL:
	{
		MQTTMsgLen = 59;
		MQTTtopic = "wpsdata";
		break;
	}
	case MSG_VEHICLE_STATUS:
	{
		MQTTMsgLen = 59;
		MQTTtopic = "wpsdata";
		break;
	}
	case MSG_WPS_LOST:
	{
		MQTTMsgLen = 17;
		MQTTtopic = "wpslost";
		break;
	}
	}

	char MQTTMsg[MQTTMsgLen];


	// MQTT MESSAGE PREAMBLE
	MQTTMsg[0] = (uint8_t)MessageType;						// 00 - PACKET TYPE
	MQTTMsg[1] = (uint8_t)(coordinator.serial >> 8);		// 01 - COORDINATOR_SERIAL_MSB
	MQTTMsg[2] = (uint8_t)(coordinator.serial);				// 02 - COORDINATOR_SERIAL_LSB
	MQTTMsg[3] = (uint8_t)(now->tm_year);					// 03 - COORDINATOR TIME YEAR
	MQTTMsg[4] = (uint8_t)(now->tm_mon + 1);				// 04 - COORDINATOR TIME MONTH
	MQTTMsg[5] = (uint8_t)now->tm_mday;						// 05 - COORDINATOR TIME DAY
	MQTTMsg[6] = (uint8_t)now->tm_hour;						// 06 - COORDINATOR TIME HOUR
	MQTTMsg[7] = (uint8_t)now->tm_min;						// 07 - COORDINATOR TIME MINUTE
	MQTTMsg[8] = (uint8_t)now->tm_sec;						// 08 - COORDINATOR TIME SECOND

	/////////////////////////////////////////////////////
	//			 NEW MQTT STRUCTURE PREAMBLE
	/////////////////////////////////////////////////////
	preamble.MessageType = MSG_INITIAL;
	preamble.Serial = coordinator.serial;
	preamble.Year = (uint8_t)(now->tm_year);
	preamble.Month = (uint8_t)(now->tm_mon + 1);
	preamble.Day = (uint8_t)now->tm_mday;
	preamble.Hour = (uint8_t)now->tm_hour;
	preamble.Min = (uint8_t)now->tm_min;
	preamble.Sec = (uint8_t)now->tm_sec;

	/////////////////////////////////////////////////////
	//			 NEW MQTT STRUCTURE HEALTH
	/////////////////////////////////////////////////////
	coordhealth.Voltage = readvoltage()*23.48 / 32768;
	coordhealth.Temp = 0;
	coordhealth.Hum = 0;



	switch (MessageType)
	{
	case MSG_INITIAL:
	{
		MQTTMsg[9] = (uint8_t)coordinator.address;					// 09 - COORDINATOR_ADDRESS
		MQTTMsg[10] = (uint8_t)coordinator.NW_id;					// 10 - NETWORK ID
		MQTTMsg[11] = (uint8_t)coordinator.HWversion;				// 11 - HW_VERSION
		MQTTMsg[12] = (uint8_t)coordinator.SWversion_Major;			// 12 - SW_VERSION MAJOR
		MQTTMsg[13] = (uint8_t)coordinator.SWversion_Minor;			// 13 - SW_VERSION MINOR
		MQTTMsg[14] = (uint8_t)coordinator.power;					// 14 - RADIO POWER
		MQTTMsg[15] = (uint8_t)coordinator.max_wps;					// 15 - MAX WPS
		MQTTMsg[16] = (uint8_t)coordinator.wps_lost_cycle;			// 16 - WPS LOST CYCLE
		uint8_t *p = (uint8_t*)&coordinator.frq;
		for (int i = 0; i < sizeof(coordinator.frq); i++)
		{
			MQTTMsg[17 + i] = p[i];
		}
		MQTTMsg[21] = (uint8_t)coordinator.lost_wps_count;			// 21 - LOST WPS COUNT
		MQTTMsg[22] = (uint8_t)coordinator.registered_wps;			// 22 - REGISTERED WPS COUNT
		MQTTMsg[23] = (uint8_t)(coordinator.IPaddress >> 24);		// 23 - IP ADDRESS 1st OCTET
		MQTTMsg[24] = (uint8_t)(coordinator.IPaddress >> 16);		// 24 - IP ADDRESS 2nd OCTET
		MQTTMsg[25] = (uint8_t)(coordinator.IPaddress >> 8);		// 25 - IP ADDRESS 3rd OCTET
		MQTTMsg[26] = (uint8_t)(coordinator.IPaddress);				// 26 - IP ADDRESS 4th OCTET
		MQTTMsg[27] = (uint8_t)(coordinator.serial >> 16);			// 27 - SERIAL 3rd OCTET

	/////////////////////////////////////////////////////
	//			 NEW MQTT STRUCTURE INITIAL
	/////////////////////////////////////////////////////
		mqttinitial.preamble = preamble;
		mqttinitial.coordhealth = coordhealth;
		mqttinitial.coordcfg = coordinator;

		ret = mosquitto_publish(mosq, NULL, MQTTtopic2.c_str(), sizeof(mqttinitial), (uint8_t*)&mqttinitial, 0, false);
		if (ret)
		{
			fprintf(stderr, "Can't publish to Mosquitto server\n");
			//return false;
			//delete[] MQTTtopic;
			//exit(-1);
		}
		else fprintf(stderr, "MQTT Published \n");


		break;

	}
	case MSG_PERIODICAL:

	{
		MQTTMsg[9] = (uint8_t)coordinator.address;											// 09 - COORDINATOR_ADDRESS
		uint16_t voltage = readvoltage();

		MQTTMsg[10] = voltage >> 8;															// 10 - COORDINATOR_BATTERY_VALUE MSB
		MQTTMsg[11] = voltage;																// 11 - COORDINATOR_BATTERY_VALUE LSB
		MQTTMsg[12] = (uint8_t)coordinator.SWversion_Major;									// 12 - COORDINATOR SW_VERSION MAJOR
		MQTTMsg[13] = (uint8_t)coordinator.SWversion_Minor;									// 13 - COORDINATOR SW_VERSION MINOR
		MQTTMsg[14] = (uint8_t)coordinator.HWversion;										// 14 - COORDINATOR HW_VERSION
		MQTTMsg[15] = (uint8_t)LORAcomm[index].coordinatordata.rssi;						// 15 - COORDINATOR RSSI ---- DÜZELTİP YOLLA.

		MQTTMsg[16] = (uint8_t)LORAcomm[index].wpsdata.nw_id;								// 16 - WPS NW ID
		MQTTMsg[17] = (uint8_t)LORAcomm[index].WPSAddress;									// 17 - WPS ADDRESS
		MQTTMsg[18] = (uint8_t)LORAcomm[index].wpsdata.sequence3;							// 18 - WPS Sequence 3rd Octet
		MQTTMsg[19] = (uint8_t)LORAcomm[index].wpsdata.sequence2;							// 19 - WPS Sequence 2nd Octet
		MQTTMsg[20] = (uint8_t)LORAcomm[index].wpsdata.sequence1;							// 20 - WPS Sequence 1st Octet

		MQTTMsg[21] = (uint8_t)(LORAcomm[index].wpsdata.temp >> 8);							// 21 - WPS SENSOR TEMP MSB
		MQTTMsg[22] = (uint8_t)LORAcomm[index].wpsdata.temp;								// 22 - WPS SENSOR TEMP LSB

		MQTTMsg[23] = (uint8_t)(LORAcomm[index].wpsdata.serialmsb >> 8);						// 23 - WPS Serial 6th Octet
		MQTTMsg[24] = (uint8_t)LORAcomm[index].wpsdata.serialmsb;							// 24 - WPS Serial 5th Octet
		MQTTMsg[25] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 24);					// 25 - WPS Serial 4th Octet
		MQTTMsg[26] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 16);					// 26 - WPS Serial 3rd Octet
		MQTTMsg[27] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 8);					// 27 - WPS Serial 2nd Octet
		MQTTMsg[28] = (uint8_t)LORAcomm[index].wpsdata.serialble;							// 28 - WPS Serial 1st Octet
		MQTTMsg[29] = (uint8_t)(LORAcomm[index].wpsdata.retry_variance & 0b11110000) >> 4;	// 29 - WPS VECTOR VARIANCE
		MQTTMsg[30] = (uint8_t)(LORAcomm[index].wpsdata.error_vehicle & 0b10000000) >> 7;	// 30 - WPS Vehicle Status

		MQTTMsg[31] = (uint8_t)(LORAcomm[index].wpsdata.voltage >> 8);						// 31 - WPS BATT VALUE MSB
		MQTTMsg[32] = (uint8_t)LORAcomm[index].wpsdata.voltage;								// 32 - WPS BATT VALUE LSB


		MQTTMsg[33] = WPSVoltageLevel(LORAcomm[index].wpsdata.voltage);						// 33 - WPS_BATTERY_LEVEL

		uint16_t temprssi = (-1)*(int16_t)((uint8_t)LORAcomm[index].wpsdata.rssi);
		MQTTMsg[34] = (uint8_t)(temprssi >> 8);												// 34 - WPS RSSI MSB
		MQTTMsg[35] = (uint8_t)(temprssi);													// 35 - WPS RSSI LSB


		/*float vector;
		uint8_t *vec_pointer = (uint8_t*)&vector;
		for (uint8_t i = 0; i < sizeof(vector); i++)
		{
			vec_pointer[i] = LORAcomm[printindex].data_from_wps[28 + i];
		}*/

		MQTTMsg[36] = 0;// (uint8_t)(LORAcomm[index].wpsdata.vector >> 24);						// 36 - VECTOR 4th 
		MQTTMsg[37] = 0;//(uint8_t)(LORAcomm[index].wpsdata.vector >> 16);						// 37 - VECTOR 3rd
		MQTTMsg[38] = 0;//(uint8_t)(LORAcomm[index].wpsdata.vector >> 8);						// 38 - VECTOR 2nd

		MQTTMsg[39] = (uint8_t)(LORAcomm[index].wpsdata.versions >> 12);					// 39 - WPS HW Version
		MQTTMsg[40] = (uint8_t)(LORAcomm[index].wpsdata.versions >> 8) & 0b00001111;		// 40 - WPS SW Version Major
		MQTTMsg[41] = (uint8_t)LORAcomm[index].wpsdata.versions & 0b00111111;				// 41 - WPS SW Version Minor
		MQTTMsg[42] = (uint8_t)(LORAcomm[index].wpsdata.varlimits >> 3);					// 42 - WPS Vector Variance Limit
		MQTTMsg[43] = (uint8_t)(LORAcomm[index].wpsdata.vehicledc_sleepc & 0b11000000) >> 6;// 43 - WPS Vehicle Detection Count
		MQTTMsg[44] = (uint8_t)LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b00011111;	// 44 - WPS Radio TX Power
		MQTTMsg[45] = (uint8_t)(LORAcomm[index].wpsdata.diffmultp_bleto_gain & 0b00001100) >> 2;	// 45 - WPS Sensor Gain
		MQTTMsg[46] = (uint8_t)(LORAcomm[index].wpsdata.vehicledc_sleepc & 0b00111111); 	// 46 - WPS Sleep Count
		MQTTMsg[47] = (uint8_t)(LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b01100000) >> 5; 	// 47 - BLE TX POWER
		MQTTMsg[48] = (uint8_t)LORAcomm[index].wpsdata.lorachannel;							// 48 - LORA CHANNEL 
		MQTTMsg[49] = (uint8_t)(LORAcomm[index].wpsdata.vector);							// 49 - VECTOR 1st
		MQTTMsg[50] = (uint8_t)(coordinator.serial >> 16);									// 50 - COORDINATOR SERIAL 3rd OCTET
		MQTTMsg[51] = (uint8_t)LORAcomm[index].wpsdata.error_vehicle & 0b01111111;			// 51 - WPS ERROR CODE
		MQTTMsg[52] = ((uint8_t)LORAcomm[index].wpsdata.diffmultp_bleto_gain) >> 4;			// 52 - BLE TIMEOUT
		MQTTMsg[53] = (uint8_t)LORAcomm[index].wpsdata.retry_variance & 0b00001111;			// 53 - LoRA RETRY COUNT
		MQTTMsg[54] = ((uint8_t)LORAcomm[index].wpsdata.varlimits & 0b00000111);			// 54 - MIN_VARIANCE
		MQTTMsg[55] = ((uint8_t)LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b10000000) >> 7;	// 55 - CHECK_RSSI
		MQTTMsg[56] = ((uint8_t)LORAcomm[index].wpsdata.rssilimits & 0b00000111);			// 56 - MIN_RSSI_DIFF
		MQTTMsg[57] = ((uint8_t)LORAcomm[index].wpsdata.rssilimits & 0b11111000) >> 3;		// 57 - MAX_RSSI_DIFF
		MQTTMsg[58] = ((uint8_t)LORAcomm[index].wpsdata.mag_varordiff & 0b01111111);		// 58 - MAG_FRQ, MAG_OMZ, MAG_OMXY

		/////////////////////////////////////////////////////
		//			 NEW MQTT STRUCTURE PERIODICAL
		/////////////////////////////////////////////////////
		preamble.MessageType = MSG_PERIODICAL;
		mqttperiodical.preamble = preamble;
		mqttperiodical.coordhealth = coordhealth;
		mqttperiodical.interrim.nw_id = LORAcomm[index].wpsdata.nw_id;
		mqttperiodical.interrim.sequence = (uint32_t)LORAcomm[index].wpsdata.sequence3 << 16 | (uint32_t)LORAcomm[index].wpsdata.sequence2 << 8 | (uint32_t)LORAcomm[index].wpsdata.sequence1;
		mqttperiodical.interrim.temp = LORAcomm[index].wpsdata.temp;
		mqttperiodical.interrim.voltage = LORAcomm[index].wpsdata.voltage;
		mqttperiodical.interrim.rssi = LORAcomm[index].wpsdata.rssi;
		mqttperiodical.interrim.error = LORAcomm[index].wpsdata.error_vehicle & 0b01111111;
		mqttperiodical.interrim.vehicle = (LORAcomm[index].wpsdata.error_vehicle & 0b10000000) >> 7;
		mqttperiodical.interrim.retrycount = LORAcomm[index].wpsdata.retry_variance & 0b00001111;
		mqttperiodical.interrim.variance = (LORAcomm[index].wpsdata.retry_variance & 0b11110000) >> 4;
		mqttperiodical.interrim.vector = LORAcomm[index].wpsdata.vector;
		mqttperiodical.interrim.coordrssi = LORAcomm[index].coordinatordata.rssi;

		mqttperiodical.wpscfg.hwvers = (uint8_t)(LORAcomm[index].wpsdata.versions >> 12);
		mqttperiodical.wpscfg.swvermaj = (uint8_t)(LORAcomm[index].wpsdata.versions >> 8) & 0b00001111;
		mqttperiodical.wpscfg.swvermin = (uint8_t)LORAcomm[index].wpsdata.versions & 0b00111111;
		mqttperiodical.wpscfg.serial = (uint64_t)LORAcomm[index].wpsdata.serialmsb << 32 | LORAcomm[index].wpsdata.serialble;
		mqttperiodical.wpscfg.magomxy = (LORAcomm[index].wpsdata.mag_varordiff & 0b00001100) >> 2;
		mqttperiodical.wpscfg.magomz = (LORAcomm[index].wpsdata.mag_varordiff & 0b00000011);
		mqttperiodical.wpscfg.magfrq = (LORAcomm[index].wpsdata.mag_varordiff & 0b01110000) >> 4;
		mqttperiodical.wpscfg.maggain = (LORAcomm[index].wpsdata.diffmultp_bleto_gain & 0b00001100) >> 2;
		mqttperiodical.wpscfg.varordiff = (LORAcomm[index].wpsdata.mag_varordiff & 0b10000000) >> 7;
		mqttperiodical.wpscfg.varlimit = LORAcomm[index].wpsdata.varlimits >> 3;
		mqttperiodical.wpscfg.varbase = LORAcomm[index].wpsdata.varlimits & 0b00000111;
		mqttperiodical.wpscfg.rssilimit = LORAcomm[index].wpsdata.rssilimits & 0b11111000 >> 3;
		mqttperiodical.wpscfg.rssibase = LORAcomm[index].wpsdata.rssilimits & 0b00000111;
		mqttperiodical.wpscfg.blepw = (LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b01100000) >> 5;
		mqttperiodical.wpscfg.radiopw = LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b00011111;
		mqttperiodical.wpscfg.chkrssi = (LORAcomm[index].wpsdata.blepw_radiopw_chkrssi & 0b10000000) >> 7;
		mqttperiodical.wpscfg.vehicledc = (LORAcomm[index].wpsdata.vehicledc_sleepc & 0b11000000) >> 6;
		mqttperiodical.wpscfg.sleepc = LORAcomm[index].wpsdata.vehicledc_sleepc & 0b00111111;
		mqttperiodical.wpscfg.lorachannel = LORAcomm[index].wpsdata.lorachannel;
		mqttperiodical.wpscfg.diffmultp = LORAcomm[index].wpsdata.diffmultp_bleto_gain & 0b00000011;
		mqttperiodical.wpscfg.bleto = (LORAcomm[index].wpsdata.diffmultp_bleto_gain & 0b11110011) >> 4;

		ret = mosquitto_publish(mosq, NULL, MQTTtopic2.c_str(), sizeof(mqttperiodical), (uint8_t*)&mqttperiodical, 0, false);
		if (ret)
		{
			fprintf(stderr, "Can't publish to Mosquitto server\n");
			//return false;
			//delete[] MQTTtopic;
			//exit(-1);
		}
		else fprintf(stderr, "MQTT Published \n");
		break;
	}
	case MSG_WPS_LOST:
	{
		MQTTMsg[9] = (uint8_t)(coordinator.serial >> 16);
		MQTTMsg[10] = LORAcomm[index].WPSAddress;
		MQTTMsg[11] = (uint8_t)(LORAcomm[index].wpsdata.serialmsb >> 8);
		MQTTMsg[12] = (uint8_t)LORAcomm[index].wpsdata.serialmsb;
		MQTTMsg[13] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 24);
		MQTTMsg[14] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 16);
		MQTTMsg[15] = (uint8_t)(LORAcomm[index].wpsdata.serialble >> 8);
		MQTTMsg[16] = (uint8_t)LORAcomm[index].wpsdata.serialble;
		break;
	}

	case MSG_VEHICLE_STATUS:
	{
		preamble.MessageType = MSG_VEHICLE_STATUS;
		mqttinterrim.preamble = preamble;
		mqttinterrim.coordhealth = coordhealth;
		mqttinterrim.interrim.nw_id = LORAcomm[index].wpsdata.nw_id;
		mqttinterrim.interrim.sequence = (uint32_t)LORAcomm[index].wpsdata.sequence3 << 16 | (uint32_t)LORAcomm[index].wpsdata.sequence2 << 8 | (uint32_t)LORAcomm[index].wpsdata.sequence1;
		mqttinterrim.interrim.temp = LORAcomm[index].wpsdata.temp;
		mqttinterrim.interrim.voltage = LORAcomm[index].wpsdata.voltage;
		mqttinterrim.interrim.rssi = LORAcomm[index].wpsdata.rssi;
		mqttinterrim.interrim.error = LORAcomm[index].wpsdata.error_vehicle & 0b01111111;
		mqttinterrim.interrim.vehicle = (LORAcomm[index].wpsdata.error_vehicle & 0b10000000) >> 7;
		mqttinterrim.interrim.retrycount = LORAcomm[index].wpsdata.retry_variance & 0b00001111;
		mqttinterrim.interrim.variance = (LORAcomm[index].wpsdata.retry_variance & 0b11110000) >> 4;
		mqttinterrim.interrim.vector = LORAcomm[index].wpsdata.vector;
		mqttinterrim.interrim.coordrssi = LORAcomm[index].coordinatordata.rssi;

		ret = mosquitto_publish(mosq, NULL, MQTTtopic2.c_str(), sizeof(mqttinterrim), (uint8_t*)&mqttinterrim, 0, false);
		if (ret)
		{
			fprintf(stderr, "Can't publish to Mosquitto server\n");
			//return false;
			//delete[] MQTTtopic;
			//exit(-1);
		}
		else fprintf(stderr, "MQTT Published \n");
		break;
	}
	case MSG_INTERRIM:
	{
		preamble.MessageType = MSG_INTERRIM;
		mqttinterrim.preamble = preamble;
		mqttinterrim.coordhealth = coordhealth;
		mqttinterrim.interrim.nw_id = LORAcomm[index].wpsdata.nw_id;
		mqttinterrim.interrim.sequence = (uint32_t)LORAcomm[index].wpsdata.sequence3 << 16 | (uint32_t)LORAcomm[index].wpsdata.sequence2 << 8 | (uint32_t)LORAcomm[index].wpsdata.sequence1;
		mqttinterrim.interrim.temp = LORAcomm[index].wpsdata.temp;
		mqttinterrim.interrim.voltage = LORAcomm[index].wpsdata.voltage;
		mqttinterrim.interrim.rssi = LORAcomm[index].wpsdata.rssi;
		mqttinterrim.interrim.error = LORAcomm[index].wpsdata.error_vehicle & 0b01111111;
		mqttinterrim.interrim.vehicle = (LORAcomm[index].wpsdata.error_vehicle & 0b10000000) >> 7;
		mqttinterrim.interrim.retrycount = LORAcomm[index].wpsdata.retry_variance & 0b00001111;
		mqttinterrim.interrim.variance = (LORAcomm[index].wpsdata.retry_variance & 0b11110000) >> 4;
		mqttinterrim.interrim.vector = LORAcomm[index].wpsdata.vector;
		mqttinterrim.interrim.coordrssi = LORAcomm[index].coordinatordata.rssi;

		ret = mosquitto_publish(mosq, NULL, MQTTtopic2.c_str(), sizeof(mqttinterrim), (uint8_t*)&mqttinterrim, 0, false);
		if (ret)
		{
			fprintf(stderr, "Can't publish to Mosquitto server\n");
			return false;
			//delete[] MQTTtopic;
			//exit(-1);
		}
		else fprintf(stderr, "MQTT Published \n");
		break;
	}
	}




	//cout << MQTTtopic << endl;
	//cout << MQTTMsg << endl;

	ret = mosquitto_publish(mosq, NULL, MQTTtopic.c_str(), MQTTMsgLen, MQTTMsg, 0, false);
	if (ret)
	{
		fprintf(stderr, "Can't publish to Mosquitto server\n");
		return false;
		//delete[] MQTTtopic;
		//exit(-1);
	}
	else fprintf(stderr, "old MQTT Published \n");



	// We need a short delay here, to prevent the Mosquitto library being
	//  torn down by the operating system before all the network operations
	//  are finished.
	/*sleep(1);


	// Tidy up

	mosquitto_disconnect(mosq);
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	//delete[] MQTTtopic;*/
	return true;
}

void WPSLostCheckINIT(uint8_t wpscount)
{
	for (int i = 0; i < 256; i++)
	{
		LORAcomm[i].loststruct.LostTimer = 1000;
		LORAcomm[i].loststruct.IsLost = 0;
		LORAcomm[i].loststruct.WasLost = 0;
		LORAcomm[i].loststruct.SignalSending = false;
		LORAcomm[i].loststruct.OnRegisteredWPSFile = false;
		LORAcomm[i].loststruct.WPSSerial = 0;
		LORAcomm[i].WPSAddress = 0;
		LORAcomm[i].ACTSend = false;
		LORAcomm[i].FINReceived = false;
		LORAcomm[i].MQTTmsgSend = false;
	}
}

void LostCheck()
{
	//return;
	uint8_t NewLostCount = 0;
	uint8_t NewFoundCount = 0;
	unsigned long delta = 0;

	printf(" LOST CHECK FUNCTION....\n");

	for (int i = 0; i <= 255; i++)
		//newver	for (uint8_t i = 0; i < coordinator.registered_wps; i++)
	{
		if (LORAcomm[i].WPSAddress == 0) continue;
		delta = millis() - LORAcomm[i].loststruct.LostTimer;
		long sleepcount = LORAcomm[i].wpsdata.vehicledc_sleepc & 0b00111111;
		//newver if (delta >((SleepCycleFactor * 8000) * (long)(WPSLostCheckStruct[i].SleepCount)*(long)(coordinator.wps_lost_cycle)))
		if (delta > ((SleepCycleFactor * 8000) * sleepcount*(long)(coordinator.wps_lost_cycle)))
		{

			if (LORAcomm[i].loststruct.IsLost == false)
			{
				WPSSignalSending--;
				NUMBER_OF_LOST_WPS++;

				LORAcomm[i].loststruct.IsLost = true;

				LORAcomm[i].loststruct.SignalSending = false;

				cout << "Lost WPS Packet Will Be Sent.." << endl;
				SendMessageMQTT(i, MSG_WPS_LOST);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}
			printf(" WPS Address # %d # is lost...\n", i);
		}

		if (LORAcomm[i].loststruct.IsLost == true && LORAcomm[i].loststruct.WasLost == false)

		{
			NewLostCount++;//YENİ LOST WPS VAR
			LORAcomm[i].loststruct.WasLost = true;
			printf(" Number of new lost wps : %d\n", NewLostCount);
		}

	}
	printf("\n");
	printf(" LOST CHECK FUNCTION ENDED....\n");
	if (NewLostCount > 0)
	{
		printf("LOST PACKET WILL BE SENT!!!!\n");
		RWRegisteredWPSCfg('W', 0);
		//SendLostPacket();
	}
}

bool RWRegisteredWPSCfg(char ReadWrite, uint8_t WPSAddress)
{
	// READ FILE
	if (ReadWrite == 'R' || ReadWrite == 'r')
	{
		fstream wpsfile;
		wpsfile.open("registeredwps.conf", ios::out | ios::in);
		if (!wpsfile.is_open()) // THERE IS NO registeredwps.conf FILE... CREATE ONE
		{
			cout << "\n Creating registeredwps.conf file for first time...\n";
			wpsfile.open("registeredwps.conf", ios::out | ios::in | ios::trunc);//TRUNCATING to make new file!
			wpsfile.close();
			return true;
		}

		// IF FILE EXIST THEN READ REGISTERED to WPSLostCheckStruct VECTOR
		cout << " \n\n Reading registeredwps.conf File...\n\n";
		const int MAX_CHARS_PER_LINE = 100;
		const int MAX_TOKENS_PER_LINE = 7;
		const char* const DELIMITER = " ";
		// create a file-reading object
		ifstream fin;
		fin.open("registeredwps.conf"); // open a file
		if (!fin.good())
		{
			printf(" NOFILE \n");
			return false; // exit if file not found
		}
		// read each line of the file
		while (!fin.eof())
		{
			// read an entire line into memory
			char buf[MAX_CHARS_PER_LINE];
			fin.getline(buf, MAX_CHARS_PER_LINE);

			// parse the line into blank-delimited tokens
			int n = 0; // a for-loop index

					   // array to store memory addresses of the tokens in buf
			const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0

														 // parse the line
			token[0] = strtok(buf, DELIMITER); // first token
			if (token[0]) // zero if line is blank
			{
				for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
				{
					token[n] = strtok(0, DELIMITER); // subsequent tokens
					if (!token[n]) break; // no more tokens
				}
			}

			// process (print) the tokens
			for (int i = 0; i < n; i++) // n = #of tokens
			{
				//	cout << "Token[" << i << "] = " << token[i] << " Numeric : "<< (uint16_t)(atoi(token[i])) <<endl;



				if (string(token[i]) == "#")
				{

					uint8_t wpsaddress = atoi(token[1]);

					uint16_t msbserial = atoi(token[2]);
					uint32_t bleserial = atoi(token[3]);
					uint8_t sleepcount = (uint8_t)atoi(token[4]);
					uint8_t islost = (uint8_t)atoi(token[5]);
					uint8_t waslost = (uint8_t)atoi(token[6]);
					uint64_t wpsserial = (uint64_t)msbserial << 24 | bleserial;

					LORAcomm[wpsaddress].loststruct.WPSSerial = (uint64_t)msbserial << 32 | bleserial;
					LORAcomm[wpsaddress].WPSAddress = wpsaddress;
					LORAcomm[wpsaddress].wpsdata.serialmsb = msbserial;
					LORAcomm[wpsaddress].wpsdata.serialble = bleserial;
					LORAcomm[wpsaddress].wpsdata.vehicledc_sleepc = (LORAcomm[i].wpsdata.vehicledc_sleepc & 0b11000000) | sleepcount;
					LORAcomm[wpsaddress].loststruct.IsLost = islost;
					LORAcomm[wpsaddress].loststruct.WasLost = waslost;
					LORAcomm[wpsaddress].loststruct.OnRegisteredWPSFile = true;
					LORAcomm[wpsaddress].loststruct.SignalSending = false;



					printf("%d.	WPS Address =  %d	- WPS Serial = %.4X:%.8X	- WPS SleepCount = %d	- WPS Lost = ",
						(coordinator.registered_wps + 1), wpsaddress, msbserial, bleserial, sleepcount);


					if (LORAcomm[wpsaddress].loststruct.IsLost == 0)

					{
						printf("FALSE\n");
					}
					else
					{
						printf("TRUE\n");
						NUMBER_OF_LOST_WPS++;
					}


					LORAcomm[wpsaddress].WPSAddress = wpsaddress;
					LORAcomm[wpsaddress].FINReceived = false;
					LORAcomm[wpsaddress].ACTSend = false;
					coordinator.registered_wps++;
				}
			}
		}
		printf(" Registered =	%d			Live =	%d			Lost =	%d\n", coordinator.registered_wps, WPSSignalSending, NUMBER_OF_LOST_WPS);
	}
	// APPEND FILE
	if (ReadWrite == 'A' || ReadWrite == 'a')
	{

		//newver	for (uint8_t i = 0; i < coordinator.max_wps; i++)
		//newver	{
		//newver		if (WPSLostCheckStruct[i].WPSAddress == WPSAddress)  // WPSAddress ile WPSLostCheckStruct.WPSAddress eşitliğine bak
		//newver		if (WPSLostCheckStruct[i].OnRegisteredWPSFile) return true; // && !WPSLostCheckStruct[i].StatusChange) return true; // dosyada var ve status değişimi yoksa çık..
		if (LORAcomm[WPSAddress].loststruct.OnRegisteredWPSFile) return true;

		//newver	if (!WPSLostCheckStruct[i].OnRegisteredWPSFile) // dosya içerisinde yoksa dosyaya ekle...
		//newver		{
		fstream wpsfile;
		char buffer[10];

		wpsfile.open("registeredwps.conf", ios::out | ios::app);
		wpsfile << "# ";

		//newver int_to_str(WPSLostCheckStruct[i].WPSAddress, buffer, 0, 10);
		int_to_str(LORAcomm[WPSAddress].WPSAddress, buffer, 0, 10);
		wpsfile << buffer << " ";

		//newver int_to_str(WPSLostCheckStruct[i].serialmsb, buffer, 0, 10);
		int_to_str(LORAcomm[WPSAddress].wpsdata.serialmsb, buffer, 0, 10);

		wpsfile << buffer << " ";
		//newver int_to_str(WPSLostCheckStruct[i].seriallsb, buffer, 0, 10);
		int_to_str(LORAcomm[WPSAddress].wpsdata.serialble, buffer, 0, 10);
		wpsfile << buffer << " ";

		//newver int_to_str(WPSLostCheckStruct[i].SleepCount, buffer, 0, 10);
		int_to_str((LORAcomm[WPSAddress].wpsdata.vehicledc_sleepc & 0b00111111), buffer, 0, 10);
		wpsfile << buffer << " ";

		//newver int_to_str(WPSLostCheckStruct[i].IsLost, buffer, 0, 10);
		int_to_str(LORAcomm[WPSAddress].loststruct.IsLost, buffer, 0, 10);
		wpsfile << buffer << " ";

		//newver int_to_str(WPSLostCheckStruct[i].WasLost, buffer, 0, 10);
		int_to_str(LORAcomm[WPSAddress].loststruct.WasLost, buffer, 0, 10);
		wpsfile << buffer << " " << endl;

		wpsfile.close();
		//newver WPSLostCheckStruct[i].OnRegisteredWPSFile = true;
		LORAcomm[WPSAddress].loststruct.OnRegisteredWPSFile = true;

		//WPSLostCheckStruct[i].StatusChange = false;
		coordinator.registered_wps++;
		return true;
		//newver				}
		//newver			}
		//newver	}
	}

	// WRITE FILE (ONLY AFTER LOST, FOUND, WPS TRANSFERED/REMOVED OR STATUS CHANGED
	if (ReadWrite == 'W' || ReadWrite == 'w')
	{
		printf(" Overwrite registeredwps.conf file...With number of %d Registered WPSs\n", coordinator.registered_wps);

		fstream wpsfile;
		char buffer[10];
		wpsfile.open("registeredwps.conf", ios::out | ios::trunc);

		for (int i = 0; i < coordinator.registered_wps; i++)
		{
			wpsfile << "# ";
			//newver int_to_str(WPSLostCheckStruct[i].WPSAddress, buffer, 0, 10);
			int_to_str(LORAcomm[i].WPSAddress, buffer, 0, 10);
			wpsfile << buffer << " ";

			//newver int_to_str((uint32_t)WPSLostCheckStruct[i].serialmsb, buffer, 0, 10);
			int_to_str((uint16_t)LORAcomm[i].wpsdata.serialmsb, buffer, 0, 10);
			wpsfile << buffer << " ";

			//newver int_to_str((uint32_t)WPSLostCheckStruct[i].seriallsb, buffer, 0, 10);
			int_to_str((uint32_t)LORAcomm[i].wpsdata.serialble, buffer, 0, 10);
			wpsfile << buffer << " ";

			//newver int_to_str(WPSLostCheckStruct[i].SleepCount, buffer, 0, 10);
			int_to_str((LORAcomm[i].wpsdata.vehicledc_sleepc & 0b00111111), buffer, 0, 10);
			wpsfile << buffer << " ";

			//newver int_to_str(WPSLostCheckStruct[i].IsLost, buffer, 0, 10);
			int_to_str(LORAcomm[i].loststruct.IsLost, buffer, 0, 10);
			wpsfile << buffer << " ";

			//newver int_to_str(WPSLostCheckStruct[i].WasLost, buffer, 0, 10);
			int_to_str(LORAcomm[i].loststruct.WasLost, buffer, 0, 10);
			wpsfile << buffer << " " << endl;

			// WPS Adresi 0'dan farklı girildiyse WPS çıkartılmış veya transfer edilmiş demektir. 
		}
		wpsfile.close();
		return true;

	}
}

bool ReadCoordinatorConfig()
{
	cout << " \n\n Reading Configuration File...\n\n";
	const int MAX_CHARS_PER_LINE = 100;
	const int MAX_TOKENS_PER_LINE = 5;
	const char* const DELIMITER = " ";


	// create a file-reading object
	ifstream fin;
	fin.open("coordinator.conf"); // open a file
	if (!fin.good()) return false; // exit if file not found

								   // read each line of the file
	while (!fin.eof())
	{
		// read an entire line into memory
		char buf[MAX_CHARS_PER_LINE];
		fin.getline(buf, MAX_CHARS_PER_LINE);

		// parse the line into blank-delimited tokens
		int n = 0; // a for-loop index

				   // array to store memory addresses of the tokens in buf
		const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0

													 // parse the line
		token[0] = strtok(buf, DELIMITER); // first token
		if (token[0]) // zero if line is blank
		{
			for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
			{
				token[n] = strtok(0, DELIMITER); // subsequent tokens
				if (!token[n]) break; // no more tokens
			}
		}

		// process (print) the tokens
		for (int i = 0; i < n; i++) // n = #of tokens
		{
			//cout << "Token[" << i << "] = " << token[i] << " Size : "<< sizeof(token[i]) <<endl;
			if (string(token[i]) == "COORDINATOR_HW_VERSION")
			{
				coordinator.HWversion = (uint8_t)(atoi(token[i + 2]));
				printf("Coordinator HW Version    = %d\n", coordinator.HWversion);
			}
			if (string(token[i]) == "COORDINATOR_SW_VERSION_MAJOR") coordinator.SWversion_Major = (uint8_t)(atoi(token[i + 2]));
			if (string(token[i]) == "COORDINATOR_SW_VERSION_MINOR")
			{
				coordinator.SWversion_Minor = (uint8_t)(atoi(token[i + 2]));
				printf("Coordinator SW Version    = %d.%d\n", coordinator.SWversion_Major, coordinator.SWversion_Minor);
			}

			if (string(token[i]) == "COORDINATOR_SERIAL")
			{
				//coordinator.serial = (uint32_t)(atoi(token[i + 2]));
				string tmpserial = token[i + 2];
				coordinator.serial = hex2dec(tmpserial.c_str(), 6);
				printf("Coordinator Serial        = %.6X \n", coordinator.serial);
			}
			if (string(token[i]) == "COORDINATOR_RADIO_TX_POWER")
			{
				coordinator.power = (uint8_t)(atoi(token[i + 2]));
				printf("Radio TX Power            = %d\n", coordinator.power);
			}

			if (string(token[i]) == "COORDINATOR_CHANNEL")
			{
				//printf(token[i + 2]);
				uint8_t chnl = (uint8_t)(atoi(token[i + 2]));
				if (chnl < 64)
				{
					coordinator.frq = (BASE_FREQ433 + ((float)chnl)*0.2);
				}
				else
				{
					coordinator.frq = (BASE_FREQ865 + ((float)(64 - chnl))*0.3);
				}

				//newver coordinator.frq = strtof(token[i + 2], 0);
				printf("Radio Frequency           = %3.2f\n", coordinator.frq);
			}

			if (string(token[i]) == "COORDINATOR_ADDRESS")
			{
				coordinator.address = (uint8_t)(atoi(token[i + 2]));
				printf("Coordinator Serial        = %d\n", coordinator.address);
			}

			if (string(token[i]) == "NETWORK_ID")
			{
				coordinator.NW_id = (uint8_t)(atoi(token[i + 2]));
				printf("Network ID                = %d\n", coordinator.NW_id);
			}

			if (string(token[i]) == "WPS_LOST_CYCLE")
			{
				coordinator.wps_lost_cycle = (uint8_t)(atoi(token[i + 2]));
				printf("WPS Lost Count            = %d\n", coordinator.wps_lost_cycle);
			}

			if (string(token[i]) == "MAX_WPS_NUMBER")
			{
				coordinator.max_wps = (uint8_t)(atoi(token[i + 2]));
				printf("MAX WPS allowed           = %d\n", coordinator.max_wps);
			}

			if (string(token[i]) == "REGISTERED_WPS_COUNT")
			{
				coordinator.registered_wps = (uint8_t)(atoi(token[i + 2]));
				printf("Registered WPS count      = %d\n", coordinator.registered_wps);
			}

			if (string(token[i]) == "MQTT_SERVER")
			{
				strcpy(coordinator.ServerAddress, token[i + 2]);
				printf("MQTT Server Address       = %s\n", coordinator.ServerAddress);
			}

			if (string(token[i]) == "MQTT_PORT")
			{
				coordinator.ServerPort = (uint16_t)(atoi(token[i + 2]));
				printf("MQTT Server Port          = %d\n", coordinator.ServerPort);
			}

			if (string(token[i]) == "SECRET")
			{
				coordinator.secret = (uint16_t)(atoi(token[i + 2]));
				printf("SECRET in secret          = %d\n", coordinator.secret);
			}

			if (string(token[i]) == "MQTT_PASSWORD")
			{
				strcpy(coordinator.mqtt_password, token[i + 2]);
				printf("MQTT Password             = %s\n", coordinator.mqtt_password);
			}

			if (string(token[i]) == "MQTT_USERNAME")
			{
				strcpy(coordinator.mqtt_username, token[i + 2]);
				printf("MQTT Username             = %s\n", coordinator.mqtt_username);
			}

			if (string(token[i]) == "MQTT_TOPIC")
			{
				strcpy(coordinator.mqtt_topic, token[i + 2]);
				printf("MQTT Topic                = %s\n", coordinator.mqtt_topic);
			}

		}
	}
	return true;
}

bool PrepareCoordinator()
{
	fstream file;
	file.open("coordinator.conf", ios::out | ios::in);//Attempt to open file.
	if (file.is_open())
	{
		file.close();
		if (ReadCoordinatorConfig()) { return true; }
		else { return false; }
	}
	else
	{
		//std:ostringstream ostr;
		cout << "\n Writing configuration file for first time...\n";
		file.open("coordinator.conf", ios::out | ios::in | ios::trunc);//TRUNCATING to make new file!

		file << "COORDINATOR_HW_VERSION             = " << COORDINATOR_HW_VERSION << endl;
		file << "COORDINATOR_SW_VERSION_MAJOR       = " << COORDINATOR_SW_VERSION_MAJOR << endl;
		file << "COORDINATOR_SW_VERSION_MINOR       = " << COORDINATOR_SW_VERSION_MINOR << endl;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//					 BURAYI RASPİ SERİ NUMARASIN SON 3 BYTE'INI ALACAK ŞEKİLDE DEĞİŞTİR 
		uint16_t msb = COORDINATOR_SERIAL_MSB;
		uint8_t lsb = COORDINATOR_SERIAL_LSB;
		uint16_t sixteenbit = msb * 256 + lsb;
		char stringnumber[10];
		int_to_str(sixteenbit, stringnumber, 0, 10);
		file << "COORDINATOR_SERIAL                 = " << stringnumber << endl;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		file << "COORDINATOR_RADIO_TX_POWER         = " << COORDINATOR_RADIO_TX_POWER << endl;

		int_to_str(BASE_CHANNEL, stringnumber, 0, 10);
		file << "COORDINATOR_CHANNEL                = " << stringnumber << endl;

		int_to_str(coordinator.address, stringnumber, 0, 10);
		file << "COORDINATOR_ADDRESS                = " << stringnumber << endl;


		file << "NETWORK_ID                         = " << def_NETWORK_ID << endl;

		file << "WPS_LOST_CYCLE                     = " << def_WPS_LOST_COUNT << endl;

		file << "MAX_WPS_NUMBER                     = " << MAX_WPS_NUMBER << endl;

		int_to_str(coordinator.registered_wps, stringnumber, 0, 10);
		file << "REGISTERED_WPS_COUNT               = " << stringnumber << endl;
		file << "MQTT_SERVER                        = " << TESTSERVER << endl;

		int_to_str(def_MQTT_PORT, stringnumber, 0, 10);
		file << "MQTT_PORT                          = " << stringnumber << endl;

		file << "SECRET                             = " << def_SECRET_CODE << endl;

		file << "MQTT_PASSWORD                      = " << def_MQTT_PASSWORD << endl;
		file << "MQTT_USERNAME                      = " << def_MQTT_USERNAME << endl;
		file << "MQTT_TOPIC                         = " << def_MQTT_TOPIC << endl;
		file.close();//File must be properly closed if opened! Else issues may arise.

		ReadCoordinatorConfig();
	}
	return true;
}

bool LORARead()
{
	//*******************************************************************************
	//*******************   RADIO READING		 ************************************
	//*******************************************************************************
	uint8_t checksumreceived, checksum; //Version 3.01


	// Should be a message for us now
	//uint8_t data_from_wps_temp[WPS_PACKET_LENGHT];
	//newver uint8_t len = sizeof(data_from_wps_temp);
	uint8_t len = sizeof(wpsdata);

	uint8_t from = driver.headerFrom();
	uint8_t to = driver.headerTo();
	uint8_t id = driver.headerId();
	uint8_t flags = driver.headerFlags();
	int16_t Rssi = driver.lastRssi();
	uint8_t LORAWPSindex = 0;


	////////////////////////////////////////////////////////////////////////////////////
	//						GET DATA FROM WPS STEP #1
	////////////////////////////////////////////////////////////////////////////////////

	//if (driver.recv(data_from_wps_temp, &len))
	if (driver.recv((uint8_t*)&wpsdata, &len))
	{
		printf("\n\n\n Packet Received..\n");
		printf(" from = %d\n", from);

		printf(" Network ID     = %d\n", wpsdata.nw_id);

		switch (wpsdata.flagtype)
		{
		case FLAG_PERIODICAL: { printf(" Packet Type    = Periodical\n"); break; }
		case FLAG_VEHICLE_STATUS_CHANGE: { printf(" Packet Type    = Vehicle Status Change\n"); break; }
		case FLAG_INTER: { printf(" Packet Type    = Interrim Packet\n"); break; }
		case FLAG_CONFIGURATION_SET: { printf(" Packet Type    = Configuration Has Been Set\n"); break; }
		case FLAG_FIN: { printf(" Packet Type    = FIN\n"); break; }
		case FLAG_PING: { printf(" Packet Type    = Ping Packet\n"); break; }
		}


		if (wpsdata.nw_id != coordinator.NW_id)
		{
			printf(" Wrong NW ID...\n");
			return false;
		}


		////////////////////////////////////////////////////////////////////////////////////
		//						IF PING PACKET KEEP IT SHORT :-)
		////////////////////////////////////////////////////////////////////////////////////

		//newver if (data_from_wps_temp[1] == FLAG_PING)
		if (wpsdata.flagtype == FLAG_PING)
		{
			uint8_t ping_packet[2];
			ping_packet[0] = coordinator.NW_id;
			ping_packet[1] = FLAG_PING;
			driver.setModeRx();
			//Serial.println("CAD Control\n");
			driver.waitCAD();
			driver.setHeaderTo(from);
			//Serial.println("Set to TX\n");
			driver.setModeTx();
			//Serial.println("Sending\n");
			driver.send(ping_packet, sizeof(ping_packet));
			//Serial.println("Wait Sending\n");
			driver.waitPacketSent();
			//Serial.println("Set to RX\n");
			LORAcomm[from].wpsdata.rssi = (uint8_t)(-1 * driver.lastRssi());
			return true;
		}


		////////////////////////////////////////////////////////////////////////////////////
		//						    GET DATA FROM WPS STEP #2
		//						
		//						   FIN veya CFG_SET DEĞİL İSE
		//				BU DURUMDA PERIODICAL-VEHICLE_ST-INTERRIM PAKEDİDIR
		//					ACK_OK, RETRANSMIT VEYA CHANGE_REQUEST YOLLA
		////////////////////////////////////////////////////////////////////////////////////

		// FIN PAKEDİ DEĞİL İSE LORAcomm[LORAWPSindex] içerisine data_from_wps_temp değerini  ata.
		//newver if ((data_from_wps_temp[1] != FLAG_FIN) && (data_from_wps_temp[1] != FLAG_CONFIGURATION_SET))

		if ((wpsdata.flagtype != FLAG_FIN) && (wpsdata.flagtype != FLAG_CONFIGURATION_SET))
		{
			//newver checksumreceived = data_from_wps_temp[len - 1];


			if (wpsdata.flagtype == FLAG_INTER)
			{
				checksum = 0;
				checksumreceived = 0;
			}
			else
			{
				checksumreceived = wpsdata.checksum;
				checksum = wpsdata.nw_id;
			}

			//checksum = wpsdata.selfChecksum(); //CHECKSUM KONTROL ETMEYECEK /////////////////////////////////////////////////////////////////////

			// IF CHECKSUM ERROR SEND RETRANSMIT PACKET
			if (checksum != checksumreceived)
			{
				printf(" CHECKSUM ERROR... FIRST CHECK...\n");
				printf(" CHECKSUM R/S:	%d	%d\n", checksum, checksumreceived);
				uint8_t retransmit_packet[2];
				retransmit_packet[0] = coordinator.NW_id;
				retransmit_packet[1] = FLAG_RETRANSMIT;
				driver.setModeRx();
				//Serial.println("CAD Control\n");
				driver.waitCAD();
				driver.setHeaderTo(from);
				//Serial.println("Set to TX\n");
				driver.setModeTx();
				//Serial.println("Sending\n");
				driver.send(retransmit_packet, sizeof(retransmit_packet));
				//Serial.println("Wait Sending\n");
				driver.waitPacketSent();
				//Serial.println("Set to RX\n");
				return true;
			}


			LORAcomm[from].wpsdata = wpsdata;
			//LORAcomm[from].WPSAddress = from;
			LORAcomm[from].ACTSend = false;

			uint32_t sequence; sequence = (uint32_t)LORAcomm[from].wpsdata.sequence3 * 256 * 256 + (uint16_t)LORAcomm[from].wpsdata.sequence2 * 256 + LORAcomm[from].wpsdata.sequence1;

			///////////////////////////////////////////////////////////////
			// CHECKSUM OK İSE ACK_OK PAKETDİ YOLLA....
			///////////////////////////////////////////////////////////////

			printf(" Checksum OK. Sending ACK Packet\n");
			printf("-------------------------------------------\n");
			//PrepareDataToWPS(LORAWPSindex, FLAG_ACK_OK,Rssi);
			//PrepareDataToWPS(LORAWPSindex, FLAG_CONFIGURATION_SET,Rssi);
			if (((LORAWPSindex == 59) || (LORAWPSindex == 57) || (LORAWPSindex == 45)) && (sequence > 50000))

			{
				printf(" Checksum OK. Sending CHANGE_REQUEST Packet to : %d \n", from);
				printf("-------------------------------------------\n");
				PrepareDataToWPS(from, FLAG_CONFIGURATION_SET, Rssi);
			}
			else
			{
				PrepareDataToWPS(from, FLAG_ACK_OK, Rssi);
			}

			// ACK_OK VEYA CHG_RQ PAKEDİ YOLLA SONUCUNU ACTSend içerisine yaz...
			LORAcomm[from].ACTSend = SendToWPS(from);
			LORAcomm[from].FINReceived = false;

		}
		////////////////////////////////////////////////////////////////////////////////////
		//						GET DATA FROM WPS STEP #3
		//						
		//						   FIN veya CFG_SET İSE
		//						  HABERLEŞME SONA ERİYOR
		//						  MQTT VERİSİ YOLLANIYOR
		////////////////////////////////////////////////////////////////////////////////////
		else if (wpsdata.flagtype == FLAG_CONFIGURATION_SET || wpsdata.flagtype == FLAG_FIN)
		{
			if (wpsdata.flagtype == FLAG_CONFIGURATION_SET) printf(" CFG_APPLIED Packet Received from WPS address : %d\n", from);
			if (wpsdata.flagtype == FLAG_FIN) printf(" FIN Packet Received from WPS address : %d\n", from);
			printf(" -------------------------------------------\n");

			//FIN PAKEDİNDEN ÖNCE ACK_OK SEND EDİLMEMİŞ İSE RETRANSMİT PAKEDİ YOLLA -- UNCOMPLETED FIN
			if (LORAcomm[from].ACTSend == false)
			{
				printf(" Uncompleted FIN packet. Request Restransmit..\n\n");
				uint8_t retransmit_packet[2];
				retransmit_packet[0] = coordinator.NW_id;
				retransmit_packet[1] = FLAG_RETRANSMIT;
				driver.setModeRx();
				//Serial.println("CAD Control\n");
				driver.waitCAD();
				driver.setHeaderTo(from);
				//Serial.println("Set to TX\n");
				driver.setModeTx();
				//Serial.println("Sending\n");
				driver.send(retransmit_packet, sizeof(retransmit_packet));
				//Serial.println("Wait Sending\n");
				driver.waitPacketSent();
				//Serial.println("Set to RX\n");
				return true;
			}

			LORAcomm[from].ACTSend = false;
			LORAcomm[from].FINReceived = true;
			LORAcomm[from].MQTTmsgSend = false;


			if (LORAcomm[from].wpsdata.flagtype == FLAG_PERIODICAL)
			{
				cout << "Periodical MQTT Packet Will Be Sent.." << endl << endl;
				SendMessageMQTT(from, MSG_PERIODICAL);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}

			if (LORAcomm[from].wpsdata.flagtype == FLAG_INTER)
			{
				cout << "Interrim MQTT Packet Will Be Sent.." << endl << endl;
				SendMessageMQTT(from, MSG_INTERRIM);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}

			if (LORAcomm[from].wpsdata.flagtype == FLAG_VEHICLE_STATUS_CHANGE)
			{
				cout << "Vehicle Status MQTT Packet Will Be Sent.." << endl << endl;
				SendMessageMQTT(from, MSG_VEHICLE_STATUS);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}

			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			//								YENİ LOST RESET İŞLEMİ
			//								
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (LORAcomm[from].WPSAddress == from)
			{
				if (LORAcomm[from].loststruct.IsLost)
				{
					printf(" This WPS was lost ... %d   \n", from);
					NUMBER_OF_LOST_WPS--;
					LORAcomm[from].loststruct.IsLost = false;
					LORAcomm[from].loststruct.WasLost = false;
					RWRegisteredWPSCfg('W', 0);
				}

				LORAcomm[from].loststruct.LostTimer = millis();

				if (!LORAcomm[from].loststruct.SignalSending)
				{
					LORAcomm[from].loststruct.SignalSending = true;
					WPSSignalSending++;
				}
			}
			else
			{
				LORAcomm[from].WPSAddress = from;
				LORAcomm[from].loststruct.IsLost = 0;
				LORAcomm[from].loststruct.WasLost = 0;
				LORAcomm[from].loststruct.LostTimer = millis();

				LORAcomm[from].loststruct.WPSSerial = (uint64_t)LORAcomm[from].wpsdata.serialmsb << 32 | LORAcomm[from].wpsdata.serialble;
				LORAcomm[from].loststruct.SignalSending = true;
				WPSSignalSending++;
				printf("write to file...\n");
				RWRegisteredWPSCfg('A', from);
			}

			printf(" Live WPS count = %d/%d\n", WPSSignalSending, coordinator.registered_wps);
			for (int i = 0; i <= 255; i++)
			{
				if ((LORAcomm[i].loststruct.SignalSending == true) && (LORAcomm[i].loststruct.IsLost == false)) printf(" %d		", i);
			}
			cout << endl;

			// GELEN VERİLERİ EKRANA YAZ

			WPS_DATA_print(from, len, from, to, id, flags, Rssi, checksum, checksumreceived);
			printf(" Index No : %d\n", LORAWPSindex);

			return true;
		}
	}
	else
	{
		printf("receive failed\n");
		printf("-------------------------------------------\n");
		return false;
	}

	//EndOfStory:
	printf("\n");
	return true;
}

void PrepareDataToWPS(uint8_t LORAWPSindex, uint8_t flagtype, int16_t Rssi)
{

	uint8_t WPSADDRESS = LORAWPSindex;

	//.changerq
	uint8_t vector0reset = 0;
	uint8_t wpsreset = 0;
	uint8_t changerq = 0;
	uint8_t initialstate = 0;
	uint8_t changewps = 0; //if = 1 change NW_ID


	uint8_t radiotxpower = 13;
	uint8_t blepower = 2;
	uint8_t vdcount = 1;
	uint8_t bletimeout = 2;

	uint8_t variancebase = 3;
	uint8_t variancelimit = 5;

	uint8_t checkrssi = 0;
	uint8_t minrssidiff = 2;
	uint8_t maxrssidiff = 5;

	uint8_t sleepcount = 45;

	uint8_t maggain = 0;
	uint8_t magfrq = 3;
	uint8_t magomxy = 0;
	uint8_t magomz = 0;

	uint8_t newcoordaddr = 0;
	uint8_t lorachannel = 0;
	uint8_t newwpsaddr = WPSADDRESS;
	uint8_t newnwid = coordinator.NW_id;

	uint8_t varianceordiff = 0;
	uint8_t diffmultip = 0;

	LORAcomm[LORAWPSindex].coordinatordata.nw_id = coordinator.NW_id;
	LORAcomm[LORAWPSindex].coordinatordata.flagtype = flagtype; // PAKET TİPİ

	//						0 = No change
	//						BIT 0 = 1 = Reset VECTOR0  (Bit 0)
	//						BIT 1 = 2 = Reset WPS SOFTWARE (Bit 1)
	//						BIT 2 = 4 = CHANGE REQUEST
	//						BIT 3 = 8 = InitialState=STATE_BUSY
	//						BIT 4 = 16=	Change WPS address (0), Change NW_ID (1)
	LORAcomm[LORAWPSindex].coordinatordata.changerq = 0;

	// WPS ÜZERİNDEN ALINAN SİNYAL Rssi
	LORAcomm[LORAWPSindex].coordinatordata.rssi = (uint8_t)(-1 * Rssi);

	//hangi parametreler değişecek?
	LORAcomm[LORAWPSindex].coordinatordata.changeflag = 0;
	//						BIT 0	1-	MAG Z,XY,FRQ,VarOrDiff
	//						BIT 1	2-	Vector Variance Limits
	//						BIT 2	4-	RSSI Diff limits
	//						BIT 3	8-	BLE PWR, RADIO PWR, Chck RSSI
	//						BIT 4	16-	VDC, SLEEPC
	//						BIT 5	32-	LORA CHANNEL
	//						BIT 6	64-	DIFF MULTIPLIER, BLE TO, GAIN
	//						BIT 7	128-WPS Address or NW_ID

	//BIT 0	// MAG Z(0-1),XY(2-3),FRQ(4-6),VarOrDiff(7)
	LORAcomm[LORAWPSindex].coordinatordata.mag_varordiff = varianceordiff << 7 | magfrq << 4 | magomxy << 2 | magomz;

	//BIT 1	//VarianceBase(0-2),VarianceLimit(3-7)
	LORAcomm[LORAWPSindex].coordinatordata.varlimits = variancelimit << 3 | variancebase;

	//BIT 2	//minrssi(0-2),maxrssi(3-7)
	LORAcomm[LORAWPSindex].coordinatordata.rssilimits = maxrssidiff << 3 | minrssidiff;

	// BIT 3	//WPS RADIO_TX POWER (BIT 0-4, MIN=5, MAX=23, DEFAULT=13), BLE POWER (Bit 5-6), CHECKRSSI (Bit 7)
	LORAcomm[LORAWPSindex].coordinatordata.blepw_radiopw_chkrssi = checkrssi << 7 | blepower << 5 | radiotxpower;

	//BIT 4	// SLEEPC (0-5),VDC(6-7)
	LORAcomm[LORAWPSindex].coordinatordata.vehicledc_sleepc = vdcount << 6 | sleepcount;

	//BIT 5 // LORA Channel
	LORAcomm[LORAWPSindex].coordinatordata.lorachannel = lorachannel;

	//BIT 6 // BLETO (4-7),GAIN(2-3),DIFFMULTP(0-1)
	LORAcomm[LORAWPSindex].coordinatordata.diffmultp_bleto_gain = bletimeout << 4 | maggain << 2 | diffmultip;

	//BIT 7 // NEW WPSaddress or NW_ID
	LORAcomm[LORAWPSindex].coordinatordata.wpsaddress = newwpsaddr;
	if (changewps) LORAcomm[LORAWPSindex].coordinatordata.wpsaddress = newnwid;
}

bool SendToWPS(uint8_t LORAWPSindex)
{
	bool sendstatus = false;
	sCoordData tempcoorddata;
	tempcoorddata = LORAcomm[LORAWPSindex].coordinatordata;

	driver.setModeRx();

	driver.waitCAD();

	driver.setHeaderTo(LORAWPSindex);

	driver.setModeTx();
	//Serial.println("Sending\n");
	//newver driver.send(LORAcomm[LORAWPSindex].data_to_wps, sizeof(LORAcomm[LORAWPSindex].data_to_wps));
	driver.send((uint8_t*)&tempcoorddata, sizeof(tempcoorddata));
	//Serial.println("Wait Sending\n");
	sendstatus = driver.waitPacketSent();

	driver.setModeRx();
	switch (tempcoorddata.flagtype)
	{
	case FLAG_PERIODICAL: { printf(" PERIODICAL PACKET SENT TO : %d\n", LORAWPSindex); break; }
	case FLAG_VEHICLE_STATUS_CHANGE: { printf(" VEHICLE STATUS CHANGE PACKET SENT TO: %d\n", LORAWPSindex); break; }
	case FLAG_NEW_CONFIGURATION: { printf(" CONFIGURATION CHANGE REQUEST PACKET SENT TO: %d\n", LORAWPSindex); break; }
	case FLAG_ACK_OK: { printf(" ACK_OK PACKET SENT TO : %d\n", LORAWPSindex); break; }
	case FLAG_RETRANSMIT: { printf(" RETRANSMIT REQUEST PACKET SENT TO : %d\n", LORAWPSindex); break; }
	case FLAG_FIN: { printf(" FLAG_FIN PACKET SENT TO : %d\n", LORAWPSindex); break; }
	}
	return sendstatus;
}

void SPI_setup(void)
{
	signal(SIGINT, sig_handler);
	printf("%s\n", __BASEFILE__);

	if (!bcm2835_init()) {
		fprintf(stderr, "%s bcm2835_init() Failed\n\n", __BASEFILE__);
		// return 1;
	}

	printf("RF95 CS=GPIO%d", RF_CS_PIN);



	printf(", IRQ=GPIO%d", RF_IRQ_PIN);
	// IRQ Pin input/pull down
	//pinMode(RF_IRQ_PIN, INPUT);
	bcm2835_gpio_set_pud(RF_IRQ_PIN, BCM2835_GPIO_PUD_DOWN);
	// Now we can enable Rising edge detection
	bcm2835_gpio_ren(RF_IRQ_PIN);


	printf(", RST=GPIO%d", RF_RST_PIN);
	// Pulse a reset on module
	//pinMode(RF_RST_PIN, OUTPUT);
	digitalWrite(RF_RST_PIN, LOW);
	bcm2835_delay(150);
	digitalWrite(RF_RST_PIN, HIGH);
	bcm2835_delay(100);


}

void RF95_setup(uint8_t RadioPower, uint8_t Coordinator_Address, uint8_t Network_ID, float RFFrequency)
{
	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
	// you can set transmitter powers from 5 to 23 dBm:
	//  driver.setTxPower(23, false);
	// If you are using Modtronix inAir4 or inAir9,or any other module which uses the
	// transmitter RFO pins and not the PA_BOOST pins
	// then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true. 
	// Failure to do that will result in extremely low transmit powers.
	// rf95.setTxPower(14, true);


	// RF95 Modules don't have RFO pin connected, so just use PA_BOOST
	// check your country max power useable, in EU it's +14dB

	//bool useRFO = false;

	//driver.setTxPower(RadioPower, useRFO);
	driver.setTxPower(RadioPower);
	printf("\n TX Power : %d dBm \n", RadioPower);
	//driver.setModemConfig(Bw125Cr48Sf512);

	// You can optionally require this module to wait until Channel Activity
	// Detection shows no activity on the channel before transmitting by setting
	// the CAD timeout to non-zero:
	//rf95.setCADTimeout(10000);

	// Adjust Frequency
	driver.setFrequency(RFFrequency);

	// If we need to send something
	driver.setThisAddress(Coordinator_Address);
	driver.setHeaderFrom(Coordinator_Address);
	//driver.setHeaderId(Network_ID);

	// Be sure to grab all node packet 
	// we're sniffing to display, it's a demo
	// driver.setPromiscuous(true);

	// We're ready to listen for incoming message
	driver.setModeRx();

	printf(" OK NodeID=%d @ %3.2fMHz\n", coordinator.address, coordinator.frq);
	printf(" Listening packet...\n");
	printf("------------------------------------\n");

	//Begin the main body of code
}

void WPS_DATA_print(uint8_t printindex, uint8_t len, uint8_t from, uint8_t to, uint8_t id, uint8_t flags, int16_t Rssi, uint8_t checksum, uint8_t checksumreceived)
{
	printf(" Header ID, Mess. Len: %d - %d\n", id, len);
	printf(" Flags, Rssi         : %d - (%d)\n", flags, Rssi);
	printf(" Coordinator Time    : %s\n", datetime_f(TIMESTMP_COMPUTER));

	printf(" WPS LoRA Channel    : %d \n", LORAcomm[from].wpsdata.lorachannel);

	printf(" WPS Address         : %d\n", from);
	uint8_t serial[6];
	serial[0] = LORAcomm[from].wpsdata.serialmsb >> 8;
	serial[1] = LORAcomm[from].wpsdata.serialmsb;
	serial[2] = LORAcomm[from].wpsdata.serialble >> 24;
	serial[3] = LORAcomm[from].wpsdata.serialble >> 16;
	serial[4] = LORAcomm[from].wpsdata.serialble >> 8;
	serial[5] = LORAcomm[from].wpsdata.serialble;

	printf(" WPS Serial          : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]);
	printf(" WPS Serial          : %.2X:%.6X\n", LORAcomm[from].wpsdata.serialmsb, LORAcomm[from].wpsdata.serialble);
	uint8_t wpsswmaj = (LORAcomm[from].wpsdata.versions >> 8) & 0b00001111;
	uint8_t wpsswmin = (uint8_t)LORAcomm[from].wpsdata.versions & 0b00111111;
	if (wpsswmin > 9) printf(" WPS Software Version: %d.%d\n", wpsswmaj, wpsswmin);
	if (wpsswmin < 10) printf(" WPS Software Version: %d.0%d\n", wpsswmaj, wpsswmin);

	printf(" WPS Sequence Number : %d\n", (uint32_t)(LORAcomm[from].wpsdata.sequence3 * 256 * 256 + LORAcomm[from].wpsdata.sequence2 * 256 + LORAcomm[from].wpsdata.sequence1));

	printf(" WPS Vector Variance : %d % \n", LORAcomm[from].wpsdata.retry_variance >> 4);

	printf(" WPS VarOrDiff       : ");
	if (((LORAcomm[printindex].wpsdata.mag_varordiff & 0b10000000) >> 7) == DIFFCHK)
	{
		printf("Diff. Check\n");
		uint8_t multiplier = ((LORAcomm[from].wpsdata.diffmultp_bleto_gain & 0b00000011) + 1) * 10;
		uint8_t varlim = ((LORAcomm[from].wpsdata.varlimits & 0b11111000) >> 3);
		uint8_t varbase = (LORAcomm[from].wpsdata.varlimits & 0b00000111);
		printf(" WPS Max. Variance L.: %d {%d(10 x %d+1)} \n", varlim*multiplier, varlim, multiplier / 10 - 1);
		printf(" WPS Min. Variance L.: %d {%d(10 x %d+1)} \n", varbase*multiplier, varbase, multiplier / 10 - 1);
	}
	else
	{
		printf("Variance Check\n");
		printf(" WPS Max. Variance L.: (%d %) \n", (LORAcomm[from].wpsdata.varlimits & 0b11111000) >> 3);
		printf(" WPS Min. Variance L.: (%d %) \n", (LORAcomm[from].wpsdata.varlimits & 0b00000111));
	}

	printf(" WPS Vector          : %6.2f \n", LORAcomm[from].wpsdata.vector);

	float temperature = 0;

	int16_t tmp = (int16_t)LORAcomm[from].wpsdata.temp;
	uint8_t hwtype = LORAcomm[from].wpsdata.versions >> 12;

	if (hwtype <= 2)
	{
		temperature = (float)tmp / 128;
	}
	else
	{
		temperature = (float)tmp / 8;
	}
	temperature += 25;
	printf(" WPS Sensor Temp.    : %3.2f C\n", temperature);
	uint8_t MAG_FRQ = (LORAcomm[from].wpsdata.mag_varordiff & 0b01110000) >> 4;
	uint8_t MAG_OMXY = (LORAcomm[from].wpsdata.mag_varordiff & 0b00001100) >> 2;
	uint8_t MAG_OMZ = (LORAcomm[from].wpsdata.mag_varordiff & 0b00000011);
	uint8_t MAG_GAIN = (LORAcomm[from].wpsdata.diffmultp_bleto_gain & 0b11110011) >> 2;

	printf(" WPS Sensor Gain     : %d (", MAG_GAIN);
	switch (MAG_GAIN)
	{
	case 0: {printf("4 Gauss)\n"); break; }
	case 1: {printf("8 Gauss)\n"); break; }
	case 2: {printf("12 Gauss)\n"); break; }
	case 3: {printf("16 Gauss)\n"); break; }
	default: {printf("4 Gauss)\n"); break; }
	}

	printf(" WPS Sensor Read Frq : %d (", MAG_FRQ);
	switch (MAG_FRQ)
	{
	case 0: {printf("0.65 Hz)\n"); break; }
	case 1: {printf("1.25 Hz)\n"); break; }
	case 2: {printf("2.5 Hz)\n"); break; }
	case 3: {printf("5 Hz)\n"); break; }
	case 4: {printf("10 Hz)\n"); break; }
	case 5: {printf("20 Hz)\n"); break; }
	case 6: {printf("40 Hz)\n"); break; }
	case 7: {printf("80 Hz)\n"); break; }
	default: {printf("5 Hz)\n"); break; }
	}

	printf(" WPS Sensor XY Perf  : %d (", MAG_OMXY);
	switch (MAG_OMXY)
	{
	case 0: {printf("Low performance)\n"); break; }
	case 1: {printf("Medium Performance)\n"); break; }
	case 2: {printf("High Performance)\n"); break; }
	case 3: {printf("Ultra High Performance)\n"); break; }
	default: {printf("Low performance)\n"); break; }
	}

	printf(" WPS Sensor Z Perf   : %d (", MAG_OMZ);
	switch (MAG_OMZ)
	{
	case 0: {printf("Low performance)\n"); break; }
	case 1: {printf("Medium Performance)\n"); break; }
	case 2: {printf("High Performance)\n"); break; }
	case 3: {printf("Ultra High Performance)\n"); break; }
	default: {printf("Low performance)\n"); break; }
	}

	printf(" Battery Level       : %d % \n", WPSVoltageLevel(LORAcomm[from].wpsdata.voltage));
	printf(" Battery Voltage     : %3.2f V \n", (float)LORAcomm[from].wpsdata.voltage*3.3 * 2 / 1024);
	printf(" Vehicle Detected    : "); if ((LORAcomm[from].wpsdata.error_vehicle & 0b10000000) >> 7) { printf("True\n"); }
	else { printf("False\n"); }

	printf(" RADIO_RSSI WPS SIDE : -%d dB\n", LORAcomm[from].wpsdata.rssi);
	printf(" RADIO_RSSI_COR SIDE : %d dB\n", Rssi);
	printf(" MIN/MAX_RSSI_DIFF   : %d - %d dB \n", (LORAcomm[from].wpsdata.rssilimits & 0b00000111), (LORAcomm[from].wpsdata.rssilimits >> 3));
	printf(" CHECK_RSSI          : ");
	if ((LORAcomm[from].wpsdata.blepw_radiopw_chkrssi & 0b10000000) >> 7)
	{
		printf("True\n");
	}
	else
	{
		printf("False\n");
	}
	printf(" WPS HW Version      : %d (", hwtype);
	switch (hwtype)
	{
	case 1: {printf("Arduino+HMC5983)\n"); break; }
	case 2: {printf("Arduino+HMC5983+HM10)\n"); break; }
	case 3: {printf("Arduino+LIS3MDL)\n"); break; }
	case 4: {printf("Arduino+LSM303C)\n"); break; }
	case 5: {printf("328P+LSM303C)\n"); break; }
	case 6: {printf("328P+LSM303C+HM10)\n"); break; }
	default: {printf("HW NOT IDENTIFIED)\n"); break; }
	}
	printf(" WPS Error Code      : %d \n", LORAcomm[from].wpsdata.error_vehicle & 0b01111111);


	uint8_t bletimeout = (LORAcomm[from].wpsdata.diffmultp_bleto_gain & 0b11110000) >> 4;
	printf(" BLE TIMEOUT         : %1.2f mins\n", (float)bletimeout / 2);

	uint8_t bletxpower = (LORAcomm[from].wpsdata.blepw_radiopw_chkrssi & 0b01100000) >> 5;
	printf(" BLE TX POWER        : %d (", bletxpower);
	switch (bletxpower)
	{
	case 0: printf("-23 dBm)\n"); break;
	case 1: printf("-6 dBm)\n"); break;
	case 2: printf("0 dBm)\n"); break;
	case 3: printf("6 dBm)\n"); break;
	}

	printf(" WPS Retry Count     : %d \n", (LORAcomm[from].wpsdata.retry_variance & 0b00001111));
	printf(" WPS Vehicle Det. Cnt: %d \n", (LORAcomm[from].wpsdata.vehicledc_sleepc & 0b11000000) >> 6);



	uint16_t val = readvoltage();
	printf(" Coordinator Battery : %3.2f V\n", (float)val*23.48 / 32768);
	printf("-------------------------------------------\n");
}

char* datetime_f(char dates[])
{
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);

	char sYear[4];
	char sMon[2];
	char sDay[2];
	char sHour[2];
	char sMin[2];
	char sSec[2];
	char* datestring = dates;

	int _YEAR = (now->tm_year + 1900);
	int _MONTH = (now->tm_mon + 1);
	int _DAY = now->tm_mday;
	int _HOUR = now->tm_hour;
	int _MINUTE = now->tm_min;
	int _SECOND = now->tm_sec;

	int_to_str(_YEAR, sYear, 1, 10);
	int_to_str(_MONTH, sMon, 1, 10);
	int_to_str(_DAY, sDay, 1, 10);
	int_to_str(_HOUR, sHour, 1, 10);
	int_to_str(_MINUTE, sMin, 1, 10);
	int_to_str(_SECOND, sSec, 1, 10);


	int i = 0;
	for (i = 0; i < 4; i++) datestring[i] = sYear[i];
	datestring[i] = '-';
	for (i = 5; i < 7; i++) datestring[i] = sMon[i - 5];
	datestring[i] = '-';
	for (i = 8; i < 10; i++) datestring[i] = sDay[i - 8];
	datestring[i] = ' ';
	for (i = 11; i < 13; i++) datestring[i] = sHour[i - 11];
	datestring[i] = ':';
	for (i = 14; i < 16; i++) datestring[i] = sMin[i - 14];
	datestring[i] = ':';
	datestring[17] = sSec[0];
	datestring[18] = sSec[1];

	return dates;
}

char* int_to_str(int i, char b[], int format_, int Base)
{
	char* p = b;
	int temp = i;
	if (Base == 16)
	{
		char const digit[] = "0123456789ABCDEF";
		if (i < 0) {
			*p++ = '-';
			i *= -1;
		}
		int shifter = i;
		do {
			++p;
			shifter = shifter / 16;
		} while (shifter);

		*p = '\0';
		do {
			*--p = digit[i % 16];
			i = i / 16;
		} while (i);

		if (format_ == 1 & temp < 16)
		{
			b[1] = b[0];
			b[0] = '0';
		}

		if (format_ == 1 & temp < 1048576)
		{
			b[6] = b[5];
			b[5] = b[4];
			b[4] = b[3];
			b[3] = b[2];
			b[2] = b[1];
			b[1] = b[0];
			b[0] = '0';
		}
		return b;
	}

	if (Base == 10)
	{
		char const digit[] = "0123456789";
		if (i < 0) {
			*p++ = '-';
			i *= -1;
		}
		int shifter = i;
		do {
			++p;
			shifter = shifter / 10;
		} while (shifter);

		*p = '\0';
		do {
			*--p = digit[i % 10];
			i = i / 10;
		} while (i);

		if (format_ == 1 & temp < 10)
		{
			b[1] = b[0];
			b[0] = '0';
		}
		return b;
	}
}

void test_output()
{
	/*
	for (int i = 0; i < 54; i++)
	{
	if (i < 10)
	{
	printf(" 0%d . %.2X \n", i, test_msqtt[i]);
	}
	else
	{
	printf(" %d . %.2X \n", i, test_msqtt[i]);
	}
	}
	*/
	printf(" Message Type			: %d\n", test_msqtt[0]);
	printf(" Coordinator Serial		: %d\n", ((uint16_t)test_msqtt[1] * 256 + test_msqtt[2]));
	printf(" Coordinator Time (CALC)	: %d.%2d.%2d - %.2d:%.2d:%.2d\n", (1900 + (uint16_t)test_msqtt[3]), test_msqtt[4], test_msqtt[5], test_msqtt[6], test_msqtt[7], test_msqtt[8]);
	printf(" Coodinator Address		: %d\n", test_msqtt[9]);
	printf(" Coodinator Battery	Val	: %d\n", ((uint16_t)test_msqtt[10] * 256 + test_msqtt[11]));
	printf(" Coordinator SW Version		: %d.%d\n", test_msqtt[12], test_msqtt[13]);
	printf(" Coordinator HW Version		: %d\n", test_msqtt[14]);
	printf(" Coordinator RSSI		: (-)%d dB\n", test_msqtt[15]);
	printf(" -------------------------------------------\n");
	printf(" WPS NW ID			: %d\n", test_msqtt[16]);
	printf(" WPS Address			: %d\n", test_msqtt[17]);
	printf(" WPS Sequence Number		: %d\n", ((uint32_t)test_msqtt[18] * 256 * 256 + (uint16_t)test_msqtt[19] + test_msqtt[20]));


	float temperature;
	int16_t tmp = (int16_t)((test_msqtt[21] * 256) + test_msqtt[22]);
	temperature = (float)tmp / 8;
	temperature += 25;
	printf(" WPS Temp Sensor (CALC)		: %3.2f C\n", temperature);

	printf(" WPS X Vector			: %d\n", (int16_t)(((uint16_t)test_msqtt[23]) * 256 + test_msqtt[24]));
	printf(" WPS Y Vector			: %d\n", (int16_t)(((uint16_t)test_msqtt[25]) * 256 + test_msqtt[26]));
	printf(" WPS Z Vector			: %d\n", (int16_t)(((uint16_t)test_msqtt[27]) * 256 + test_msqtt[28]));
	printf(" WPS Vector Variance		: %d %\n", test_msqtt[29]);
	printf(" WPS Vehicle Status		: %d\n", test_msqtt[30]);
	printf(" WPS Battery Voltage		: %3.2f V\n", ((float)((uint16_t)(test_msqtt[31]) * 256 + test_msqtt[32])*3.3 * 2 / 1024));
	printf(" WPS Battery Level		: %d %\n", test_msqtt[33]);
	printf(" WPS RSSI			: (-) %d dB\n", (uint16_t)test_msqtt[34] * 256 + test_msqtt[35]);
	printf(" WPS Serial			: %.6X\n", (uint32_t)test_msqtt[36] * 256 * 256 + (uint16_t)test_msqtt[37] * 256 + test_msqtt[38]);
	printf(" WPS HW Version			: %d\n", test_msqtt[39]);
	printf(" WPS SW Version			: %d.%d\n", test_msqtt[40], test_msqtt[41]);
	printf(" WPS Vector Variance Lim	: %d\n", test_msqtt[42]);
	printf(" WPS Vehicle Detectin Co	: %d\n", test_msqtt[43]);
	printf(" WPS Radio TX Power		: %d\n", test_msqtt[44]);
	printf(" WPS Sensor Gain		: %d\n", test_msqtt[45]);
	printf(" WPS Sleep Count		: %d\n", test_msqtt[46]);


	float wps_frq;
	uint8_t *frq_pointer = (uint8_t*)&wps_frq;
	for (uint8_t i = 0; i < sizeof(wps_frq); i++)
	{
		frq_pointer[i] = test_msqtt[47 + i];
	}
	printf(" WPS Frequency	(CALC)		: %3.2f MHz\n", wps_frq);


	printf(" WPS Error Code			: %d\n", test_msqtt[51]);
	printf(" WPS BLE Timeout		: %d mins.\n", test_msqtt[52]);
	printf(" WPS Future Use			: %d\n\n", test_msqtt[53]);


}

void FindIPAddress()
{
	FILE *f;
	char line[100], *p, *c;

	f = fopen("/proc/net/route", "r");

	while (fgets(line, 100, f))
	{
		p = strtok(line, " \t");
		c = strtok(NULL, " \t");

		if (p != NULL && c != NULL)
		{
			if (strcmp(c, "00000000") == 0)
			{
				printf("Default interface is : %s \n", p);
				break;
			}
		}
	}

	//which family do we require , AF_INET or AF_INET6
	int fm = AF_INET;
	struct ifaddrs *ifaddr, *ifa;
	int family, s;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1)
	{
		perror("getifaddrs");
		exit(EXIT_FAILURE);
	}

	//Walk through linked list, maintaining head pointer so we can free list later
	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
	{
		if (ifa->ifa_addr == NULL)
		{
			continue;
		}

		family = ifa->ifa_addr->sa_family;

		if (strcmp(ifa->ifa_name, p) == 0)
		{
			if (family == fm)
			{
				s = getnameinfo(ifa->ifa_addr, (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

				if (s != 0)
				{
					printf("getnameinfo() failed: %s\n", gai_strerror(s));
					exit(EXIT_FAILURE);
				}

				printf("address: %s\n", host);
				coordinator.IPaddress = inet_addr(host);

			}
			printf("\n");
		}
	}


	freeifaddrs(ifaddr);

}

uint32_t hex2dec(const char*s, int length)
{
	uint32_t n = 0;
	for (int i = 0, v = 0; i < length && s[i] != '\0'; i++)
	{
		//int v;
		if ('a' <= s[i] && s[i] <= 'f') { v = s[i] - 97 + 10; }
		else if ('A' <= s[i] && s[i] <= 'F') { v = s[i] - 65 + 10; }
		else if ('0' <= s[i] && s[i] <= '9') { v = s[i] - 48; }
		else break;
		n *= 16;
		n += v;
	}
	return n;
}
