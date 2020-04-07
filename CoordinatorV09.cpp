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

// SPI �zerinden RFM95 kullan�m� i�in tan�mlar...
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

char TIMESTMP_COMPUTER[19];							// zaman damgas� de�i�ken tan�m�
													//uint32_t IPADDRESS;


													/*// THREAD ���N
													thread *backgroundThread = nullptr;
													atomic_bool stopBackgroundThread;
													mutex dataMutex;
													*/


													// ----------------------------------------------------------------------------------------------
													// ------------------------                               ---------------------------------------
													//							COORDINATOR - SERVER PACKET ARRAYS
													// ------------------------                               ---------------------------------------
													// ----------------------------------------------------------------------------------------------
char coordinator_data[256];		// MAX PACKET TO SOCKET SERVER 256 bytes... MQ kullan�l�rsa gerek yok
char ServerReplyMsg[256];		// MAX PACKET FROM SOCKET SERVER 256 bytes... MQ kullan�l�rsa gerek yok
uint8_t PacketType = 0;			// COORDINATOR TO SOCKET SERVER PACKET TYPE. VEHICLE STATUS CHANGE PAKED�N� �NCEL�KLEND�RMEKTE KULLANILACAK... 
								// MQ kullan�l�rsa gerek yok


								// ----------------------------------------------------------------------------------------------
								// ------------------------                               ---------------------------------------
								//							WPS LOST VARIABLES AND CLASSES
								// ------------------------                               ---------------------------------------
								// ----------------------------------------------------------------------------------------------
uint8_t NUMBER_OF_LOST_WPS;
//uint8_t WPS_LOST_COUNT = def_WPS_LOST_COUNT; //NUMBER OF TRANSMISION CYCLES TO ASSUME WPS LOST
uint8_t DETECTED_WPS = 0;
int WPSSignalSending = 0; // Paket yollayan WPS say�s�
						  //uint8_t WPSRegistered = 0; // registered WPS dosyas�ndan okunan WPS say�s�
unsigned long LostCheckTimer = 0;		// lost wps i�in timeout sayac�. olarak �al���yor.
										//uint8_t WPS_ID_CORRECTED = 0; //COORDINATOR ADRESINE G�RE D�ZELT�LM�� WPS ID ...LOST CHECK ��LEM� ���N
struct sWPSLostCheckObj
{
	uint16_t dummy;
	uint8_t SleepCount;
	uint8_t IsLost;
	uint8_t WasLost;
	uint8_t WPSAddress;
	bool OnRegisteredWPSFile;
	bool SignalSend;
	uint64_t WPSSerial;
	uint32_t serialmsb;
	uint32_t seriallsb;
	unsigned long LostTimer;

	//bool StatusChange;
};
//cWPSLostCheckObj WPSLostCheckObj[100];
vector <sWPSLostCheckObj> WPSLostCheckObj;

struct sWPS_Config
{
	uint32_t serialmsb;
	uint32_t seriallsb;
	uint8_t address;
	uint8_t coordinator_address;
	uint8_t network_id;
	uint8_t gain;
	uint8_t sleep_count;
	uint8_t vdc;			//vehicle detection count
	uint8_t set_power;
	uint8_t set_config;		//0=NONE
							//BIT 0 = RADIO TX POWER
							//BIT 1 = VEHICLE DETECTION COUNT
							//BIT 2 = VECTOR VARIANCE LIMIT
							//BIT 3 = SLEEP COUNT
							//BIT 4 = GAIN
							//BIT 5 = COORDINATOR ADDRESS & FRQ
							//BIT 6 = WPS ADDRESS
							//BIT 7 = NETWORK ID
	uint8_t reset;	    	//0=NONE
							//2=RESET VECTOR0
							//3=RESET WPS
	float freq;
};
vector <sWPS_Config> wpsnewconfig;

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
	uint32_t dummy;
	uint8_t WPSAddress;
	bool ACTSend;
	bool FINReceived;
	bool MQTTmsgSend;
	uint8_t data_from_wps[WPS_PACKET_LENGHT + 2];// DEFINE VARIABLE FOR WPS PACKET LENGHT
	uint8_t data_to_wps[COORDINATOR_PACKET_LENGHT + 2];// DEFINE VARIABLE FOR COORDINATOR PACKET LENGHT
};
#pragma pack()
vector <sLORAcomm> LORAcomm;

class cCoordinator_Config
{
public:
	float frq;
	uint8_t registered_wps;
	uint8_t lost_wps_count; // LOST WPS SENSOR COUNT i�in initial de�er....
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
} coordinator;

// MQTT TANIMLAMASI
struct mosquitto *mosq = NULL;
char client_id[] = { 'c','o','o','r','d','-','0','0','0','0','0','0','\0'}; //MQTT Client ID ilk de�eri
char *MQTTtopic = "wpsdata";
//Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;
void sig_handler(int sig)
{
	printf("\n%s Break received, exiting!\n", __BASEFILE__);
	force_exit = true;
}


// ----------------------------------------------------------------------------------------------
// ------------------------                               ---------------------------------------
//							  FONKS�YON DEKLARASYONLARI
// ------------------------                               ---------------------------------------
// ----------------------------------------------------------------------------------------------
char* datetime_f(char dates[]);						// zaman� string haline getirmek i�in
char* int_to_str(int i, char b[], int format_,int Base);		// datetime_f i�in kullan�lan fonksiyn
bool LORARead(); //WPS �zerinden paket okumak ve sonland�rmak i�in
void SPI_setup(); // SPI setup i�lemleri
void RF95_setup(uint8_t RadioPower, uint8_t Coordinator_Address, uint8_t Network_ID, float RFFrequency); //Radyo setup i�lemleri
void SendToWPS(uint8_t LORAWPSindex); // WPS �zerine veri yollama fonksiyonu
void WPS_DATA_print(uint8_t printindex, uint8_t len, uint8_t from, uint8_t to, uint8_t id, uint8_t flags, int16_t Rssi, uint8_t checksum, uint8_t checksumreceived); // debug print
void PrepareDataToWPS(uint8_t LORAWPSindex, uint8_t flagtype); // WPS'e yollanacak verileri haz�rlama fonksiyonu
bool PrepareCoordinator(); // Program �al��t�r�ld���nda coordinator.conf dosyas� okuma ve yazma
bool ReadCoordinatorConfig(); //coordinator.conf read i�in
bool WriteConfig(); //coordinator.conf write i�in... config de�i�ti�inde t�m dosyay� yeniden olu�turacak
void WPSLostCheckINIT(uint8_t wpscount);
void LostCheck();
bool RWRegisteredWPSCfg(char ReadWrite, uint8_t WPSAddress);
uint8_t LORAPacketIndex(uint8_t from, bool recivedpacket);
bool SendMessageMQTT(uint8_t LORAWPSindexOnFIN, uint8_t MessageType);
void FindIPAddress();
uint32_t hex2dec(const char*hexvalue, int lenght);
void test_output();

uint16_t readvoltage();

uint8_t test_msqtt[54];

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

	LORAcomm.resize(coordinator.max_wps);

	WPSLostCheckObj.resize(coordinator.max_wps);
	WPSLostCheckINIT(coordinator.max_wps);

	RWRegisteredWPSCfg('R', 0);

	wpsnewconfig.resize(coordinator.max_wps);

	//MQTT ClientID, Coordinator.Serial ile birlikte olu�turulacak ..
	
	
	char stringnumber[6];
	int_to_str(coordinator.serial, stringnumber, 1,16);
	
	for (int i = 0; i< 6; i++)
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

			// LostCheck i�lemi i�in timer ayar� 60 sn
			if ((millis() - LostCheckTimer) > 60000) // 60 sn'de bir Lost_Check() yap�lacak
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
			bcm2835_delay(200);
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
		exit(-1);
	}
	usleep(10000);
	//////////////////////////////
	// wait for conversion complete
	do {
		if (read(fd, buf, 2) != 2) {
			perror("Read conversion");
			exit(-1);
		}
	} while (!(buf[0] & 0x80));
	usleep(10000);
	//////////////////////////////
	// read conversion register
	buf[0] = 0;   // conversion register is 0
	if (write(fd, buf, 1) != 1) {
		perror("Write register select");
		exit(-1);
	}
	usleep(10000);
	if (read(fd, buf, 2) != 2) {
		perror("Read conversion");
		exit(-1);
	}

	//////////////////////////////
	// convert output and display results
	val = (int16_t)buf[0] * 256 + (uint16_t)buf[1];
	//printf("Conversion %02x %02x %d %f\n", buf[0], buf[1], val, (float)val*23.48 / 32768);

	close(fd);


	return val;


}

bool SendMessageMQTT(uint8_t LORAWPSindexOnFIN, uint8_t MessageType)
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


	// MQTT Paket haz�rlama
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);

	int MQTTMsgLen = 0;

	switch (MessageType)
	{
	case MSG_INITIAL:
	{
		MQTTMsgLen = 28;
		break;
	}
	case MSG_PERIODICAL:
	{
		MQTTMsgLen = 54;
		break;
	}
	case MSG_VEHICLE_STATUS:
	{
		MQTTMsgLen = 54;
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
		/*MQTTMsg[17] = (uint8_t)(coordinator.frq >> 24);			// 17 - FRQ 4th
		MQTTMsg[18] = (uint8_t)(coordinator.frq >> 16);				// 18 - FRQ 3rd
		MQTTMsg[19] = (uint8_t)(coordinator.frq >> 8);				// 19 - FRQ 2nd
		MQTTMsg[20] = (uint8_t)(coordinator.frq);					// 20 - FRQ 1st
		*/
		MQTTMsg[21] = (uint8_t)coordinator.lost_wps_count;			// 21 - LOST WPS COUNT
		MQTTMsg[22] = (uint8_t)coordinator.registered_wps;			// 22 - REGISTERED WPS COUNT
		MQTTMsg[23] = (uint8_t)(coordinator.IPaddress >> 24);		// 23 - IP ADDRESS 1st OCTET
		MQTTMsg[24] = (uint8_t)(coordinator.IPaddress >> 16);		// 24 - IP ADDRESS 2nd OCTET
		MQTTMsg[25] = (uint8_t)(coordinator.IPaddress >> 8);		// 25 - IP ADDRESS 3rd OCTET
		MQTTMsg[26] = (uint8_t)(coordinator.IPaddress);				// 26 - IP ADDRESS 4th OCTET
		MQTTMsg[27] = (uint8_t)(coordinator.serial>>16);			// 27 - SERIAL 3rd OCTET




		//printf("FRQ : %d %d %d %d\n", MQTTMsg[17], MQTTMsg[18], MQTTMsg[19], MQTTMsg[20]);
		//printf("IP ADDRESS : %d.%d.%d.%d\n", MQTTMsg[26], MQTTMsg[25], MQTTMsg[24], MQTTMsg[23]);
		break;
	}
	case MSG_PERIODICAL:
	case MSG_VEHICLE_STATUS:
	{
		MQTTMsg[9] = (uint8_t)coordinator.address;											// 09 - COORDINATOR_ADDRESS
		uint16_t voltage = readvoltage();

		MQTTMsg[10] = voltage >> 8;															// 10 - COORDINATOR_BATTERY_VALUE MSB
		MQTTMsg[11] = voltage;																// 11 - COORDINATOR_BATTERY_VALUE LSB
		MQTTMsg[12] = (uint8_t)coordinator.SWversion_Major;									// 12 - COORDINATOR SW_VERSION MAJOR
		MQTTMsg[13] = (uint8_t)coordinator.SWversion_Minor;									// 13 - COORDINATOR SW_VERSION MINOR
		MQTTMsg[14] = (uint8_t)coordinator.HWversion;										// 14 - COORDINATOR HW_VERSION
		MQTTMsg[15] = (uint8_t)(LORAcomm[LORAWPSindexOnFIN].data_to_wps[4] * (-1));			// 15 - COORDINATOR RSSI ---- D�ZELT�P YOLLA.
		MQTTMsg[16] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[0];				// 16 - WPS NW ID
		MQTTMsg[17] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].WPSAddress;						// 17 - WPS ADDRESS
		MQTTMsg[18] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[2];				// 18 - WPS Sequence 3rd Octet
		MQTTMsg[19] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[3];				// 19 - WPS Sequence 2nd Octet
		MQTTMsg[20] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[4];				// 20 - WPS Sequence 1st Octet
		MQTTMsg[21] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[5];				// 21 - WPS SENSOR TEMP MSB
		MQTTMsg[22] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[6];				// 22 - WPS SENSOR TEMP LSB
		MQTTMsg[23] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[14];				// 23 - WPS Serial 6th Octet - ESK� WPS VECTOR X MSB
		MQTTMsg[24] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[15];				// 24 - WPS Serial 5th Octet - ESK� WPS VECTOR X LSB
		MQTTMsg[25] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[16];				// 25 - WPS Serial 4th Octet - ESK� WPS VECTOR Y MSB
		MQTTMsg[26] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[17];				// 26 - WPS Serial 3rd Octet - ESK� WPS VECTOR Y LSB
		MQTTMsg[27] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[18];				// 27 - WPS Serial 2nd Octet - ESK� WPS VECTOR Z MSB
		MQTTMsg[28] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[19];				// 28 - WPS Serial 1st Octet - ESK� WPS VECTOR Z LSB
		MQTTMsg[29] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[7];				// 29 - WPS VECTOR VARIANCE
		MQTTMsg[30] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[8];				// 30 - WPS Vehicle Status
		MQTTMsg[31] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[9];				// 31 - WPS BATT VALUE MSB
		MQTTMsg[32] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[10];				// 32 - WPS BATT VALUE LSB
		MQTTMsg[33] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[11];				// 33 - WPS_BATTERY_LEVEL

		uint16_t temprssi = (-1)*(int16_t)((uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[12] * 256 + (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[13]);
		MQTTMsg[34] = (uint8_t)(temprssi >> 8);												// 34 - WPS RSSI MSB
		MQTTMsg[35] = (uint8_t)(temprssi);													// 35 - WPS RSSI LSB
		MQTTMsg[36] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[17];				// 36 - WPS Serial 3rd Octet
		MQTTMsg[37] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[18];				// 37 - WPS Serial 2nd Octet
		MQTTMsg[38] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[19];				// 38 - WPS Serial 1st Octet
		MQTTMsg[39] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[20] >> 4;			// 39 - WPS HW Version
		MQTTMsg[40] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[20] & 0b00001111;	// 40 - WPS SW Version Major
		MQTTMsg[41] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[21];				// 41 - WPS SW Version Minor
		MQTTMsg[42] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[22] >> 2;			// 42 - WPS Vector Variance Limit
		MQTTMsg[43] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[22] & 0b00000011;	// 43 - WPS Vehicle Detection Count
		MQTTMsg[44] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[23] & 0b00011111;	// 44 - WPS Radio TX Power
		MQTTMsg[45] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[24] >> 6;			// 45 - WPS Sensor Gain
		MQTTMsg[46] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[24] & 0b00111111;	// 46 - WPS Sleep Count
		MQTTMsg[47] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[23] & 0b01100000;
		MQTTMsg[47] = MQTTMsg[47] >> 5;														// 47 - BLE TX POWER -ESK�  WPS Frequency 4th Octet
		MQTTMsg[48] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[25];				// 48 - LORA CHANNEL - ESK� WPS Frequency 3rd Octet
		MQTTMsg[49] = 0;// (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[15];			// 49 - BO�
		MQTTMsg[50] = (uint8_t)(coordinator.serial >> 16);									// 50 - COORDINATOR SERIAL 3rd OCTET
		MQTTMsg[51] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[26];				// 51 - WPS ERROR CODE
		MQTTMsg[52] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[27] >> 4;			// 52 - BLE TIMEOUT
		MQTTMsg[53] = (uint8_t)LORAcomm[LORAWPSindexOnFIN].data_from_wps[27] & 0b00001111;	// 53 - LoRA RETRY COUNT


																							//for (int i = 0; i < 54; i++) test_msqtt[i] = MQTTMsg[i];
																							//test_output();

		break;
	}
	case MSG_WPS_LOST:
	{
		break;
	}

	}




	cout << MQTTtopic << endl;
	cout << MQTTMsg << endl;

	ret = mosquitto_publish(mosq, NULL, MQTTtopic, MQTTMsgLen, MQTTMsg, 0, false);
	if (ret)
	{
		fprintf(stderr, "Can't publish to Mosquitto server\n");
		delete[] MQTTtopic;

		//exit(-1);
	}



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
	for (uint8_t i = 0; i<wpscount; i++)
	{
		WPSLostCheckObj[i].LostTimer = 1000;
		WPSLostCheckObj[i].SleepCount = def_WPS_SLEEP_COUNT;
		WPSLostCheckObj[i].IsLost = 0;
		WPSLostCheckObj[i].WasLost = 0;
		WPSLostCheckObj[i].WPSAddress = 0;
		WPSLostCheckObj[i].WPSSerial = 0;
		WPSLostCheckObj[i].serialmsb = 0;
		WPSLostCheckObj[i].seriallsb = 0;
		WPSLostCheckObj[i].SignalSend = false;
		WPSLostCheckObj[i].OnRegisteredWPSFile = false;
		// ALTTAK� SATIRLAR LORAComm init i�lemleri i�in
		LORAcomm[i].WPSAddress = 0;
		LORAcomm[i].ACTSend = false;
		LORAcomm[i].FINReceived = false;
		LORAcomm[i].MQTTmsgSend = false;
	}
}

void LostCheck()
{
	uint8_t NewLostCount = 0;
	uint8_t NewFoundCount = 0;
	unsigned long delta = 0;

	printf(" LOST CHECK FUNCTION....\n");
	//printf(" WPS Count In File : %d\n", coordinator.registered_wps);
	for (uint8_t i = 0; i < coordinator.registered_wps; i++)
	{
		//printf(" %d		",WPSLostCheckObj[i].WPSAddress);
		//WPSSignalSending += WPSLostCheckObj[i].SignalSend;
		delta = millis() - WPSLostCheckObj[i].LostTimer;
		if (delta >((SleepCycleFactor * 8000) * (long)(WPSLostCheckObj[i].SleepCount)*(long)(coordinator.wps_lost_cycle)))
		{
			if (WPSLostCheckObj[i].IsLost == 0)
			{
				WPSSignalSending--;
				NUMBER_OF_LOST_WPS++;
				WPSLostCheckObj[i].IsLost = 1;
				WPSLostCheckObj[i].SignalSend = false;
			}

			printf(" WPS Address # %d # is lost...\n", WPSLostCheckObj[i].WPSAddress);
		}

		if (WPSLostCheckObj[i].IsLost > WPSLostCheckObj[i].WasLost)
		{
			NewLostCount++;//YEN� LOST WPS VAR
						   //WPSLostCheckObj[i].StatusChange = true;
			WPSLostCheckObj[i].WasLost = WPSLostCheckObj[i].IsLost;
			printf(" Number of new lost wps : %d\n", NewLostCount);
		}

	}
	printf("\n");

	if (NewLostCount> 0)
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

		// IF FILE EXIST THEN READ REGISTERED to WPSLostCheckObj VECTOR
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
					uint32_t msbserial = atoi(token[2]);
					uint32_t lsbserial = atoi(token[3]);
					uint8_t sleepcount = (uint8_t)atoi(token[4]);
					uint8_t islost = (uint8_t)atoi(token[5]);
					uint8_t waslost = (uint8_t)atoi(token[6]);
					uint64_t wpsserial = (uint64_t)msbserial << 24 | lsbserial;
					/*printf("1. %d\n", wpsaddress);
					printf("2. %.6x\n", msbserial);
					printf("3. %.6x\n", lsbserial);
					printf("4. %d\n", sleepcount);
					printf("5. %s\n", islost);
					printf("6. %s\n", waslost);*/

					WPSLostCheckObj[coordinator.registered_wps].WPSAddress = wpsaddress;

					WPSLostCheckObj[coordinator.registered_wps].serialmsb = msbserial;
					WPSLostCheckObj[coordinator.registered_wps].seriallsb = lsbserial;
					WPSLostCheckObj[coordinator.registered_wps].WPSSerial = wpsserial;
					//printf("%.12x\n", (uint64_t)msbserial<<24);
					WPSLostCheckObj[coordinator.registered_wps].SleepCount = sleepcount;
					//printf("%d\n", WPSLostCheckObj[coordinator.registered_wps].SleepCount);
					WPSLostCheckObj[coordinator.registered_wps].IsLost = islost;
					WPSLostCheckObj[coordinator.registered_wps].WasLost = waslost;
					WPSLostCheckObj[coordinator.registered_wps].OnRegisteredWPSFile = true;
					//WPSLostCheckObj[coordinator.registered_wps].StatusChange = false;
					WPSLostCheckObj[coordinator.registered_wps].SignalSend = false;

					printf("%d.	WPS Address =  %d	- WPS Serial = %.6X:%.6X	- WPS SleepCount = %d	- WPS Lost = ",
						(coordinator.registered_wps + 1),
						WPSLostCheckObj[coordinator.registered_wps].WPSAddress,
						WPSLostCheckObj[coordinator.registered_wps].serialmsb,
						WPSLostCheckObj[coordinator.registered_wps].seriallsb,
						WPSLostCheckObj[coordinator.registered_wps].SleepCount);
					if (WPSLostCheckObj[coordinator.registered_wps].IsLost == 0)
					{
						printf("FALSE\n");
					}
					else
					{
						printf("TRUE\n");
						NUMBER_OF_LOST_WPS++;
					}

					LORAcomm[coordinator.registered_wps].WPSAddress = WPSLostCheckObj[coordinator.registered_wps].WPSAddress;
					LORAcomm[coordinator.registered_wps].FINReceived = false;
					LORAcomm[coordinator.registered_wps].ACTSend = false;

					//WPSSignalSending += WPSLostCheckObj[coordinator.registered_wps].IsLost;
					coordinator.registered_wps++;

					//if (WPSAddress == 0) WPSSignalSending++;

				}

			}
		}
		printf(" Registered =	%d			Live =	%d			Lost =	%d\n", coordinator.registered_wps, WPSSignalSending, NUMBER_OF_LOST_WPS);
	}
	// APPEND FILE
	if (ReadWrite == 'A' || ReadWrite == 'a')
	{

		for (uint8_t i = 0; i < coordinator.max_wps; i++)
		{
			if (WPSLostCheckObj[i].WPSAddress == WPSAddress)  // WPSAddress ile WPSLostCheckObj.WPSAddress e�itli�ine bak
				if (WPSLostCheckObj[i].OnRegisteredWPSFile) return true; // && !WPSLostCheckObj[i].StatusChange) return true; // dosyada var ve status de�i�imi yoksa ��k..
				else
				{
					if (!WPSLostCheckObj[i].OnRegisteredWPSFile) // dosya i�erisinde yoksa dosyaya ekle...
					{
						fstream wpsfile;
						char buffer[10];

						wpsfile.open("registeredwps.conf", ios::out | ios::app);
						wpsfile << "# ";

						int_to_str(WPSLostCheckObj[i].WPSAddress, buffer, 0,10);
						wpsfile << buffer << " ";


						int_to_str(WPSLostCheckObj[i].serialmsb, buffer, 0,10);
						wpsfile << buffer << " ";
						int_to_str(WPSLostCheckObj[i].seriallsb, buffer, 0,10);
						wpsfile << buffer << " ";




						int_to_str(WPSLostCheckObj[i].SleepCount, buffer, 0,10);
						wpsfile << buffer << " ";

						int_to_str(WPSLostCheckObj[i].IsLost, buffer, 0,10);
						wpsfile << buffer << " ";

						int_to_str(WPSLostCheckObj[i].WasLost, buffer, 0,10);
						wpsfile << buffer << " " << endl;

						wpsfile.close();
						WPSLostCheckObj[i].OnRegisteredWPSFile = true;
						//WPSLostCheckObj[i].StatusChange = false;
						coordinator.registered_wps++;
						return true;
					}
				}

		}

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
			int_to_str(WPSLostCheckObj[i].WPSAddress, buffer, 0,10);
			wpsfile << buffer << " ";

			int_to_str((uint32_t)WPSLostCheckObj[i].serialmsb, buffer, 0,10);
			wpsfile << buffer << " ";

			int_to_str((uint32_t)WPSLostCheckObj[i].seriallsb, buffer, 0, 10);
			wpsfile << buffer << " ";

			int_to_str(WPSLostCheckObj[i].SleepCount, buffer, 0, 10);
			wpsfile << buffer << " ";

			int_to_str(WPSLostCheckObj[i].IsLost, buffer, 0, 10);
			wpsfile << buffer << " ";

			int_to_str(WPSLostCheckObj[i].WasLost, buffer, 0, 10);
			wpsfile << buffer << " " << endl;


			// WPS Adresi 0'dan farkl� girildiyse WPS ��kart�lm�� veya transfer edilmi� demektir. 
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

			if (string(token[i]) == "COORDINATOR_FREQ")
			{
				//printf(token[i + 2]);
				coordinator.frq = strtof(token[i + 2], 0);
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
				printf("Registered WPS count       = %d\n", coordinator.registered_wps);
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
		//					 BURAYI RASP� SER� NUMARASIN SON 3 BYTE'INI ALACAK �EK�LDE DE���T�R 
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
		file << "COORDINATOR_FREQ                   = " << DEFAULT_FREQ << endl;

		int_to_str(coordinator.address, stringnumber, 0, 10);
		file << "COORDINATOR_ADDRESS                = " << stringnumber << endl;

		file << "NETWORK_ID                         = " << def_NETWORK_ID << endl;

		file << "WPS_LOST_CYCLE                     = " << def_WPS_LOST_COUNT << endl;

		file << "MAX_WPS_NUMBER                     = " << MAX_WPS_NUMBER << endl;

		int_to_str(coordinator.registered_wps, stringnumber, 0, 10);
		file << "REGISTERED_WPS_COUNT               = " << stringnumber << endl;
		file << "MQTT_SERVER                        = " << TESTSERVER << endl;
		file << "MQTT_PORT                          = " << TESTPORT << endl;

		file << "SECRET                             = " << def_SECRET_CODE << endl;

		file << "MQTT_PASSWORD                      = " << def_MQTT_PASSWORD << endl;
		file << "MQTT_USERNAME                      = " << def_MQTT_USERNAME << endl;
		file << "MQTT_TOPIC                         = " << def_MQTT_TOPIC << endl;
		file.close();//File must be properly closed if opened! Else issues may arise.

		ReadCoordinatorConfig();
	}
	return true;
}

bool LORARead(void)
{
	//*******************************************************************************
	//*******************   RADIO READING		 ************************************
	//*******************************************************************************
	uint16_t checksumreceived, checksum;


	// Should be a message for us now
	uint8_t data_from_wps_temp[WPS_PACKET_LENGHT];
	uint8_t len = sizeof(data_from_wps_temp);
	uint8_t from = driver.headerFrom();
	uint8_t to = driver.headerTo();
	uint8_t id = driver.headerId();
	uint8_t flags = driver.headerFlags();
	int16_t Rssi = driver.lastRssi();
	uint8_t LORAWPSindex = 0;

	if (driver.recv(data_from_wps_temp, &len))
	{
		printf("\n\n\n Packet Received..\n");
		printf(" from = %d\n", from);
		printf(" Network ID     = %d\n", data_from_wps_temp[0]);
		switch (data_from_wps_temp[1])
		{
		case 1: { printf(" Packet Type    = Periodical\n"); break; }
		case 2: { printf(" Packet Type    = Vehicle Status Change\n"); break; }
		case 4: { printf(" Packet Type    = Configuration Change Request\n"); break; }
		case 8: { printf(" Packet Type    = Acknowledge OK\n"); break; }
		case 16: { printf(" Packet Type    = Retransmit Request\n"); break; }
		case 32: { printf(" Packet Type    = FIN\n"); break; }
		}
		if (data_from_wps_temp[0] != coordinator.NW_id)
		{
			printf(" Wrong NW ID...\n");
			goto EndOfStory;
		}




		// FIN PAKED� DE��L �SE LORAcomm[LORAWPSindex] i�erisine data_from_wps_temp de�erini  ata.
		if ((data_from_wps_temp[1] != FLAG_FIN)) //|| (data_from_wps_temp[1] != FLAG_CONFIGURATION_SET))
		{
			checksumreceived = data_from_wps_temp[len - 1];
			checksum = 0;

			LORAWPSindex = LORAPacketIndex(from, NOT_FIN_PACKET);
			for (uint8_t i = 0; i < len - 1; i++)
			{
				checksum = checksum ^ data_from_wps_temp[i];
				LORAcomm[LORAWPSindex].data_from_wps[i] = data_from_wps_temp[i];
			}
			LORAcomm[LORAWPSindex].WPSAddress = from;
			LORAcomm[LORAWPSindex].ACTSend = false;
			LORAcomm[LORAWPSindex].FINReceived = false;


			if (checksum != checksumreceived)
			{
				printf(" CHECKSUM ERROR... FIRST CHECK...\n");
				printf(" CHECKSUM R/S:	%d	%d\n", checksum, checksumreceived);
				PrepareDataToWPS(from, FLAG_RETRANSMIT);
				SendToWPS(from);
				goto EndOfStory;

			}


			//LORAcomm[LORAWPSindex].WPSAddress = from;
			//LORAcomm[LORAWPSindex].ACTSend = false;
			//LORAcomm[LORAWPSindex].FINReceived = false;
			//for (uint8_t i = 0; i < len -1; i++)
			//{
			//	LORAcomm[LORAWPSindex].data_from_wps[i] = data_from_wps_temp[i];
			//	//printf(" %d - %.2X\n", i, data_from_wps_temp[i]);
			//}
			//

		}



		// GELEN PAKET FIN PAKEDI MI?
		// FIN PAKED� �SE WPSLostCheckObj de�erilerini ayarla...
		// Registiration ve Signal de�i�kenkerini ayarla....

		if ((data_from_wps_temp[1] == FLAG_FIN))// || (data_from_wps_temp[1] == FLAG_CONFIGURATION_SET))
		{
			printf(" FIN Packet Received from WPS address : %d\n", from);
			printf(" -------------------------------------------\n");
			uint8_t LORAWPSindexOnFIN = LORAPacketIndex(from, FIN_PACKET);
			printf(" LORA FIN index : %d\n", LORAWPSindexOnFIN);
			if (LORAcomm[LORAWPSindexOnFIN].ACTSend == false)
			{
				printf(" Uncompleted FIN packet...\n\n");
				goto EndOfStory;
			}

			for (uint8_t i = 0; i < (coordinator.max_wps); i++)
			{
				if (WPSLostCheckObj[i].WPSAddress == from)
				{
					if (WPSLostCheckObj[i].IsLost == 1)
					{
						printf(" This WPS was lost ... %d   \n", WPSLostCheckObj[i].WPSAddress);
						NUMBER_OF_LOST_WPS--;
						WPSLostCheckObj[i].IsLost = 0;
						WPSLostCheckObj[i].WasLost = 0;
						RWRegisteredWPSCfg('W', 0);

					}
					WPSLostCheckObj[i].LostTimer = millis();
					if (!WPSLostCheckObj[i].SignalSend)
					{
						WPSLostCheckObj[i].SignalSend = true;
						WPSSignalSending++;
					}
					break;
				}
				if (WPSLostCheckObj[i].WPSAddress == 0)
				{
					WPSLostCheckObj[i].WPSAddress = from;
					WPSLostCheckObj[i].IsLost = 0;
					WPSLostCheckObj[i].WasLost = 0;
					WPSLostCheckObj[i].LostTimer = millis();
					WPSLostCheckObj[i].SleepCount = (LORAcomm[LORAWPSindexOnFIN].data_from_wps[24]) & 0b00011111;
					WPSLostCheckObj[i].serialmsb =
						((uint32_t)(LORAcomm[LORAWPSindexOnFIN].data_from_wps[14]) << 16) |
						((uint32_t)(LORAcomm[LORAWPSindexOnFIN].data_from_wps[15]) << 8) |
						((uint32_t)(LORAcomm[LORAWPSindexOnFIN].data_from_wps[16]));
					WPSLostCheckObj[i].seriallsb =
						((uint32_t)(LORAcomm[LORAWPSindexOnFIN].data_from_wps[17]) << 16) |
						((uint16_t)(LORAcomm[LORAWPSindexOnFIN].data_from_wps[18]) << 8) |
						(LORAcomm[LORAWPSindexOnFIN].data_from_wps[19]);
					WPSLostCheckObj[i].WPSSerial = (uint64_t)WPSLostCheckObj[i].serialmsb << 24 | WPSLostCheckObj[i].seriallsb;
					WPSLostCheckObj[i].SignalSend = true;
					WPSSignalSending++;

					RWRegisteredWPSCfg('A', from);
					break;
				}
			}



			printf(" Live WPS count = %d/%d\n", WPSSignalSending, coordinator.registered_wps);
			for (uint8_t i = 0; i < (coordinator.max_wps); i++)
			{
				if (WPSLostCheckObj[i].SignalSend && (WPSLostCheckObj[i].IsLost == 0)) printf(" %d		", WPSLostCheckObj[i].WPSAddress);
			}
			cout << endl;
			LORAcomm[LORAWPSindexOnFIN].ACTSend = false;
			LORAcomm[LORAWPSindexOnFIN].FINReceived = true;
			LORAcomm[LORAWPSindexOnFIN].MQTTmsgSend = false;

			int tmp_mgstype = LORAcomm[LORAWPSindexOnFIN].data_from_wps[1];

			if (tmp_mgstype == 1)
			{
				cout << "Periodical Packet Will Be Sent.." << endl;
				SendMessageMQTT(LORAWPSindexOnFIN, MSG_PERIODICAL);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}
			if (tmp_mgstype == 2)
			{
				cout << "Vehicle Status Packet Will Be Sent.." << endl;
				SendMessageMQTT(LORAWPSindexOnFIN, MSG_VEHICLE_STATUS);
				mosquitto_disconnect(mosq);
				mosquitto_destroy(mosq);
				mosquitto_lib_cleanup();
			}


			goto EndOfStory;
		}
	}
	else
	{
		printf("receive failed\n");
		printf("-------------------------------------------\n");
		return false;
	}


	// CHECKSUM OK �SE ACK PAKETD� YOLLA....
	if (checksum == checksumreceived)
	{
		printf(" Checksum OK. Sending ACK Packet\n");
		printf("-------------------------------------------\n");
		PrepareDataToWPS(LORAWPSindex, FLAG_ACK_OK);
		//PrepareDataToWPS(LORAWPSindex, FLAG_CONFIGURATION_SET);

		SendToWPS(LORAWPSindex);
		LORAcomm[LORAWPSindex].ACTSend = true;
		LORAcomm[LORAWPSindex].FINReceived = false;
	}


	// GELEN VER�LER� EKRANA YAZ

	WPS_DATA_print(LORAWPSindex, len, from, to, id, flags, Rssi, checksum, checksumreceived);
	printf(" Index No : %d\n", LORAWPSindex);

	/*for (uint8_t i = 0; i < len - 1; i++)
	{
	LORAcomm[LORAWPSindex].data_from_wps[i] = data_from_wps_temp[i];
	printf(" %d - %.2X\n", i, data_from_wps_temp[i]);
	}
	printf(" Index No : %d\n", LORAWPSindex);
	*/


EndOfStory:
	printf("\n");
	return true;
}

uint8_t LORAPacketIndex(uint8_t from, bool receivedpacket)
{
	// FIN PAKED� DE��L �SE �LK E�LE�EN� VEYA 0 OLAN �LK �NDEX� D�NECEK...
	if (receivedpacket)
	{
		uint8_t i = 0;
		for (i = 0; i < coordinator.max_wps; i++)
		{
			if (LORAcomm[i].WPSAddress == from)
			{
				return i;
				break;
			}
			//DETECTED_WPS++;
			//return DETECTED_WPS-1;

			if (LORAcomm[i].WPSAddress == 0)
			{
				printf("\n NOT PREVIOUSLY SENDING WPS...\n");
				return i;
				break;

			}

		}
	}
	// FIN PAKET� �SE DAHA �NCEDEN KAYITLI OLMASI GEREKL� ...

	if (!receivedpacket)
	{
		for (uint8_t i = 0; i < coordinator.max_wps; i++)
		{
			if (LORAcomm[i].WPSAddress == from)
			{
				return i;
				break;
			}
		}
	}
}

void PrepareDataToWPS(uint8_t LORAWPSindex, uint8_t flagtype)
{
	uint8_t WPSADDRESS = LORAcomm[LORAWPSindex].WPSAddress;
	//driver.setHeaderFlags(flagtype,255);
	LORAcomm[LORAWPSindex].data_to_wps[0] = coordinator.NW_id;
	LORAcomm[LORAWPSindex].data_to_wps[1] = flagtype; // PAKET T�P�


	LORAcomm[LORAWPSindex].data_to_wps[2] = WPSADDRESS; // HANG� WPS ADRES�NE 
														//LORAcomm[LORAWPSindex].data_to_wps[2] = WPS_BROADCAST_ADDRESS; // T�M WPS ADRESLER�NE 

														// GELEN SEQUENCE ID




	LORAcomm[LORAWPSindex].data_to_wps[3] = 0; //VECTOR0/ WPS RESETLENS�N M�? (1=VECTOR0, 2=WPS SW)
	LORAcomm[LORAWPSindex].data_to_wps[4] = (int8_t)driver.lastRssi(); // WPS �ZER�NDEN ALINAN S�NYAL Rssi



	LORAcomm[LORAWPSindex].data_to_wps[5] = 1;			//YEN� CONF�G VAR MI
														//0=NONE
														//BIT 0 = RADIO TX POWER AND BLE TX POWER
														//BIT 1 = VEHICLE DETECTION COUNT
														//BIT 2 = VECTOR VARIANCE LIMIT
														//BIT 3 = SLEEP COUNT
														//BIT 4 = GAIN
														//BIT 5 = COORDINATOR ADDRESS & LORA CHANNEL
														//BIT 6 = WPS ADDRESS
														//BIT 7 = NETWORK ID

	LORAcomm[LORAWPSindex].data_to_wps[6] = GAIN_088; // Default GAIN_088
	LORAcomm[LORAWPSindex].data_to_wps[7] = def_NETWORK_ID; // Default 0xAE
	LORAcomm[LORAWPSindex].data_to_wps[8] = WPSADDRESS; // YEN� WPS ADRES�
	LORAcomm[LORAWPSindex].data_to_wps[9] = def_COORDINATOR_ADDRESS; // YEN� COORDINATOR ADRES� (Default=0x00)
	LORAcomm[LORAWPSindex].data_to_wps[10] = 30; // YEN� WPS_SLEEP_COUNT 
	LORAcomm[LORAWPSindex].data_to_wps[11] = 10; //YEN� VEKT�R SAPMA ORANI  (Default=10)

												 // BURAYA D�KKAT.... BLE TIME OUT 5-8 B�TLER�NE YAZILACAK...
	LORAcomm[LORAWPSindex].data_to_wps[12] = 2 << 4; //YEN� BLE TIME OUT 5-8 Bitlerine Yaz
	LORAcomm[LORAWPSindex].data_to_wps[12] = LORAcomm[LORAWPSindex].data_to_wps[12] | 1; //YEN� ARA� TESP�T KATSAYISI (Default=5)  1-4 Bitlerine yaz
	uint8_t loratxpwr, bletxpwr, txpwr;
	txpwr = 0;
	loratxpwr = 13;
	bletxpwr = 2;
	txpwr = bletxpwr << 5 + loratxpwr;
	LORAcomm[LORAWPSindex].data_to_wps[13] = txpwr; //WPS RADIO_TX POWER (MIN=5, MAX=23, DEFAULT=13)
	LORAcomm[LORAWPSindex].data_to_wps[14] = 0;  //LORA CHANNEL DEFAULT 0


}

void SendToWPS(uint8_t LORAWPSindex)
{
	/*uint8_t xxxx = 10;
	uint8_t tempdata[xxxx];

	for (int i = 0; i < xxxx; i++) tempdata[i] = i+100;
	driver.setHeaderTo(50);
	driver.waitCAD();

	driver.setModeTx();

	driver.send(tempdata, xxxx);
	Serial.println("Wait Sending\n");
	driver.waitPacketSent();
	driver.setModeRx();*/


	//uint8_t WPSADDRESS = LORAcomm[LORAWPSindex].WPSAddress;
	//for (int i = 1; i < COORDINATOR_PACKET_LENGHT; i++) LORAcomm[LORAWPSindex].data_to_wps[i] = i;
	driver.setModeRx();
	//Serial.println("CAD Control\n");

	driver.waitCAD();

	driver.setHeaderTo(LORAcomm[LORAWPSindex].WPSAddress);

	//Serial.println("Set to TX\n");
	driver.setModeTx();
	//Serial.println("Sending\n");
	driver.send(LORAcomm[LORAWPSindex].data_to_wps, sizeof(LORAcomm[LORAWPSindex].data_to_wps));
	//Serial.println("Wait Sending\n");
	driver.waitPacketSent();
	//Serial.println("Set to RX\n");

	//for (int i = 0; i < COORDINATOR_PACKET_LENGHT; i++)
	//{
	//printf("%d\n", LORAcomm[LORAWPSindex].data_to_wps[i]);
	//}


	driver.setModeRx();
	switch (LORAcomm[LORAWPSindex].data_to_wps[1])
	{
	case 1: { printf(" PERIODICAL PACKET SENT TO : %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	case 2: { printf(" VEHICLE STATUS CHANGE PACKET SENT TO: %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	case 4: { printf(" CONFIGURATION CHANGE REQUEST PACKET SENT TO: %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	case 8: { printf(" ACK_OK PACKET SENT TO : %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	case 16: { printf(" RETRANSMIT REQUEST PACKET SENT TO : %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	case 32: { printf(" FLAG_FIN PACKET SENT TO : %d\n", LORAcomm[LORAWPSindex].WPSAddress); break; }
	}


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

	/*for (int y = 0; y <len; y++)
	{
	printf(" %d - %.2X\n", y, LORAcomm[printindex].data_from_wps[y]);
	}
	printf(" Index No : %d\n", printindex);
	*/
	printf(" Header ID, Mess. Len: %d - %d\n", id, len);
	printf(" Flags, Rssi         : %d - (%d)\n", flags, Rssi);
	printf(" Coordinator Time    : %s\n", datetime_f(TIMESTMP_COMPUTER));

	printf(" WPS LoRA Channel    : %d \n", LORAcomm[printindex].data_from_wps[25]);

	printf(" WPS Address         : %d\n", from);
	printf(" WPS Serial          : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", LORAcomm[printindex].data_from_wps[14], LORAcomm[printindex].data_from_wps[15], LORAcomm[printindex].data_from_wps[16], LORAcomm[printindex].data_from_wps[17], LORAcomm[printindex].data_from_wps[18], LORAcomm[printindex].data_from_wps[19]);
	printf(" WPS Software Version: %d.%d\n", LORAcomm[printindex].data_from_wps[20] & 0b00001111, LORAcomm[printindex].data_from_wps[21]);
	printf(" WPS Sequence Number : %d\n", (uint32_t)((LORAcomm[printindex].data_from_wps[2] * 256 * 256 + LORAcomm[printindex].data_from_wps[3] * 256 + LORAcomm[printindex].data_from_wps[4])));

	printf(" WPS Vector Variance : %d % \n", LORAcomm[printindex].data_from_wps[7]);

	float temperature = 0;
	int16_t tmp = (int16_t)LORAcomm[printindex].data_from_wps[5] * 256 | LORAcomm[printindex].data_from_wps[6];
	uint8_t hwtype = LORAcomm[printindex].data_from_wps[20] >> 4;

	if (hwtype < 2)
	{
		temperature = (float)tmp / 128;
	}
	else
	{
		temperature = (float)tmp / 8;
	}
	temperature += 25;
	printf(" WPS Sensor Temp.    : %3.2f C\n", temperature); //((float)((int16_t)(data_from_wps_temp[5] * 256 + data_from_wps_temp[6]) / 128) + 25));

	printf(" Battery Level       : %d % \n", LORAcomm[printindex].data_from_wps[11]);
	printf(" Battery Voltage     : %3.2f V \n", ((float)((uint16_t)(LORAcomm[printindex].data_from_wps[9]) << 8 | LORAcomm[printindex].data_from_wps[10])*3.3 * 2 / 1024));
	printf(" Vehicle Detected    : "); if (LORAcomm[printindex].data_from_wps[8]) { printf("True\n"); }
	else { printf("False\n"); }
	//printf(" Config Applied      : "); if (data_from_wps_temp[22]) { printf("True\n"); }
	//else { printf("False\n"); }
	printf(" RADIO_RSSI WPS SIDE : %d dB\n", (int16_t)(LORAcomm[printindex].data_from_wps[12] << 8 | LORAcomm[printindex].data_from_wps[13]));
	printf(" RADIO_RSSI_COR SIDE : %d dB\n", Rssi);
	printf(" WPS HW Version      : %d (", hwtype);
	switch (hwtype)
	{
	case 1: {printf("Arduino+HMC5983)\n"); break; }
	case 2: {printf("Moteino+HMC5983)\n"); break; }
	case 3: {printf("Arduino+LIS3MDL)\n"); break; }
	case 4: {printf("Arduino+LSM303C)\n"); break; }
	case 5: {printf("328P+LSM303C)\n"); break; }
	case 6: {printf("328P+LSM303C+HM10)\n"); break; }
	default: {printf("HW NOT IDENTIFIED)\n"); break; }
	}
	printf(" WPS Error Code      : %d \n", LORAcomm[printindex].data_from_wps[26]);
	uint8_t bletimeout = 0;
	bletimeout = (LORAcomm[printindex].data_from_wps[27] & 0b11110000) >> 4;
	printf(" BLE TIMEOUT         : %1.2f mins\n", (float)bletimeout / 2);
	uint8_t bletxpower = LORAcomm[printindex].data_from_wps[23] & 0b01100000;
	bletxpower = bletxpower >> 5;

	printf(" BLE TX POWER        : %d (", bletxpower);
	switch (bletxpower)
	{
	case 0: printf("-23 dBm)\n"); break;
	case 1: printf("-6 dBm)\n"); break;
	case 2: printf("0 dBm)\n"); break;
	case 3: printf("6 dBm)\n"); break;
	}
	printf(" WPS Retry Count     : %d \n", (LORAcomm[printindex].data_from_wps[27] & 0b00001111));

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
	for (i = 0; i<4; i++) datestring[i] = sYear[i];
	datestring[i] = '-';
	for (i = 5; i<7; i++) datestring[i] = sMon[i - 5];
	datestring[i] = '-';
	for (i = 8; i<10; i++) datestring[i] = sDay[i - 8];
	datestring[i] = ' ';
	for (i = 11; i<13; i++) datestring[i] = sHour[i - 11];
	datestring[i] = ':';
	for (i = 14; i<16; i++) datestring[i] = sMin[i - 14];
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
	for (int i = 0, v = 0; i<length && s[i] != '\0'; i++)
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
