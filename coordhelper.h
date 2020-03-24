#ifndef _coordhelper_h_
#define _coordhelper_h_
#endif

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
};
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
};
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
};
#pragma pack()

#pragma pack(1)
struct sMQTTCoordinatorHealth
{
	//12 bytes
	float Voltage;
	float Temp;
	float Hum;
};
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
};
#pragma pack()

#pragma pack(1)
struct sMQTTPeriodical
{
	sMQTTPreamble preamble;//11 bytes
	sMQTTCoordinatorHealth coordhealth; //12 bytes
	sMQTTWPSinterrim interrim;//19 bytes
	sMQTTWPScfg wpscfg;//28 bytes
};
#pragma pack()

#pragma pack(1)
struct sMQTTinterrim
{
	sMQTTPreamble preamble;//11 bytes
	sMQTTCoordinatorHealth coordhealth;//12 bytes
	sMQTTWPSinterrim interrim;//19 bytes
};
#pragma pack()


// ----------------------------------------------------------------------------------------------
// ------------------------                               ---------------------------------------
//							      RADIO DEFINITIONS
// ------------------------                               ---------------------------------------
// ----------------------------------------------------------------------------------------------
#pragma pack(1)
struct sLORAcomm
{
	uint8_t WPSAddress;
	bool FirstPacketReceived;
	bool ACTSend;
	bool FINReceived;
	bool MQTTmsgSend;
	sWPSData wpsdata;
	sLostStruct loststruct;
	sCoordData coordinatordata;
};
#pragma pack()