#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/object.h"
#include "ns3/olsr-helper.h"
#include "ns3/olsr-repositories.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/stats-module.h"
#include "ns3/wifi-module.h"

//#include "ns3/collision-avoidance-helper.h"

#include "ns3/MaliciousDroneHelper.h"
#include "ns3/SatelliteHelper.h"
#include "ns3/normal-drone-helper.h"

// #include "ns3/rng-seed-manager.h"

#include <ns3/itu-r-1411-los-propagation-loss-model.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/lr-wpan-spectrum-value-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/single-model-spectrum-channel.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef EXPERIMENT
#define EXPERIMENT

using namespace ns3;
using namespace std;

enum MethodName
{
	NormalCase,
	AuthenticWithSpoofing,
	Spoofing10,
	Spoofing20,
	Spoofing30,
	OneSatellite,
	AllSatellite,
	Burglary,
	AbsolutePower,
	AbsolutePower_CN0
};

struct limite
{
    double MaxX;
    double MaxY;
    double MaxZ;

    double MinX;
    double MinY;
    double MinZ;

    int NbrOfUAV;

    double Speed;
};

struct powerCN
{
	double power;
	double cn;
};

class Experiment : public Object
{
 public:
    static TypeId GetTypeId(void);
	Experiment();
	~Experiment();

 public:
	// std::vector<Gnuplot2dDataset> Run(MethodName method, bool Dynamic, bool onStateChange);
	// void Run(MethodName method, bool Dynamic, bool onStateChange);
	Gnuplot2dDataset Run(MethodName method, bool Dynamic, bool onStateChange);

	int numberOfMalicious;
 	
	// Helpers
	void SetDefaults();
	void Configure();

    void CreateNodesLrWPAN(int NbrUAV);
	void AffectationDesAddress_LrWPAN(uint32_t debut, uint32_t fin);
	void AffectationDesAddress_GPS_PRN1(uint32_t debut, uint32_t fin, uint32_t PRN_index);

	void InstallRandomMobilityLrWPAN(uint32_t debut, uint32_t fin, limite LimiteG1, limite LimiteG2, limite LimiteG3);

	void InitialisationChannelLrwpan();
	void InitialisationChannelGPS();

	void InstallChannelLrwpan(uint32_t debut, uint32_t fin);
	void InstallChannelGPS(uint32_t debut, uint32_t fin);

	void InstallDevicesLrwpan(uint32_t debut, uint32_t fin);
	void InstallDevicesGPS(uint32_t debut, uint32_t fin);

	void InstallApps(uint32_t debut, uint32_t fin, uint32_t type);

	void CreateNSatellite(int NbrSatellite, double TxPower, double StartTime);
	void CreateOneSatellite(double x, double y, double z, uint32_t PRN, double TxPower, double StartTime);
	void StartSatelliteApp(Ptr<Node> satellite, Ptr<LrWpanNetDevice> dev_satellite, double TxPower, uint32_t PRN);


	std::string UniformRange(uint32_t min, uint32_t max);
	std::string WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY);
	
	void ConnectCallbacks();

	// Callbacks

	void Cn0VersusTSP(double receivingSatellitepower, double CN0);
	void Alert(double receivingSatellitepower, double CN0, double index);
	void Detection(double receivingSatellitepower, double CN0, double index);

	void TrustPerformance(double index);
	void SNRPerformance(double index);

	// void TotalNumber(double index);
	void DronePacketNumber(double index);
	void SatelliteNumber(double index);
	
	// void TotalReceivedNumber(double index);
	void DroneReceivedNumber(double index);
	void SatelliteReceivedNumber(double index);

	Ptr<LrWpanNetDevice> GetDevice(uint16_t i);

	// Packets
	int totalReceivedPackets = 0;
	int droneReceivedPackets = 0;
	int satelliteReceivedPackets = 0;

	// Packets
	int totalNumberOfPackets = 0;
	int totalDronePackets = 0;
	int totalSatellitePackets = 0;

	// Trust Packet Counters
	double trust_True_Positive = 0;
	double trust_False_Positive = 0;
	double trust_False_Negative = 0;
	double trust_True_Negative = 0;

	// Satellite Packet Counters
	double satellite_True_Positive = 0;
	double satellite_False_Positive = 0;
	double satellite_False_Negative = 0;
	double satellite_True_Negative = 0;

	// Packet Counters
	double m_True_Positive = 0;
	double m_False_Positive = 0;
	double m_False_Negative = 0;
	double m_True_Negative = 0;

	uint32_t m_numVictim = 0;
	uint32_t m_numUAV = 0;


 private:
	int m_packetsSent;
	double m_SpoofingSatellitePower;
	int cp_packetsSent;
	int cp_packetsReceived;
	double m_ReceivingSatellitepower;
	double m_CN0;
	// *** //

	// vector<double, double> powerCNVector;
	vector<powerCN> powerCNVector;
	double maxPower = 1000.00;
	double minPower = -1000.00;

	// Power management shared memory
	std::vector<double> m_phyStats;
	std::vector<Ptr<WifiPhy>> m_phyList;

	// Power management statistics
	double m_maxStat;
	double m_minStat;
	// Power management configs
	bool m_onStateChange;
	bool m_Dynamic;
	MethodName m_method;
	std::string m_methodString;

	// *** //

	// Drone Info
	NodeContainer m_drones;
	uint32_t m_numDrones = 0;
	// uint32_t m_numVictim = 0;
	// uint32_t m_numUAV = 0;
	
	ApplicationContainer m_DroneApps;
	ApplicationContainer m_SatelliteApps;
	ApplicationContainer m_SpoofingSatelliteApps;
	// *** //

	// LRWPan
	std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesTable;
	std::vector<Ptr<ConstantPositionMobilityModel>> m_ConstantPositionMobilityTable;
	std::vector<Ptr<RandomWalk2dMobilityModel>> m_RandomMobilityTable;
	EventId m_sendEvent;
	// *** //

	Ptr<MultiModelSpectrumChannel> channel;
	Ptr<ItuR1411LosPropagationLossModel> model;
	Ptr<ConstantSpeedPropagationDelayModel> delayModel;

	// GPS Receiver
	Ptr<MultiModelSpectrumChannel> GPSchannel;
	Ptr<FriisPropagationLossModel> GPSmodel;
	Ptr<ConstantSpeedPropagationDelayModel> GPSdelayModel;

	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN0;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN1;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN2;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN3;

	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN4;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN5;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN6;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN7;

	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN8;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN9;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN10;
	std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN11;

	// *** //

	// Satellite :
	uint32_t m_numSatellites = 0;
	NodeContainer m_satellites;

	// Ptr<LrWpanNetDevice> dev_sattelite ;
	std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesSatelliteTable;

	// Drone Groupe1 Info
	uint32_t m_maxX;
	uint32_t m_minX;
	uint32_t m_maxY;
	uint32_t m_minY;
	// *** //

	// Drone Communication Info
	uint32_t m_PacketSize; // bytes
	double m_DataUavFrequency; // seconds
	uint32_t m_imagePacketSize;
	double m_imageFrequency;
	// *** //

	// PHY Info
	std::string m_phyMode;
	// *** //

	// Throughput
	Gnuplot2dDataset m_success_rate;
	Gnuplot2dDataset m_cn0VersusAbsolutPower;
	Gnuplot2dDataset m_authenticSignal;

	uint32_t m_dataReceived;
	uint32_t m_dataSent;
	uint32_t cp_dataReceived;
	uint32_t cp_dataSent;

	double txPowerLrwpan = 10;
	uint32_t channelNumber = 11;
	uint32_t GPSchannelNumber = 11;

	int UAV_CONSTANTE_MOBILITY = 0;
	int UAV_RANDOM_WALK_MOBILITY = 1;
	int UAV_HERARCHICAL_MOBILITY = 2;
	int UAV_GRID_HERARCHICAL_MOBILITY = 3;

	uint32_t UAV_GOOD = 1;
	uint32_t UAV_BAD = 2;
	uint32_t UAV_VICTIM = 3;
};

#endif