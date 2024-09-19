// #include "ns3/core-module.h"
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

#include "subdir/experiment.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("MainExperiment");

string averages = "";

int 
main(int argc, char* argv[])
{
	NS_LOG_INFO("Starting : main function");
	cout << "Starting : main function" << endl;

	// Grab current time as seed if no seed is specified
	time_t currentTime;
	time(&currentTime); 

	/* Creating GNUPlot Objects */
	//
	cout << "Creating GNUPlot Objects" << endl;

	ostringstream os;
	ofstream SpoofingOutPutStream_Threshold("results/SpoofingThreshold.plt");
	ofstream SpoofingOutPutStream_SuccessRate("results/Success_rate.plt");

	Gnuplot DiagramFileSpoofing_Threshold = Gnuplot("results/SpoofingThreshold.eps");
	Gnuplot DiagramFileSpoofing_Success_Rate = Gnuplot("results/Success_rate.eps");

	std::vector<Gnuplot2dDataset> DataSets_SpoofingThreshold = {};
	std::vector<Gnuplot2dDataset> DataSets_Success_Rate = {};
	

	DiagramFileSpoofing_Threshold.SetTitle("Figure1");
	DiagramFileSpoofing_Success_Rate.SetTitle("Figure2");

	DiagramFileSpoofing_Threshold.SetTerminal("pdf");
	DiagramFileSpoofing_Success_Rate.SetTerminal("pdf");

	DiagramFileSpoofing_Threshold.SetLegend("Spoofing power (dBW)", "C/N0");
	DiagramFileSpoofing_Success_Rate.SetLegend("", "%");

	DiagramFileSpoofing_Threshold.SetExtra("set key font \"Verdana,8\" \n\
	set xrange [-160:-140]\n\
	set yrange [35:55]\n\
	set grid\n\
	set style line 1 linewidth 5\n\
	set style increment user");

	DiagramFileSpoofing_Success_Rate.SetExtra(
		"set xtics(\"True_Positive\" 1 ,\"False_Positive\" 2, \"True_Negative\" 3,\"False_Negative\" 4) \n\
		set xrange [0:5] \n\
		set style data histogram \n\
		set style fill solid  \n\
		set style histogram clustered ");

	cout << "Finish Creating GNUPlot Objects" << endl;
	/* Finish Creating GNUPlot Objects */

	/* Experiment Initialisation */

	LogComponentEnable("MainExperiment", LOG_LEVEL_INFO);

	std::string phyMode("OfdmRate9Mbps");

	uint32_t numDrones = 100;
	uint32_t maxX = 1000; // in (m)
	uint32_t maxY = 1000; // in (m)
	// uint32_t maxZ = 1000; // in (m)
	uint32_t collPacketSize = 24; // kilobytes (KB)
	double collFrequency = 1; // 0.5 seconds
	uint32_t imagePacketSize = 5; // 5 megabytes (MB)
	double imageFrequency = 10.0; // seconds
	bool onStateChange = true;

	/* Command line Initialisation */
	CommandLine cmd;

	cmd.AddValue("numDrones", "number of drones in network", numDrones);
	cmd.AddValue("maxY", "position of wall (top) for drone box (meters)", maxY);
	cmd.AddValue("maxX", "position of wall (right) for drone box (meters)", maxX);
	cmd.AddValue("collPacketSize", "collision detection packet size (KB)", collPacketSize);
	cmd.AddValue("collFrequency", "collision detection packet frequency (seconds)", collFrequency);
	cmd.AddValue("imagePacketSize", "image packet size (MB)", imagePacketSize);
	cmd.AddValue("imageFrequency", "image packet frequency (seconds)", imageFrequency);
	cmd.AddValue("onStateChange", "whether to adjust gains while idle.", onStateChange);
	cmd.AddValue("seed", "seed for the RngSeedManager", currentTime);

	cmd.Parse(argc, argv);
	RngSeedManager::SetSeed(currentTime);
	
	Gnuplot packetLoss = Gnuplot("results/packet-loss-rate.png");
	Gnuplot throughput = Gnuplot("results/data-throughput.png");

	cout << "Creating Experiment Object" << endl;
	Ptr<Experiment> experiment;
	experiment = CreateObject<Experiment>();
	experiment->SetAttribute("NumDrones", UintegerValue(numDrones));
	experiment->SetAttribute("MaxX", UintegerValue(maxX));
	experiment->SetAttribute("MaxY", UintegerValue(maxY));
	experiment->SetAttribute("CollisionPacketSize", UintegerValue(collPacketSize));
	experiment->SetAttribute("CollisionPacketFrequency", DoubleValue(collFrequency));
	experiment->SetAttribute("ImagePacketSize", UintegerValue(imagePacketSize * 1024 * 1024));
	experiment->SetAttribute("ImagePacketFrequency", DoubleValue(imageFrequency));
	experiment->SetAttribute("PHYmode", StringValue(phyMode));

	averages = to_string(numDrones);

	cout << "Experiment Object Created" << endl;
	
	/* Finish Initialisation */

	// Start Experiment //
	// DataSets_Success_Rate = experiment->Run(Burglary, true, onStateChange);
	// experiment->Run(AllSatellite, true, onStateChange);
	Gnuplot2dDataset powerCNDataSet = experiment->Run(Spoofing10, true, onStateChange);
	powerCNDataSet.SetStyle(Gnuplot2dDataset::HISTEPS);

	Gnuplot powerCNDiagram = Gnuplot("powerCNDiagram.eps");
	powerCNDiagram.SetTitle("Power to C/N0 Diagram");
	powerCNDiagram.SetTerminal("pdf");
	powerCNDiagram.SetLegend("Received Power (dBW)", "C/N0");

	powerCNDiagram.SetExtra("set key font \'Verdana,8\" \n\
		set xrange[-155:-160]\n\
		set yrange [40:45]\n\
		set grid\n\
		set style line 1 linewidth 5\n\
		set style increment user");

	powerCNDiagram.AddDataset(powerCNDataSet);

	ofstream powerCNOutstream("powerCN.plt");

	ofstream performanceResults("performance.plt");

	performanceResults << "Number of Nodes = " << experiment->m_numUAV << endl;
	performanceResults << "totalNumberOfPackets = " << experiment->totalNumberOfPackets << endl;

	performanceResults << "totalDronePackets = " << experiment->droneReceivedPackets << endl;
	performanceResults << "droneReceivedPackets = " << experiment->droneReceivedPackets << endl;

	performanceResults << "totalSatellitePackets = " << experiment->totalSatellitePackets << endl;
	performanceResults << "satelliteReceivedPackets = " << experiment->satelliteReceivedPackets << endl;

	double temp = 0;

	// Performance
	performanceResults << endl;
	performanceResults << "SNR System Results" << endl;
	temp = experiment->satellite_True_Positive + experiment->satellite_False_Positive + experiment->satellite_False_Negative + experiment->satellite_True_Negative;
	performanceResults << "Totale = " << temp << endl;
	performanceResults << "satellite_True_Positive = " << experiment->satellite_True_Positive << " == " << (experiment->satellite_True_Positive / temp) << endl;
	performanceResults << "satellite_False_Positive = " << experiment->satellite_False_Positive << " == " << (experiment->satellite_False_Positive / temp) << endl;
	performanceResults << "satellite_False_Negative = " << experiment->satellite_False_Negative << " == " << (experiment->satellite_False_Negative / temp) << endl;
	performanceResults << "satellite_True_Negative = " << experiment->satellite_True_Negative << " == " << (experiment->satellite_True_Negative / temp) << endl;

	// Performance
	performanceResults << endl;
	performanceResults << "Trust System Results" << endl;
	temp = experiment->trust_True_Positive + experiment->trust_False_Positive + experiment->trust_False_Negative + experiment->trust_True_Negative;
	performanceResults << "Totale = " << temp << endl;
	performanceResults << "trust_True_Positive = " << experiment->trust_True_Positive << " == " << (experiment->trust_True_Positive / temp) << endl;
	performanceResults << "trust_False_Positive = " << experiment->trust_False_Positive << " == " << (experiment->trust_False_Positive / temp) << endl;
	performanceResults << "trust_False_Negative = " << experiment->trust_False_Negative << " == " << (experiment->trust_False_Negative / temp) << endl;
	performanceResults << "trust_True_Negative = " << experiment->trust_True_Negative << " == " << (experiment->trust_True_Negative / temp) << endl;

	powerCNDiagram.GenerateOutput(powerCNOutstream);
	
	powerCNOutstream.close();
	// experiment->Run(AuthenticWithSpoofing, true, onStateChange);

	
	// Tahar
	/*
	NS_LOG_INFO("Start : Debug CreateNodesLrWPAN()");
	experiment->CreateNodesLrWPAN(100);
	NS_LOG_INFO("Finish : Debug CreateNodesLrWPAN()");
	
	NS_LOG_INFO("Start : Debug AffectationDesAddress_LrWPAN()");
	// experiment->AffectationDesAddress_LrWPAN(0, 100);
	NS_LOG_INFO("Finish : Debug AffectationDesAddress_LrWPAN()");
	
	NS_LOG_INFO("Start : Debug AffectationDesAddress_GPS_PRN1()");
	experiment->AffectationDesAddress_GPS_PRN1(0, 100, 0);
	NS_LOG_INFO("Finish : Debug AffectationDesAddress_GPS_PRN1()");
	*/


	// *** //

	// Start Simulation
	/*
	AnimationInterface anim("Experiment.xml");
	Simulator::Stop(Seconds(10.0)); // 40.0 + 120.0
	Simulator::Run();
	Simulator::Destroy();
	*/
	// *** //


	return 0;
}


TypeId
Experiment::GetTypeId(void)
{
	NS_LOG_INFO("Start : GetTypeId");
	static TypeId tid =
		TypeId("ns3::Experiment")
			.SetParent<Object>()
			.SetGroupName("Experiments")
			.AddConstructor<Experiment>()
			.AddAttribute("CollisionPacketSize",
				"The size of the packets used for the CollisionAvoidanceApp.",
				UintegerValue(1024),
				MakeUintegerAccessor(&Experiment::m_PacketSize),
				MakeUintegerChecker<uint32_t>(1))
			.AddAttribute(
				"CollisionPacketFrequency",
				"The interval in seconds of how often CollisionAvoidance packets should be sent.",
				DoubleValue(1),
				MakeDoubleAccessor(&Experiment::m_DataUavFrequency),
				MakeDoubleChecker<double>())
			.AddAttribute(
				"ImagePacketSize",
				"The size of the packets used for the ImageManagementApp (app not yet created).",
				UintegerValue(5 * 1024 * 1024),
				MakeUintegerAccessor(&Experiment::m_imagePacketSize),
				MakeUintegerChecker<uint32_t>(1))
			.AddAttribute(
				"ImagePacketFrequency",
				"The interval in seconds of how often ImageManagementApp packets should be sent.",
				DoubleValue(10.0),
				MakeDoubleAccessor(&Experiment::m_imageFrequency),
				MakeDoubleChecker<double>())
			.AddAttribute("NumDrones",
						  "The number of drones to use in the experiment.",
						  UintegerValue(30),
						  MakeUintegerAccessor(&Experiment::m_numDrones),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("PHYmode",
						  "The PHY mode to use for the PHY layer.",
						  StringValue("OfdmRate6Mbps"),
						  MakeStringAccessor(&Experiment::m_phyMode),
						  MakeStringChecker())
			.AddAttribute("MaxX",
						  "The right most wall of the drone environment.",
						  UintegerValue(1000),
						  MakeUintegerAccessor(&Experiment::m_maxX),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("MinX",
						  "The left most wall of the drone environment.",
						  UintegerValue(0),
						  MakeUintegerAccessor(&Experiment::m_minX),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("MaxY",
						  "The upper most wall of the drone environment.",
						  UintegerValue(1000),
						  MakeUintegerAccessor(&Experiment::m_maxY),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("MinY",
						  "The bottom most wall of the drone environment.",
						  UintegerValue(0),
						  MakeUintegerAccessor(&Experiment::m_minY),
						  MakeUintegerChecker<uint32_t>(1));

	NS_LOG_INFO("Finish : GetTypeId");
	return tid;
}

Experiment::Experiment()
	: m_numDrones(100),
	  m_maxX(1000),
	  m_minX(0),
	  m_maxY(1000),
	  m_minY(0),
	  m_PacketSize(4),
	  m_DataUavFrequency(1),
	  m_imagePacketSize(5 * 1024 * 1024),
	  m_imageFrequency(10.0),
	  m_phyMode("OfdmRate6Mbps")

{
	NS_LOG_INFO("Start & Finish Experiment Constructor : initialiser automatique");
}

Experiment::~Experiment()
{
	NS_LOG_INFO("Start and Finish Experiment Destructor : without Initialization");
}

// std::vector<Gnuplot2dDataset>
// void
Gnuplot2dDataset 
Experiment::Run(MethodName method, bool Dynamic, bool onStateChange)
{
	NS_LOG_INFO("Start : Run");

	RngSeedManager::ResetStreamIndex();
	m_method = method;
	m_Dynamic = Dynamic;
	m_onStateChange = onStateChange;

	SetDefaults();

	// Initialize Number of UAVs
	int victim = 10;
	numberOfMalicious = 20;
	m_numVictim = victim;

	int uavs = 90 + victim; 
	m_numUAV = uavs;

	int NbrUAV = uavs ; //+ victim; // 100 + 20 // + references

	int debut = 0;

	limite G1;
	G1.MinX = 1.0;
	G1.MaxX = 30000.0;
	//G1.MaxX = 50000.0;

	G1.MinY = 1.0;
	G1.MaxY = 30000.0;
	// G1.MaxY = 100000.0;
	G1.MinZ = 1000.0;
	G1.MaxZ = 2000.0;
	// G1.MaxZ = 10000.0;

	G1.NbrOfUAV = NbrUAV; // 100;
	G1.Speed = 200.0;

	limite G2;
	G2.MinX = 10000.0;
	G2.MaxX = 20000.0;

	G2.MinY = 1.0;
	G2.MaxY = 10000.0;

	G2.MinZ = 1000.0;
	G2.MaxZ = 5000.0;

	G2.NbrOfUAV = 0;
	G2.Speed = 100.0;

	limite G3;
	G3.MinX = 10000.0;
	G3.MaxX = 20000.0;

	G3.MinY = 10000.0;
	G3.MaxY = 20000.0;

	G3.MinZ = 1000.0;
	G3.MaxZ = 4000.0;

	G3.NbrOfUAV = 0;
	G3.Speed = 200.0;

	// Start Experiment //
	// DataSets_Success_Rate = experiment->Run(Burglary, true, onStateChange);
	
	// Start Creating UAVs
	// CreateNodesLrWPAN(NbrUAV)

	// LrWpanHelper helper;
	// helper.EnablePcap(string("/media/tahar/Projects/sim/results/data"), );

	// helper.EnableLogComponents ();

	NS_LOG_INFO("Start : Debug CreateNodesLrWPAN()");
	CreateNodesLrWPAN(NbrUAV);
	NS_LOG_INFO("Finish : Debug CreateNodesLrWPAN()");

	NS_LOG_INFO("Start : Debug AffectationDesAddress_LrWPAN()");
	AffectationDesAddress_LrWPAN(debut, NbrUAV);
	NS_LOG_INFO("Finish : Debug AffectationDesAddress_LrWPAN()");

	NS_LOG_INFO("Start : Debug AffectationDesAddress_GPS_PRN1()");
	AffectationDesAddress_GPS_PRN1(debut, NbrUAV, 0);
	NS_LOG_INFO("Finish : Debug AffectationDesAddress_GPS_PRN1()");	

	InstallRandomMobilityLrWPAN(debut, NbrUAV, G1, G2, G3);

	InitialisationChannelLrwpan();
	InitialisationChannelGPS();

	InstallChannelLrwpan(debut, NbrUAV);
	InstallChannelGPS(debut, NbrUAV);

	InstallDevicesLrwpan(debut, NbrUAV);
	InstallDevicesGPS(debut, NbrUAV);

	int startMalicious = victim;
	int endMalicious = startMalicious + numberOfMalicious;
	int startWitness = endMalicious;
	int endWitness = uavs; // NbrUAV

	// Installing Apps
	InstallApps(debut, startMalicious, UAV_VICTIM); // Good
	// InstallApps(startMalicious, endMalicious, UAV_BAD); // BAD
	InstallApps(startMalicious, endMalicious, UAV_BAD); // BAD
	InstallApps(startWitness, endWitness, UAV_GOOD); // GOOD

	// Always After Installing Applications
	ConnectCallbacks();

	// if ((method == Burglary) || (method == AbsolutePower) | (method == AbsolutePower_CN0))
	if (true)
	{
		// Create N Satellites
		int n = 9;
		CreateNSatellite(n, 57.0, 0.0);
		// CreateNSatellite(n, 57.0, 0.0);
	}

	// *** Create 12 Authentic Satellites *** //
/*
	if (method == OneSatellite)
	{
		// Max C/N0
		uint32_t firstPRN = 4294967295;
		CreateOneSatellite(1, 1, 20100000, firstPRN, 58.6, 0.0);
	}

	if (method == AllSatellite)
	{
		// Min C/N0
		CreateNSatellite(12, 57.0, 0.0);
	}

	if (method == NormalCase)
	{
		// Average C/N0 with 7 Satellites
	
		CreateNSatellite(7, 57.0, 0.0);
	}
*/
	// ***
	// *** Create Sppofing Satellites *** //
	// double power = m_SpoofingSatellitePower;
	// m_SpoofingSatellitePower = 59;
/*
	if (method == AuthenticWithSpoofing)
	{
		for (int32_t i = 0; i < m_numUAV; i++)
		// for (int32_t i = 0; i < m_numDrones; i++)
		{
			m_GPS_device_Table_PRN0[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN1[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN2[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN3[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN4[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN5[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN6[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN7[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN8[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN9[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN10[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN11[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
		}

		// 7 Authentic
		CreateNSatellite(12, 58.6, 0.0);

		// 10 Spoofing Satellites
		double appStartTime = 0.006;
		CreateNSatellite(30, power, appStartTime);
	}

	if (true) // (method == Spoofing10 || method == Spoofing20 || method == Spoofing30)
	{
		for (int32_t i = 0; i < m_numDrones; i++)
		{
			m_GPS_device_Table_PRN0[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN1[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN2[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN3[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN4[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN5[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN6[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN7[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN8[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN9[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN10[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
			m_GPS_device_Table_PRN11[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
		}
		// 7 Authentic
		CreateNSatellite(12, 57.0, 0.06);

		// 10 Spoofing Satellites
		double appStartTime = 0.0;
		CreateNSatellite(24, power, appStartTime);
	}

	if (method == Spoofing20 || method == Spoofing30)
	{
		// Add 10 Spoofing Satellites
		double appStartTime = 0.0;
		CreateNSatellite(10, power, appStartTime);
	}
	if (method == Spoofing30)
	{
		// Add 10 Spoofing Satellites
		double appStartTime = 0.0;
		CreateNSatellite(10, power, appStartTime);
	}
*/
	
	// ***

	// helper.EnablePcapAll(string("/media/tahar/Projects/sim/results/data"));
	// helper.EnablePcap(string("/media/tahar/Projects/sim/results/data"), );
	/*
	AsciiTraceHelper ascii;
	Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr_wpan_data.tr");
	helper.EnableAsciiAll(stream);
	*/

	// Starting Simulation
	AnimationInterface anim("Experiment.xml");
    anim.EnablePacketMetadata(true);

	// Setting NetAnim Node (Colors + Sizes)
	for (int i = debut; i < (NbrUAV+m_numSatellites); i++)
	{
		if (i < startMalicious)
		{
			anim.UpdateNodeSize (i, 1500, 1500);
			anim.UpdateNodeColor (i, 255, 179, 0);
		}
		else if (i < endMalicious)
		{
			anim.UpdateNodeSize (i, 1500, 1500);
			anim.UpdateNodeColor (i, 255, 0, 0);
		}
		else if (i < NbrUAV)
		{
			anim.UpdateNodeSize (i, 1500, 1500);
			anim.UpdateNodeColor (i, 0, 255, 0);
		}
		else if (i < (NbrUAV+m_numSatellites))
		{
			anim.UpdateNodeSize (i, 5000, 5000);
			anim.UpdateNodeColor (i, 0, 0, 255);
		}
	}

	Simulator::Stop(Seconds(50.0)); // 40.0 + 120.0
	Simulator::Run();
	Simulator::Destroy();

	return m_cn0VersusAbsolutPower;
}

void
Experiment::SetDefaults()
{
	NS_LOG_INFO("Start : SetDefaults");

	// Make sure these are reset just in case the same Experiment is run twice
	m_packetsSent = 0;
	m_SpoofingSatellitePower = 50.1;
	m_dataReceived = 0;
	m_dataSent = 0;
	cp_packetsSent = 0;
	cp_packetsReceived = 0;
	cp_dataReceived = 0;
	cp_dataSent = 0;

	m_True_Negative = 0;
	m_True_Positive = 0;
	m_False_Negative = 0;
	m_False_Positive = 0;

	m_phyList = {};
	m_phyStats = {};

	// Create enough space for all drones
	m_phyList.resize(0);
	m_phyList.resize(m_numDrones);
	m_phyStats.resize(0);
	m_phyStats.resize(m_numDrones);

	switch (m_method)
	{
	case AuthenticWithSpoofing:
		m_methodString = "Authentic* + Spoofing (Na=7  Ns=10)";
		break;
	case Spoofing10:
		m_methodString = "Spoofing* + Authentic (Na=7   Ns=10)";
		break;
	case Spoofing20:
		m_methodString = "Spoofing* + Authentic (Na=7   Ns=20)";
		break;
	case Spoofing30:
		m_methodString = "Spoofing* + Authentic (Na=7   Ns=30)";
		break;
	case OneSatellite:
		m_methodString = "Max: authentic* signal (Na=1  Ns=0)";
		break;
	case AllSatellite:
		m_methodString = "Min: authentic* signal (Na=14  Ns=0) ";
		break;

	case NormalCase:
		m_methodString = "Authentic* signal avrg  (Na=7  Ns=0)";
		break;

	case Burglary:
		m_methodString = "Burglary";
		break;
	case AbsolutePower:
		m_methodString = "AbsolutePower";
		break;
	case AbsolutePower_CN0:
		m_methodString = "AbsolutePower_CN0";
		break;
	}

	m_drones = NodeContainer();

	m_cn0VersusAbsolutPower = Gnuplot2dDataset(m_methodString);
	m_cn0VersusAbsolutPower.SetStyle(Gnuplot2dDataset::POINTS);
	m_cn0VersusAbsolutPower.SetExtra("set key font \",20\" ");

	m_success_rate = Gnuplot2dDataset(m_methodString);

	// m_success_rate.SetStyle (Gnuplot2dDataset::POINTS);

	m_NetDevicesTable.clear();
	m_ConstantPositionMobilityTable.clear();
	m_RandomMobilityTable.clear();

	m_GPS_device_Table_PRN0.clear();
	m_GPS_device_Table_PRN1.clear();
	m_GPS_device_Table_PRN2.clear();
	m_GPS_device_Table_PRN3.clear();

	m_GPS_device_Table_PRN4.clear();
	m_GPS_device_Table_PRN5.clear();
	m_GPS_device_Table_PRN6.clear();
	m_GPS_device_Table_PRN7.clear();

	m_GPS_device_Table_PRN8.clear();
	m_GPS_device_Table_PRN9.clear();
	m_GPS_device_Table_PRN10.clear();
	m_GPS_device_Table_PRN11.clear();

	m_NetDevicesSatelliteTable.clear();
	m_satellites = NodeContainer();
	;

	powerCNVector.clear();

	m_numSatellites = 0;
	m_numDrones = 0;
	
	NS_LOG_INFO("Finish : SetDefaults");
}

std::string 
Experiment::UniformRange(uint32_t min, uint32_t max)
{
	NS_LOG_INFO("Start : UniformRange");

	return "ns3::UniformRandomVariable[Min=" + to_string(min) + "|Max=" + to_string(max) + "]";
	NS_LOG_INFO("Finish : UniformRange");
}

std::string
Experiment::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
	NS_LOG_INFO("Start : WalkBounds");

	return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" +
		   std::to_string(maxY);

	NS_LOG_INFO("Finish : WalkBounds");
}

void Experiment::ConnectCallbacks()
{
	NS_LOG_INFO("Start : ConnectCallbacks");

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/SNRPerformance",
		MakeCallback(&Experiment::SNRPerformance, this));

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/TrustPerformance",
		MakeCallback(&Experiment::TrustPerformance, this));

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/DronePacketNumber",
		MakeCallback(&Experiment::DronePacketNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/SatelliteNumber",
		MakeCallback(&Experiment::SatelliteNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/DroneReceivedNumber",
		MakeCallback(&Experiment::DroneReceivedNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/SatelliteReceivedNumber",
		MakeCallback(&Experiment::SatelliteReceivedNumber, this));

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/DronePacketNumber",
		MakeCallback(&Experiment::DronePacketNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/SatelliteNumber",
		MakeCallback(&Experiment::SatelliteNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/DroneReceivedNumber",
		MakeCallback(&Experiment::DroneReceivedNumber, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/SatelliteReceivedNumber",
		MakeCallback(&Experiment::SatelliteReceivedNumber, this));

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/Rx",
		MakeCallback(&Experiment::Cn0VersusTSP, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/RxAlert",
		MakeCallback(&Experiment::Alert, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/RxDetection",
		MakeCallback(&Experiment::Detection, this));

	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/Rx",
	MakeCallback(&Experiment::Cn0VersusTSP, this));

	NS_LOG_INFO("Finish : ConnectCallbacks");
}

// CallBacks
// void Experiment::

void Experiment::SNRPerformance(double index)
{
	if (index == 10)
	{
    	m_True_Positive++;
		satellite_True_Positive++;
    	std::cout << "satellite_True_Positive++;" << std::endl ;
    }

    if (index == 15)
	{
       	m_False_Positive++;
		satellite_False_Positive++;
    	std::cout << "satellite_False_Positive++;" << std::endl ;
    }


    if (index == 20)
	{
       	m_True_Negative++;
		satellite_True_Negative++;
    	std::cout << "satellite_True_Negative++;" << std::endl ;
    }

    if (index == 25)
	{
       	m_False_Negative++;
    	satellite_False_Negative++;
		std::cout << "satellite_False_Negative++;" << std::endl ;
    }
}

void Experiment::TrustPerformance(double index)
{
    std::cout << "Start : TrustPerformance" << std::endl ;
	
	if (index == 10)
	{
    	m_True_Positive++;
		trust_True_Positive++;
    	std::cout << "trust_True_Positive++;" << std::endl ;
    	std::cout << "m_True_Positive : " << m_True_Positive << std::endl ;
    	std::cout << "trust_True_Positive : " << trust_True_Positive << std::endl ;
    }

    if (index == 15)
	{
       	m_False_Positive++;
		trust_False_Positive++;
    	std::cout << "trust_False_Positive++;" << std::endl ;
		std::cout << "m_False_Positive : " << m_False_Positive << std::endl ;
    	std::cout << "trust_False_Positive : " << trust_False_Positive << std::endl ;

    }

    if (index == 20)
	{
       	m_True_Negative++;
		trust_True_Negative++;
    	std::cout << "trust_True_Negative++;" << std::endl ;
		std::cout << "m_True_Negative : " << m_True_Negative << std::endl ;
    	std::cout << "trust_True_Negative : " << trust_True_Negative << std::endl ;

    }

    if (index == 25)
	{
       	m_False_Negative++;
		trust_False_Negative++;
    	std::cout << "trust_False_Negative++;" << std::endl ;
		std::cout << "m_False_Negative : " << m_False_Negative << std::endl ;
		std::cout << "trust_False_Negative : " << trust_False_Negative << std::endl ;
 
	}
}

void Experiment::DronePacketNumber(double index)
{
	totalNumberOfPackets++;
	totalDronePackets++;
	cout << "Total sent packet number : " << totalNumberOfPackets << endl;
	cout << "Total Drone sent packet number : " << totalDronePackets << endl;
}

void Experiment::SatelliteNumber(double index)
{
	totalNumberOfPackets++;
	totalSatellitePackets++;
	cout << "Total sent packet number : " << totalNumberOfPackets << endl;
	cout << "Total Satellite sent packet number : " << totalSatellitePackets << endl;

}

void Experiment::DroneReceivedNumber(double index)
{
	totalReceivedPackets++;
	droneReceivedPackets++;
	cout << "Total received packet number : " << totalReceivedPackets << endl;
	cout << "Total drone received packet number : " << droneReceivedPackets << endl;

}

void Experiment::SatelliteReceivedNumber(double index)
{
	satelliteReceivedPackets++;
	totalReceivedPackets++;
	cout << "Total received packet number : " << totalReceivedPackets << endl;
	cout << "Total satellite received packet number : " << satelliteReceivedPackets << endl;

}


// Cn0VersusTSP
void Experiment::Cn0VersusTSP(double receivingSatellitepower, double CN0)
{
	NS_LOG_INFO("Start : Cn0VersusTSP");
	m_SpoofingSatellitePower += 1;

    double txSpoofingPower = m_SpoofingSatellitePower-30 ;
    double rxSpoofingPower= txSpoofingPower - 182.4 -  2.0;

	if (receivingSatellitepower > maxPower)
		maxPower = receivingSatellitepower;
	if (receivingSatellitepower < minPower)
		minPower = receivingSatellitepower;

    std::cout<<"			    < satellite power== "<< receivingSatellitepower<<"      >> \n";
    std::cout<<"			    < c/n0== "<< CN0<<"      >> \n";
    std::cout<<"			    < rxSpoofingPower== "<< rxSpoofingPower<<"      >> \n";

	powerCN data;
	data.cn = CN0;
	data.power = receivingSatellitepower;

	cout << "maxPower" << maxPower << endl;
	cout << "minPower" << minPower << endl;

	powerCNVector.push_back(data);

	m_cn0VersusAbsolutPower.Add(receivingSatellitepower, CN0);

	NS_LOG_INFO("Finish : Cn0VersusTSP");
}
// Alert
void Experiment::Alert(double receivingSatellitepower, double CN0, double index)
{
	NS_LOG_INFO("Start : Alert");
/*
	if (index ==10 ){
    m_True_Positive++;
    std::cout<<"	    	m_True_Positive++;     >> \n";
    }

    if (index ==15 ){
       	m_False_Positive++;
    	std::cout<<"	    	m_False_Positive++;     >> \n";
       }


    if (index ==20 ){
       	m_True_Negative++;
    	std::cout<<"	    		m_True_Negative++;     >> \n";
       }


    if (index ==25 ){
       	m_False_Negative++;
    	std::cout<<"	    			m_False_Negative++;    >> \n";
       }
*/
	NS_LOG_INFO("Finish : Alert");
}
// Detection
void Experiment::Detection(double receivingSatellitepower, double CN0, double index)
{
	NS_LOG_INFO("Start : Detection");
	NS_LOG_INFO("Finish : Detection");
}

Ptr<LrWpanNetDevice> Experiment::GetDevice(uint16_t i)
{
	
	
	if (i > 60928)
		i = i - 60928;

	if (i > (m_numUAV + m_numVictim))
		i = i - m_numUAV;

	return m_NetDevicesTable[i];
}

void Experiment::CreateNodesLrWPAN(int NbrUAV)
{
	NS_LOG_INFO("Start : Create LrWPAN Nodes");

	RngSeedManager::SetSeed(4);
	RngSeedManager::SetRun(6);

	m_numDrones = NbrUAV;
	m_drones.Create(NbrUAV);

	NS_LOG_INFO("Finish : Create LrWPAN Nodes");
}

void Experiment::AffectationDesAddress_LrWPAN(uint32_t debut, uint32_t fin)
{
	NS_LOG_INFO("Start : Assigning LrWPAN Addresses");

	// Creating an address model
	char address[5] = {'0', '0', ':', '0', '0', };

	// Creating Network Card for each Drone
	// MAC Address (16 bits) 00:00 - FF:FF
	// Stocking MAC16 Address in Pointer Vector
	Ptr<LrWpanNetDevice> dev_i; // = CreateObject<LrWpanNetDevice>();
	
	// LrWpanHelper helper;

	for (uint16_t i = debut; i < fin; i++)
	{
		// cout << "i : " << i << endl;
		stringstream str;
		str << (i);

		// LrWpanNetDevice tempDevice = CreateObject<LrWpanNetDevice>();
		dev_i = CreateObject<LrWpanNetDevice>();
		// dev_i->GetPhy()->SetRxSensitivity(-94);
		dev_i->GetPhy()->SetRxSensitivity(-87);
		// cout << "after create dev_i : " << dev_i << endl;

		if (i < 10)
		{
			address[4] = str.str()[0];
		}
		else
		{
			address[4] = str.str()[1];
			address[3] = str.str()[0];
		}
		/*
		dev_i->SetAddress(
			// Assign MAC Address to Network Device i
			Mac16Address(address));
		*/

		// dev_i->SetAddress(
		// 		Mac16Address(i));
		// 	m_NetDevicesTable.push_back(dev_i);
		// This One is Better
		if ((i >= m_numVictim) && (i < (m_numVictim+numberOfMalicious)))
		{
			dev_i->SetAddress(
				Mac16Address(60928+i));	
		}
		else
		{
		// Assign MAC Address to Network Device i
			dev_i->SetAddress(
				Mac16Address(i));
		}

		m_NetDevicesTable.push_back(dev_i);
		/*
		cout << "i : " << i << endl;
		cout << "address: " << address << endl;
		cout << "Network Card MAC : " << dev_i->GetAddress() << endl;
		*/
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/lr_wpan_"+to_string(i)), dev_i, true, true);
	}

	NS_LOG_INFO("Finish : Assigning LrWPAN Addresses");
}

void Experiment::AffectationDesAddress_GPS_PRN1(uint32_t debut, uint32_t fin, uint32_t PRN_index)
{
	NS_LOG_INFO("Start : Assigning GPS PRN1 Addresses");
	
	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	// Creating an address model for GPS Addresses
	// char address[5] = {'7', '7', ':', '7', '7', };
	char address[5] = {'7', '7', ':', '7', '7', };
	int32_t macAddress = 30583;

	// Creating Network Card for each PRN
	// MAC Address (16 bits) 00:00 - FF:FF
	// Stocking MAC16 Address in Pointer Vector
	
	Ptr<LrWpanNetDevice> GPSdev_PRN0;
	Ptr<LrWpanNetDevice> GPSdev_PRN1;
	Ptr<LrWpanNetDevice> GPSdev_PRN2;
	Ptr<LrWpanNetDevice> GPSdev_PRN3;

	Ptr<LrWpanNetDevice> GPSdev_PRN4;
	Ptr<LrWpanNetDevice> GPSdev_PRN5;
	Ptr<LrWpanNetDevice> GPSdev_PRN6;
	Ptr<LrWpanNetDevice> GPSdev_PRN7;

	Ptr<LrWpanNetDevice> GPSdev_PRN8;
	Ptr<LrWpanNetDevice> GPSdev_PRN9;
	Ptr<LrWpanNetDevice> GPSdev_PRN10;
	Ptr<LrWpanNetDevice> GPSdev_PRN11;

	LrWpanHelper helper;

	for (uint32_t i = debut; i < fin; i++)
	{
		stringstream str;
		str << (i);

		GPSdev_PRN0 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN1 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN2 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN3 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN4 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN5 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN6 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN7 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN8 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN9 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN10 = CreateObject<LrWpanNetDevice>();
		GPSdev_PRN11 = CreateObject<LrWpanNetDevice>();

		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN0"), GPSdev_PRN0, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN1"), GPSdev_PRN1, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN2"), GPSdev_PRN2, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN3"), GPSdev_PRN3, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN4"), GPSdev_PRN4, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN5"), GPSdev_PRN5, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN6"), GPSdev_PRN6, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN7"), GPSdev_PRN7, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN8"), GPSdev_PRN8, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN9"), GPSdev_PRN9, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN10"), GPSdev_PRN10, true, true);
		// helper.EnablePcap(string("/media/tahar/Projects/sim/results/DEV_"+to_string(i)+"_PRN11"), GPSdev_PRN11, true, true);

		GPSdev_PRN0->SetAddress(
			// Assign MAC Address to GPS Network Device 0
			Mac16Address(macAddress));

		GPSdev_PRN1->SetAddress(
			// Assign MAC Address to GPS Network Device 1
			Mac16Address(macAddress));

		GPSdev_PRN2->SetAddress(
			// Assign MAC Address to GPS Network Device 2
			Mac16Address(macAddress));

		GPSdev_PRN3->SetAddress(
			// Assign MAC Address to GPS Network Device 3
			Mac16Address(macAddress));

		GPSdev_PRN4->SetAddress(
			// Assign MAC Address to GPS Network Device 4
			Mac16Address(macAddress));
		
		GPSdev_PRN5->SetAddress(
			// Assign MAC Address to GPS Network Device 5
			Mac16Address(macAddress));

		GPSdev_PRN6->SetAddress(
			// Assign MAC Address to GPS Network Device 6
			Mac16Address(macAddress));
		
		GPSdev_PRN7->SetAddress(
			// Assign MAC Address to GPS Network Device 7
			Mac16Address(macAddress));
		
		GPSdev_PRN8->SetAddress(
			// Assign MAC Address to GPS Network Device 8
			Mac16Address(macAddress));
		
		GPSdev_PRN9->SetAddress(
			// Assign MAC Address to GPS Network Device 9
			Mac16Address(macAddress));
		
		GPSdev_PRN10->SetAddress(
			// Assign MAC Address to GPS Network Device 10
			Mac16Address(macAddress));
		
		GPSdev_PRN11->SetAddress(
			// Assign MAC Address to GPS Network Device 11
			Mac16Address(macAddress));

/*
		GPSdev_PRN0->SetAddress(
			// Assign MAC Address to GPS Network Device 0
			Mac16Address(address));
		GPSdev_PRN1->SetAddress(
			// Assign MAC Address to GPS Network Device 1
			Mac16Address(address));
		GPSdev_PRN2->SetAddress(
			// Assign MAC Address to GPS Network Device 2
			Mac16Address(address));
		GPSdev_PRN3->SetAddress(
			// Assign MAC Address to GPS Network Device 3
			Mac16Address(address));

		GPSdev_PRN4->SetAddress(
			// Assign MAC Address to GPS Network Device 4
			Mac16Address(address));
		GPSdev_PRN5->SetAddress(
			// Assign MAC Address to GPS Network Device 5
			Mac16Address(address));
		GPSdev_PRN6->SetAddress(
			// Assign MAC Address to GPS Network Device 6
			Mac16Address(address));
		GPSdev_PRN7->SetAddress(
			// Assign MAC Address to GPS Network Device 7
			Mac16Address(address));
		
		GPSdev_PRN8->SetAddress(
			// Assign MAC Address to GPS Network Device 8
			Mac16Address(address));
		GPSdev_PRN9->SetAddress(
			// Assign MAC Address to GPS Network Device 9
			Mac16Address(address));
		GPSdev_PRN10->SetAddress(
			// Assign MAC Address to GPS Network Device 10
			Mac16Address(address));
		GPSdev_PRN11->SetAddress(
			// Assign MAC Address to GPS Network Device 11
			Mac16Address(address));
*/

		GPSdev_PRN0->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN1->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN2->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN3->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN4->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN5->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN6->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN7->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN8->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN9->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN10->GetPhy()->m_isSatellite = 1;
		GPSdev_PRN11->GetPhy()->m_isSatellite = 1;

		GPSdev_PRN0->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN1->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN2->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN3->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN4->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN5->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN6->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN7->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN8->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN9->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN11->GetPhy()->m_GPSstate = 0;
		GPSdev_PRN10->GetPhy()->m_GPSstate = 0;

		GPSdev_PRN0->GetPhy()->m_PrnChannel = authenticPRN[0];
		GPSdev_PRN1->GetPhy()->m_PrnChannel = authenticPRN[1];
		GPSdev_PRN2->GetPhy()->m_PrnChannel = authenticPRN[2];
		GPSdev_PRN3->GetPhy()->m_PrnChannel = authenticPRN[3];
		GPSdev_PRN4->GetPhy()->m_PrnChannel = authenticPRN[4];
		GPSdev_PRN5->GetPhy()->m_PrnChannel = authenticPRN[5];
		GPSdev_PRN6->GetPhy()->m_PrnChannel = authenticPRN[6];
		GPSdev_PRN7->GetPhy()->m_PrnChannel = authenticPRN[7];
		GPSdev_PRN8->GetPhy()->m_PrnChannel = authenticPRN[8];
		GPSdev_PRN9->GetPhy()->m_PrnChannel = authenticPRN[9];
		GPSdev_PRN10->GetPhy()->m_PrnChannel = authenticPRN[10];
		GPSdev_PRN11->GetPhy()->m_PrnChannel = authenticPRN[11];

		GPSdev_PRN0->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN1->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN2->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN3->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN4->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN5->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN6->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN7->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN8->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN9->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN10->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
		GPSdev_PRN11->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);

		m_GPS_device_Table_PRN0.push_back(GPSdev_PRN0);
		m_GPS_device_Table_PRN1.push_back(GPSdev_PRN1);
		m_GPS_device_Table_PRN2.push_back(GPSdev_PRN2);
		m_GPS_device_Table_PRN3.push_back(GPSdev_PRN3);
		m_GPS_device_Table_PRN4.push_back(GPSdev_PRN4);
		m_GPS_device_Table_PRN5.push_back(GPSdev_PRN5);
		m_GPS_device_Table_PRN6.push_back(GPSdev_PRN6);
		m_GPS_device_Table_PRN7.push_back(GPSdev_PRN7);
		m_GPS_device_Table_PRN8.push_back(GPSdev_PRN8);
		m_GPS_device_Table_PRN9.push_back(GPSdev_PRN9);
		m_GPS_device_Table_PRN10.push_back(GPSdev_PRN10);
		m_GPS_device_Table_PRN11.push_back(GPSdev_PRN11);

	}

	NS_LOG_INFO("Finish : Assigning GPS PRN1 Addresses");
}

void Experiment::InstallRandomMobilityLrWPAN(uint32_t debut, uint32_t fin, limite LimiteG1, limite LimiteG2, limite LimiteG3)
{
	NS_LOG_INFO("Start : InstallRandomMobilityLrWPAN");

	// Group 1 Mobility for NetAnim
	/*
	Ptr<RandomWalk2dMobilityModel> mobilityModel = CreateObject<RandomWalk2dMobilityModel>();
	mobilityModel->SetAttribute("Mode", StringValue("Time"));
	mobilityModel->SetAttribute("Time", StringValue("0.5s"));
	mobilityModel->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG1.Speed) +"]")); // 5.0]"));
	// mobilityModel->SetAttribute("Bounds", RectangleValue(Rectangle(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
	mobilityModel->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
	*/

/*
	Ptr<RandomWalk2dMobilityModel> mobilityModel;

	{
		mobilityModel = CreateObject<RandomWalk2dMobilityModel>();
		mobilityModel->SetAttribute("Mode", StringValue("Time"));
		mobilityModel->SetAttribute("Time", StringValue("0.5s"));
		mobilityModel->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG1.Speed) +"]")); // 5.0]"));
		// mobilityModel->SetAttribute("Bounds", RectangleValue(Rectangle(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
		mobilityModel->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
	}
*/

	Ptr<RandomBoxPositionAllocator> positionAllocator = CreateObject<RandomBoxPositionAllocator>();
	Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
	x->SetAttribute ("Min", DoubleValue (LimiteG1.MinX));
	x->SetAttribute ("Max", DoubleValue (LimiteG1.MaxX));
	Ptr<UniformRandomVariable> y = CreateObject<UniformRandomVariable> ();
	y->SetAttribute ("Min", DoubleValue (LimiteG1.MinY));
	y->SetAttribute ("Max", DoubleValue (LimiteG1.MaxY));
	Ptr<UniformRandomVariable> z = CreateObject<UniformRandomVariable> ();
	z->SetAttribute ("Min", DoubleValue (LimiteG1.MinZ));
	z->SetAttribute ("Max", DoubleValue (LimiteG1.MaxZ));

	positionAllocator->SetX(x);
	positionAllocator->SetY(y);
	positionAllocator->SetZ(z);

	// "ns3::RandomBoxPositionAllocator",
	// "X", StringValue("ns3::UniformRandomVariable[Min=1|Max=50000]"),
	// "Y", StringValue("ns3::UniformRandomVariable[Min=1|Max=100000]"),
	// "Z", StringValue("ns3::UniformRandomVariable[Min=1000|Max=10000]")

	// For NetAnim
	MobilityHelper mobility;
	mobility.SetMobilityModel(
		"ns3::RandomWalk2dMobilityModel",
		"Mode",
		StringValue("Time"),
		"Time",
		StringValue("0.5s"),
		"Speed",
		StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG1.Speed) + "]"),
		"Bounds",
		// RectangleValue(Rectangle(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY))
		StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY))
	);
	mobility.SetPositionAllocator(positionAllocator);

	/*
	mobility.SetPositionAllocator(
		"ns3::RandomBoxPositionAllocator",
		"X", StringValue(UniformRange(LimiteG1.MinX, LimiteG1.MaxX)),
		"Y", StringValue(UniformRange(LimiteG1.MinY, LimiteG1.MaxY)),
		"Z", StringValue(UniformRange(LimiteG1.MinZ, LimiteG1.MaxZ))
	);
	*/

/*
	mobility.SetPositionAllocator(
		"ns3::RandomBoxPositionAllocator",
		"X", StringValue("ns3::UniformRandomVariable[Min=1|Max=50000]"),
		"Y", StringValue("ns3::UniformRandomVariable[Min=1|Max=100000]"),
		"Z", StringValue("ns3::UniformRandomVariable[Min=1000|Max=2000]")
	);
*/
	
	Ptr<RandomWalk2dMobilityModel> animMob;
	Ptr<RandomWalk2dMobilityModel> lrMob;
	
	for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
	{

		Vector vector = positionAllocator->GetNext();
		// mobilityModel->SetPosition(vector);

		/*
		cout << "i : " << i << endl;
		cout << "Vector : " << vector << endl;
		cout << "My NetDevice Position : " << m_NetDevicesTable[i]->GetPhy()->GetMobility()->GetPosition() << endl;
		*/

		mobility.Install(m_drones.Get(i));
		animMob = m_drones.Get(i)->GetObject<RandomWalk2dMobilityModel>();
		animMob->SetPosition(vector);

		lrMob = CreateObject<RandomWalk2dMobilityModel>();
		lrMob->SetAttribute("Mode", StringValue("Time"));
		lrMob->SetAttribute("Time", StringValue("0.5s"));
		lrMob->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
		lrMob->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));

		lrMob->SetPosition(vector);

		m_NetDevicesTable[i]->GetPhy()->SetMobility(lrMob);
		m_RandomMobilityTable.push_back(lrMob);

		cout << "i : " << i << endl;
		cout << "Address : " << m_NetDevicesTable[i]->GetAddress() << endl;
		cout << "NetAnim : Position : " << m_drones.Get(i)->GetObject<RandomWalk2dMobilityModel>()->GetPosition() << endl;
		cout << "LRWPAN : Position : " << m_NetDevicesTable[i]->GetPhy()->GetMobility()->GetPosition() << endl;
	}

/*
	MobilityHelper mobility_01;
	mobility_01.SetPositionAllocator(
		"ns3::RandomBoxPositionAllocator",
		"X", StringValue("ns3::UniformRandomVariable[Min=1|Max=50000]"),
		"Y", StringValue("ns3::UniformRandomVariable[Min=1|Max=100000]"),
		"Z", StringValue("ns3::UniformRandomVariable[Min=1000|Max=10000]")
	);

	mobility_01.SetMobilityModel(
		"ns3::RandomWalk2dMobilityModel",
		"Mode",
		StringValue("Time"),
		"Time",
		StringValue("5s"),
		"Speed",
		StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG1.Speed) + "]"),
		"Bounds",
		StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY))
	);
	
	for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
	{
		mobility_01.Install(m_drones.Get(i));
	};
/*
	// Group 2 Mobility for NetAnim
	MobilityHelper mobility_02;
	mobility_02.SetPositionAllocator(
		"ns3::RandomBoxPositionAllocator",
		"X", StringValue(UniformRange(LimiteG2.MinX, LimiteG2.MaxX)),
		"Y", StringValue(UniformRange(LimiteG2.MinY, LimiteG2.MaxY)),
		"Z", StringValue(UniformRange(LimiteG2.MinZ, LimiteG2.MaxZ))
	);

	mobility_02.SetMobilityModel(
		"ns3::RandomWalk2dMobilityModel",
		"Mode",
		StringValue("Time"),
		"Time",
		StringValue("0.5s"),
		"Speed",
		StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG2.Speed) + "]"),
		"Bounds",
		StringValue(WalkBounds(LimiteG2.MinX, LimiteG2.MaxX, LimiteG2.MinY, LimiteG2.MaxY))
	);
	
	for (uint32_t i = (debut + LimiteG1.NbrOfUAV); i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV); i++)
	{
		mobility_02.Install(m_drones.Get(i));
	};

	// Group 3 Mobility for NetAnim

	MobilityHelper mobility_03;
	mobility_03.SetPositionAllocator(
		"ns3::RandomBoxPositionAllocator",
		"X", StringValue(UniformRange(LimiteG3.MinX, LimiteG3.MaxX)),
		"Y", StringValue(UniformRange(LimiteG3.MinY, LimiteG3.MaxY)),
		"Z", StringValue(UniformRange(LimiteG3.MinZ, LimiteG3.MaxZ))
	);

	mobility_03.SetMobilityModel(
		"ns3::RandomWalk2dMobilityModel",
		"Mode",
		StringValue("Time"),
		"Time",
		StringValue("0.5s"),
		"Speed",
		StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG3.Speed) + "]"),
		"Bounds",
		StringValue(WalkBounds(LimiteG3.MinX, LimiteG3.MaxX, LimiteG3.MinY, LimiteG3.MaxY))
	);
	for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV); i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV); i++)
	{
		mobility_03.Install(m_drones.Get(i));
	}
*/

	// Setting LrWPAN Mobility
/*
	// Group 1 : Install Mobility Model in the devices Drones[0 .. N1]
	Ptr<RandomWalk2dMobilityModel> mobility01 = CreateObject<RandomWalk2dMobilityModel>();
	mobility01->SetAttribute("Mode", StringValue("Time"));
	mobility01->SetAttribute("Time", StringValue("0.5s"));
	mobility01->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
	mobility01->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));

	for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
	{
		m_NetDevicesTable[i]->GetPhy()->SetMobility(mobility01);
		m_RandomMobilityTable.push_back(mobility01);
	}

	// Group 2 : Install Mobility Model in the devices Drones[N1+1 .. N2]
	Ptr<RandomWalk2dMobilityModel> mobility02 = CreateObject<RandomWalk2dMobilityModel>();
	mobility02->SetAttribute("Mode", StringValue("Time"));
	mobility02->SetAttribute("Time", StringValue("0.5s"));
	mobility02->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
	mobility02->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG2.MinX, LimiteG2.MaxX, LimiteG2.MinY, LimiteG2.MaxY)));

	for (uint32_t i = (debut + LimiteG1.NbrOfUAV); i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV); i++)
	{
		m_NetDevicesTable[i]->GetPhy()->SetMobility(mobility02);
		m_RandomMobilityTable.push_back(mobility02);
	}

	// Group 3 : Install Mobility Model in the devices Drones[N2+1 .. N3]
	Ptr<RandomWalk2dMobilityModel> mobility03 = CreateObject<RandomWalk2dMobilityModel>();
	mobility03->SetAttribute("Mode", StringValue("Time"));
	mobility03->SetAttribute("Time", StringValue("0.5s"));
	mobility03->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
	mobility03->SetAttribute("Bounds", StringValue(WalkBounds(LimiteG3.MinX, LimiteG3.MaxX, LimiteG3.MinY, LimiteG3.MaxY)));

	for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV); i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV); i++)
	{
		m_NetDevicesTable[i]->GetPhy()->SetMobility(mobility03);
		m_RandomMobilityTable.push_back(mobility03);
	}
*/

	NS_LOG_INFO("Finish : InstallRandomMobilityLrWPAN");
}

void Experiment::InitialisationChannelLrwpan()
{
	NS_LOG_INFO("Start : InitialisationChannelLrwpan");

	channel = CreateObject<MultiModelSpectrumChannel>();
	model = CreateObject<ItuR1411LosPropagationLossModel>();
	delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	channel->AddPropagationLossModel(model);

	// Unpredictable results
	// channel->SetPropagationDelayModel (delayModel);

	NS_LOG_INFO("Finish : InitialisationChannelLrwpan");
}

void Experiment::InitialisationChannelGPS()
{
	NS_LOG_INFO("Start : InitialisationChannelGPS");

	GPSchannel = CreateObject<MultiModelSpectrumChannel>();
	GPSmodel = CreateObject<FriisPropagationLossModel>();
	GPSmodel->SetFrequency(1575420000);
	
	GPSdelayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	GPSdelayModel->SetSpeed(299792458.0);

	GPSchannel->AddPropagationLossModel(GPSmodel);
	GPSchannel->SetPropagationDelayModel(GPSdelayModel);

	NS_LOG_INFO("Finish : InitialisationChannelGPS");
}

void Experiment::InstallChannelLrwpan(uint32_t debut, uint32_t fin)
{
	NS_LOG_INFO("Start : InstallChannelLrwpan");
	
	LrWpanSpectrumValueHelper svh;

	Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(2.0, channelNumber);

	// Assigning Power and Channel
	for (uint32_t i = debut; i < fin; i++)
	{
		m_NetDevicesTable[i]->SetChannel(channel);
		m_NetDevicesTable[i]->GetPhy()->SetTxPowerSpectralDensity(psd);
	}
	
	NS_LOG_INFO("Finish : InstallChannelLrwpan");
}

void Experiment::InstallChannelGPS(uint32_t debut, uint32_t fin)
{
	NS_LOG_INFO("Start : InstallChannelGPS");
	LrWpanSpectrumValueHelper svh;
	Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(57.0, GPSchannelNumber); // 57.0 dbmW

	// Assigning Power and Channel
	for (uint32_t i = debut; i < fin; i++)
	{
		m_GPS_device_Table_PRN0[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN0[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN1[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN1[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN2[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN2[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN3[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN3[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN4[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN4[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN5[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN5[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN6[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN6[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN7[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN7[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN8[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN8[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN9[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN9[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN10[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN10[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

		m_GPS_device_Table_PRN11[i]->SetChannel(GPSchannel);
		m_GPS_device_Table_PRN11[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

	}

	NS_LOG_INFO("Finish : InstallChannelGPS");
}

void Experiment::InstallDevicesLrwpan(uint32_t debut, uint32_t fin)
{
	NS_LOG_INFO("Start : InstallDevicesLrwpan");

	// Adding LrWPAN device to nodes
	// To complete configuration

	for (uint32_t i = debut; i < fin; i++)
	{
		m_drones.Get(i)->AddLrWpanNetDevice(m_NetDevicesTable[i]);
		m_drones.Get(i)->AddDevice(m_NetDevicesTable[i]);
	
		for (uint32_t j = debut; j < fin; j++)
		{
			if (i != j)
				m_drones.Get(i)->AddWpanNetDeviceNeighbors(m_NetDevicesTable[j]);
		}
	}
	NS_LOG_INFO("Finish : InstallDevicesLrwpan");

}

void Experiment::InstallDevicesGPS(uint32_t debut, uint32_t fin)
{
	NS_LOG_INFO("Start : InstallDevicesGPS");

	// Adding GPS device to nodes
	// To complete configuration

	for (uint32_t i = debut; i < fin; i++)
	{
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN0[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN0[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN1[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN1[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN2[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN2[i]);

		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN3[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN3[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN4[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN4[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN5[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN5[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN6[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN6[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN7[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN7[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN8[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN8[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN9[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN9[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN10[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN10[i]);
		
		m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN11[i]);
		m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN11[i]);

		// Ptr<LrWpanNetDevice> MoiDevReceiver = CreateObject<LrWpanNetDevice>();
		Ptr<LrWpanNetDevice> MoiDevReceiver = m_NetDevicesTable[i];

		Mac16Address address = m_NetDevicesTable[i]->GetMac()->GetShortAddress();
		uint8_t* myAddress = new uint8_t[2];
		address.CopyTo(myAddress);
		
		/*
		cout << "i : " << i << endl;
		cout << "Address : " << address << endl;
		cout << "MyAddress : " << myAddress << endl;
		*/
		
		m_GPS_device_Table_PRN0[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN1[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN2[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN3[i]->GetPhy()->m_attachedAdresse = myAddress;

		m_GPS_device_Table_PRN4[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN5[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN6[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN7[i]->GetPhy()->m_attachedAdresse = myAddress;

		m_GPS_device_Table_PRN8[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN9[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN10[i]->GetPhy()->m_attachedAdresse = myAddress;
		m_GPS_device_Table_PRN11[i]->GetPhy()->m_attachedAdresse = myAddress;


		// LrWpanHelper::EnablePcapInternal(("GPS_Net_" + to_string(i) + "_Dev"),m_GPS_device_Table_PRN0[i], true, false);
		// PcapHelperForDevice helper;
		// Ptr<PcapHelperForDevice> helper = CreateObject<PcapHelperForDevice>();
		// helper->EnablePcap(("GPS_Net_" + to_string(i) + "_Dev"), m_GPS_device_Table_PRN11[i], true, false);	
		// m_GPS_device_Table_PRN0[i]->Enable
	}

	NS_LOG_INFO("Finish : InstallDevicesGPS");
}

void Experiment::InstallApps(uint32_t debut, uint32_t fin, uint32_t type)
{
	NS_LOG_INFO("Start : InstallApps");

	cout << "m_DataUavFrequency= : " << m_DataUavFrequency << endl;

	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	if (type == UAV_VICTIM)
	{

		for (uint32_t i = debut; i < fin; i++)
		{
			NormalDroneHelper normalDroneHelper;
			
			normalDroneHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
			normalDroneHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
			normalDroneHelper.SetAttribute("Victim", UintegerValue(1));

			normalDroneHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i]));
			normalDroneHelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy()));
			normalDroneHelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac()));
			
			normalDroneHelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i]));
			normalDroneHelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i]));
			normalDroneHelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i]));
			normalDroneHelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i]));

			normalDroneHelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i]));
			normalDroneHelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i]));
			normalDroneHelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i]));
			normalDroneHelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i]));

			normalDroneHelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i]));
			normalDroneHelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i]));
			normalDroneHelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i]));
			normalDroneHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));
			
			normalDroneHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));

			normalDroneHelper.SetAttribute("GetDevice", (CallbackValue)MakeCallback(&Experiment::GetDevice, this));
			
			m_DroneApps = normalDroneHelper.Install(m_drones.Get(i));
		}
	}
	else if (type == UAV_GOOD)
	{
		NormalDroneHelper normalDroneHelper;
		for (uint32_t i = debut; i < fin; i++)
		{
			normalDroneHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
			normalDroneHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
			normalDroneHelper.SetAttribute("Victim", UintegerValue(2));

			normalDroneHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i]));
			normalDroneHelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy()));
			normalDroneHelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac()));

			normalDroneHelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i]));
			normalDroneHelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i]));
			normalDroneHelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i]));
			normalDroneHelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i]));

			normalDroneHelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i]));
			normalDroneHelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i]));
			normalDroneHelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i]));
			normalDroneHelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i]));

			normalDroneHelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i]));
			normalDroneHelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i]));
			normalDroneHelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i]));
			normalDroneHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));

			normalDroneHelper.SetAttribute("GetDevice", (CallbackValue)MakeCallback(&Experiment::GetDevice, this));

			if (m_method == AbsolutePower)
				normalDroneHelper.SetAttribute("Methode", UintegerValue(1));
			else if (m_method == AbsolutePower_CN0)
				normalDroneHelper.SetAttribute("Methode", UintegerValue(2));
			else if (m_method == Burglary)
				normalDroneHelper.SetAttribute("Methode", UintegerValue(3));

			m_DroneApps = normalDroneHelper.Install(m_drones.Get(i));
		}
	}
	else if (type == UAV_BAD)
	{
		for (uint32_t i = debut; i < fin; i++)
		{
			MaliciousDroneHelper maliciousDroneHelper;

			maliciousDroneHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency))); 
			maliciousDroneHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize)); 

			maliciousDroneHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i])); 
			maliciousDroneHelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy())); 
			maliciousDroneHelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac())); 

			maliciousDroneHelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i])); 

			maliciousDroneHelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i])); 

			maliciousDroneHelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i])); 
			maliciousDroneHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i])); 

			maliciousDroneHelper.SetAttribute("MyPRN", UintegerValue(authenticPRN[i % 12])); 
		
			maliciousDroneHelper.SetAttribute("number_victim", UintegerValue(m_numVictim));
			maliciousDroneHelper.SetAttribute("number_uav", UintegerValue(m_numUAV));

			maliciousDroneHelper.SetAttribute("GetDevice", (CallbackValue)MakeCallback(&Experiment::GetDevice, this));

			m_GPS_device_Table_PRN0[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN1[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN2[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN3[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN4[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN5[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN6[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN7[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN8[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN9[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN10[i]->GetPhy()->m_GPSstate = 0;
			m_GPS_device_Table_PRN11[i]->GetPhy()->m_GPSstate = 0;

			m_DroneApps = maliciousDroneHelper.Install(m_drones.Get(i));
		}
	}

	NS_LOG_INFO("Finish : InstallApps");
}

void Experiment::CreateNSatellite(int NbrSatellite, double TxPower, double StartTime)
{
	NS_LOG_INFO("Start : CreateNSatellite");

	// Authentic PRNs
	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	// 32 Satellite Positions
/*
	Vector SatellitePosition[] = {
		Vector(2, 2, 20100000),         Vector(1, 10000, 20100000),
		Vector(10000, 10000, 20100000), Vector(10000, 1, 20100000),
		Vector(1, 20000, 20100000),     Vector(10000, 20000, 20100000),
		Vector(20000, 20000, 20100000), Vector(20000, 10000, 20100000),
		Vector(20000, 0, 20100000),     Vector(1, 30000, 20100000),
		Vector(10000, 30000, 20100000), Vector(20000, 30000, 20100000),
		Vector(30000, 30000, 20100000), Vector(30000, 20000, 20100000),
		Vector(30000, 10000, 20100000), Vector(30000, 1, 20100000),
		Vector(1, 40000, 20100000),     Vector(10000, 40000, 20100000),
		Vector(20000, 40000, 20100000), Vector(30000, 40000, 20100000),
		Vector(40000, 40000, 20100000), Vector(40000, 30000, 20100000),
		Vector(40000, 20000, 20100000), Vector(40000, 10000, 20100000),
		Vector(40000, 1, 20100000),     Vector(1, 50000, 20100000),
		Vector(10000, 50000, 20100000), Vector(20000, 50000, 20100000),
		Vector(30000, 50000, 20100000), Vector(40000, 50000, 20100000),
		Vector(50000, 50000, 20100000), Vector(50000, 40000, 20100000)
	};
*/
		Vector SatellitePosition[] = {
		Vector(2, 2, 20100000),         Vector(1, 15000, 20100000),
		Vector(15000, 15000, 20100000), Vector(15000, 1, 20100000),
		Vector(1, 30000, 20100000),     Vector(15000, 30000, 20100000),
		Vector(30000, 30000, 20100000), Vector(30000, 15000, 20100000),
		Vector(30000, 0, 20100000),     Vector(1, 30000, 20100000),
		Vector(10000, 30000, 20100000), Vector(20000, 30000, 20100000),
		Vector(30000, 30000, 20100000), Vector(30000, 20000, 20100000),
		Vector(30000, 10000, 20100000), Vector(30000, 1, 20100000),
		Vector(1, 40000, 20100000),     Vector(10000, 40000, 20100000),
		Vector(20000, 40000, 20100000), Vector(30000, 40000, 20100000),
		Vector(40000, 40000, 20100000), Vector(40000, 30000, 20100000),
		Vector(40000, 20000, 20100000), Vector(40000, 10000, 20100000),
		Vector(40000, 1, 20100000),     Vector(1, 50000, 20100000),
		Vector(10000, 50000, 20100000), Vector(20000, 50000, 20100000),
		Vector(30000, 50000, 20100000), Vector(40000, 50000, 20100000),
		Vector(50000, 50000, 20100000), Vector(50000, 40000, 20100000)
	};

	for (int i = 0; i < NbrSatellite; i++)
	{
		CreateOneSatellite(
			SatellitePosition[i % 32].x+(m_numSatellites*50),
			SatellitePosition[i % 32].y+(m_numSatellites*50),
			SatellitePosition[i % 32].z,
			authenticPRN[i % 12],
			TxPower,
			StartTime // One Satellite
		);
	}

	NS_LOG_INFO("Finish : CreateNSatellite");
}

void Experiment::CreateOneSatellite(double x, double y, double z, uint32_t PRN, double TxPower, double StartTime)
{
	NS_LOG_INFO("Start : CreateOneSatellite");

	m_numSatellites++;

	// Create Satellite Node
	Ptr<Node> satellite = CreateObject<Node>();
	char address[5] = {'0', '0', ':', '0', '0'};

	int i = m_numSatellites + m_numDrones - 1;
	int j = m_numSatellites - 1;

	// cout << "i : " << i << endl;
	// cout << "j : " << j << endl;

	// Assigning Address 01:55 -> 32:55

	std::stringstream str;
	str << (i);
	Ptr<LrWpanNetDevice> dev_satellite = CreateObject<LrWpanNetDevice>();
	dev_satellite->GetPhy()->m_isSatellite = 1;
	dev_satellite->GetPhy()->DisableGPS();

	/*
	if (i < 10)
	{
		address[1] = str.str()[0];

		dev_satellite->SetAddress(
			Mac16Address(address)
		);
		m_NetDevicesSatelliteTable.push_back(dev_satellite);
	} else 
	{
		address[1] = str.str()[1];
		address[0] = str.str()[0];
		dev_satellite->SetAddress(
			Mac16Address(address)
		);
		m_NetDevicesSatelliteTable.push_back(dev_satellite);

	}
	*/
	dev_satellite->SetAddress(
		Mac16Address(((m_numSatellites)*256)+85)
	);
	m_NetDevicesSatelliteTable.push_back(dev_satellite);

	LrWpanHelper helper;

	helper.EnablePcap(string("/media/tahar/Projects/sim/results/GPS_Satellite_DEV_"+to_string(m_numSatellites)+"_PRN11"), dev_satellite, true, true);

	Mac16Address mac16Address = dev_satellite->GetMac()->GetShortAddress();
	cout << "i : " << i << " / Satellite Mac 16 Address : " << mac16Address << endl;
	/*
	cout << "i : " << i << " / Satellite Mac Address : " << dev_satellite->GetAddress() << endl;
	cout << "i : " << i << " / Satellite Mac Address : " << dev_satellite->GetMac()->GetExtendedAddress() << endl;
	*/

	// Setting Mobility
	Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
	dev_satellite->GetPhy()->SetMobility(mobility);
	mobility->SetPosition(Vector(x, y, z));
	
	// Retrieve UAV Assigned Mobility to Deploy in NetAnim 
	Ptr<ConstantPositionMobilityModel> mobility_00;

	MobilityHelper mobilityHelper;
	mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	// One Satellite
	mobilityHelper.Install(satellite);
	mobility_00 = satellite->GetObject<ConstantPositionMobilityModel>();
	mobility_00->SetPosition(Vector(x, y, z));
	cout << "i : " << i << " / Satellite : " << dev_satellite->GetPhy()->GetMobility()->GetPosition() << endl;
	
	/*
	Ptr<MobilityModel> mob = m_drones.Get(m_numSatellites)->GetObject<MobilityModel>();
	cout << "Node : " << m_numSatellites << " / Node : " << mob->GetPosition() << endl;
	*/

	LrWpanSpectrumValueHelper svh;
	Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(TxPower, GPSchannelNumber);

	// Assigning Power and Channel
	dev_satellite->SetChannel(GPSchannel);
	dev_satellite->GetPhy()->SetTxPowerSpectralDensity(psd);

	// Assigning Satellite (NetworkCard) Devices to Satellite Node
	satellite->AddLrWpanNetDevice(dev_satellite);
	satellite->AddDevice(dev_satellite);

	// m_satellites.Add(satellite);

	Simulator::Schedule(
		Seconds(StartTime),
		&Experiment::StartSatelliteApp,
		this,
		satellite,
		dev_satellite,
		TxPower,
		PRN
	);

	NS_LOG_INFO("Finish : CreateOneSatellite");
}

void Experiment::StartSatelliteApp(Ptr<Node> satellite, Ptr<LrWpanNetDevice> dev_satellite, double TxPower, uint32_t PRN)
{
	NS_LOG_INFO("Start : StartSatelliteApp");

	SatelliteHelper satelliteHelper;

	satelliteHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
	satelliteHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
	satelliteHelper.SetAttribute("TxPower", DoubleValue(TxPower));
	satelliteHelper.SetAttribute("PRN", UintegerValue(PRN));

	satelliteHelper.SetAttribute("Phy", PointerValue(dev_satellite->GetPhy()));
	satelliteHelper.SetAttribute("Mac", PointerValue(dev_satellite->GetMac()));

	satelliteHelper.SetAttribute("SatelliteNumber", (CallbackValue)MakeCallback(&Experiment::SatelliteNumber, this));


	m_SatelliteApps = satelliteHelper.Install(satellite);

	m_satellites.Add(satellite);

	NS_LOG_INFO("Finish : StartSatelliteApp");
}


