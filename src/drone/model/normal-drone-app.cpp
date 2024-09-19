/*
 * normal-drone-app.cpp
 *
 *  Created on: September 2024
 *      Author: Tahar_Final
 */

#include "normal-drone-app.h"

#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nstime.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-state.h"
#include "ns3/pointer.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-module.h"
#include <ns3/lr-wpan-mac.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/mac16-address.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NormalDroneApp");

NS_OBJECT_ENSURE_REGISTERED(NormalDroneApp);

TypeId
NormalDroneApp::GetTypeId(void)
{
	static TypeId tid =
		TypeId("ns3::NormalDroneApp")
			.SetParent<Application>()
			.SetGroupName("Applications")
			.AddConstructor<NormalDroneApp>()
			.AddAttribute("PacketSize",
						  "The size of the NormalDroneApp packet sent to neighbors.",
						  UintegerValue(4),
						  MakeUintegerAccessor(&NormalDroneApp::m_pktSize),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("PacketFrequency",
						  "The time interval of sending a single packet.",
						  TimeValue(Seconds(3)),
						  MakeTimeAccessor(&NormalDroneApp::m_pktFreq),
						  MakeTimeChecker())
			.AddAttribute("Protocol",
						  "The type of protocol to use. (e.g. UdpSocketFactory)",
						  TypeIdValue(UdpSocketFactory::GetTypeId()),
						  MakeTypeIdAccessor(&NormalDroneApp::m_tid),
						  MakeTypeIdChecker())
						  
			.AddAttribute("JITTER",
						  "The UniformRandomVariable used to create jitter when starting.",
						  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_jitter),
						  MakePointerChecker<RandomVariableStream>())
			
			.AddAttribute("GetDevice",
						  "Callback function",
						  CallbackValue(),
						  MakeCallbackAccessor(&NormalDroneApp::GetDevice),
						  MakeCallbackChecker()
						)
/*
			.AddAttribute("m_container",
						  "Simulation Nodes Container",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakeAttributeContainerAccessor(&NormalDroneApp::m_container),
						  MakeAttributeContainerChecker<NodeContainer>())

			.AddAttribute("m_container",
						  "Simulation Nodes Container",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakeAttributeContainerAccessor(&NormalDroneApp::m_container),
						  MakePointerChecker<NodeContainer>())
*/


			.AddAttribute("Mob",
						  "Random mobility pointer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_randomMobility),
						  MakePointerChecker<RandomWalk2dMobilityModel>())
						  
			.AddAttribute("Phy",
						  "Physic layer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_phy),
						  MakePointerChecker<LrWpanPhy>())

			.AddAttribute("GPS_PRN0",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN0),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN1",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN1),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN2",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN2),
						  MakePointerChecker<LrWpanNetDevice>())
			.AddAttribute("GPS_PRN3",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN3),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN4",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN4),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN5",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN5),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN6",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN6),
						  MakePointerChecker<LrWpanNetDevice>())
			.AddAttribute("GPS_PRN7",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN7),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN8",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN8),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN9",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN9),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("GPS_PRN10",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN10),
						  MakePointerChecker<LrWpanNetDevice>())
			.AddAttribute("GPS_PRN11",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_gps_PRN11),
						  MakePointerChecker<LrWpanNetDevice>())

			.AddAttribute("Methode",
						  "The size of the NormalDroneApp packet sent to neighbors.",
						  UintegerValue(0),
						  MakeUintegerAccessor(&NormalDroneApp::m_methode),
						  MakeUintegerChecker<uint32_t>(1))

			.AddAttribute("Mac",
						  "Physic layer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&NormalDroneApp::m_mac),
						  MakePointerChecker<LrWpanMac>())
			.AddAttribute("Victim",
						  "1 si l'uav est une victim.",
						  UintegerValue(0),
						  MakeUintegerAccessor(&NormalDroneApp::m_isVictim),
						  MakeUintegerChecker<uint32_t>(1))

			.AddTraceSource("Rx",
							"Received NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::m_rxPacketTrace),
							"ns3::NormalDroneApp::NormalDroneAppPacketRxTracedCallback")

			.AddTraceSource("RxAlert",
							"Received NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::m_rxAlert),
							"ns3::NormalDroneApp::NormalDroneAppPacketRxAlert")

			.AddTraceSource("RxDetection",
							"Received NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::m_rxDetection),
							"ns3::NormalDroneApp::NormalDroneAppPacketRxDetection")
			.AddTraceSource("Tx",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::m_txPacketTrace),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")

			.AddTraceSource("SNRPerformance",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::SNRPerformance),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")

			.AddTraceSource("TrustPerformance",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::TrustPerformance),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")
			
			.AddTraceSource("DronePacketNumber",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::DronePacketNumber),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")

			.AddTraceSource("SatelliteNumber",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::SatelliteNumber),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")

			.AddTraceSource("DroneReceivedNumber",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::DroneReceivedNumber),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback")

			.AddTraceSource("SatelliteReceivedNumber",
							"Sent NormalDroneApp avoidance packet.",
							MakeTraceSourceAccessor(&NormalDroneApp::SatelliteReceivedNumber),
							"ns3::NormalDroneApp::NormalDroneAppPacketTxTracedCallback");

	return tid;
}

NormalDroneApp::NormalDroneApp()
	: m_state(0),
	  m_socket(0),
	  m_sent(0),
	  m_received(0)
{
	NS_LOG_FUNCTION(this);
	// m_state = GetOlsrState ();
}

NormalDroneApp::~NormalDroneApp()
{
	NS_LOG_FUNCTION(this);
}

void
NormalDroneApp::DoDispose(void)
{
	NS_LOG_FUNCTION(this);

	// Do any cleaning up here.

	// Application::DoDispose();
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/************************************* Methodes d'indications
 * ******************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************************CallBack**********************************************************************************************************************/
/******************************************************************************************************************************************************************/

double
Distance(Vector position1, Vector position2)
{
	double distance = 0;
	// std::cout<<"\n(x1 :" << position1.x <<"   ,   ";
	// std::cout<<"y1 :" << position1.y <<"   ,   ";
	// std::cout<<"z1 :" << position1.z <<")";

	// std::cout<<"   (x2 :" << position2.x <<"  ,  ";
	// std::cout<<" y2 :" << position2.y <<"  ,  ";
	// std::cout<<" z2 :" << position2.z <<")";

	distance = sqrt((position1.x - position2.x) * (position1.x - position2.x) +
					(position1.y - position2.y) * (position1.y - position2.y) +
					(position1.z - position2.z) * (position1.z - position2.z));

	return distance;
}

void
PlmeEdConfirm(NormalDroneApp* App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
{
	std::cout << "Start : PlmeEdConfirm (NormalDroneApp) " << std::endl;
	std::cout << "Finish : PlmeEdConfirm (NormalDroneApp) " << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
PlmeEdConfirm_V2(NormalDroneApp* App,
				 Ptr<Node> drone,
				 LrWpanPhyEnumeration status,
				 uint8_t level,
				 double averagePower,
				 double m_InterferenceAvg,
				 double myPRN)
{
	// std::cout << "Start : PlmeEdConfirm_V2 (NormalDroneApp) " << std::endl;
	// std::cout << "time : " << Simulator::Now().GetSeconds() << std::endl;

	/****************************recuperation du courant
	 * device*********************************************/
	Ptr<LrWpanNetDevice> dev; //  = CreateObject<LrWpanNetDevice>();
	dev = drone->GetLrWpanNetDevice(0);

	/****************************recuperation du courant GPS
	 * device*********************************************/
	
	/*
	Ptr<LrWpanNetDevice> GPSdev; //  = CreateObject<LrWpanNetDevice>();
	for (int i = 1; i < 13; i++)
	{
		GPSdev = drone->GetLrWpanNetDevice(i);
		std::cout << "GPSdev->GetPhy()->GetGPSstate() = " << GPSdev->GetPhy()->GetGPSstate() << std::endl;
		std::cout << "GPSdev->GetAddress() = "  << i << " : " << GPSdev->GetAddress() << std::endl;
		GPSdev->GetPhy()->SetRxSensitivity(-200);
	}
	*/
	// std::cout << "LrWpanPhyEnumeration : status : " << status << std::endl;
	// std::cout << "uint8_t : level : " << level << std::endl;


	// GPSdev = drone->GetLrWpanNetDevice(1);
	// GPSdev->GetPhy()->EnableGPS();

	// Ptr<LrWpanNetDevice> me = GetNode();
	// std::cout << "me : " << me << std::endl;
	// std::cout << "me address : " << me->GetMac()->GetShortAddress() << std::endl;
	// std::cout << " : " << Address << std::endl;
	

	/****************************recuperation de l'adress du
	 * sender*********************************************/
	Mac16Address Address = dev->GetMac()->GetShortAddress();
	uint8_t* MonAdd = new uint8_t[2];
	Address.CopyTo(MonAdd);
	// std::cout << "Sender Address : " << Address << std::endl;
	// std::cout << "Sender : " << dev << std::endl;

	/***************************** recuperation de ma
	 * position****************************************/
	Vector position;
	Ptr<MobilityModel> mobility = dev->GetPhy()->GetMobility();
	position = mobility->GetPosition();
	// mousaab std::cout<<"	je suis le noeud ["<< Address<<"] ma position reel (from IMU) est : x :"
	// << position.x<<" , y= "<<position.y<<" , z= "<<position.z<<"    \n\n";

	App->m_LastEnergy = static_cast<uint32_t>(level);
	// std::cout << "m_LastEnergy : " << App->m_LastEnergy << std::endl;
	// std::cout << "average power : " << averagePower  << " watt" << std::endl;
	// std::cout << "average power : " << 10.0 * log10(averagePower) << " db" << std::endl;
	// std::cout << "average power : " << (30+10.0*log10(averagePower)) << " dbm" << std::endl;

    App->m_LastPowerWatt = averagePower; // in watt
    App->m_LastPowerDB = 10.0 * log10(averagePower); // in dB
    App->m_LastPowerDBM = 10.0 * log10(averagePower) + 30; // in dBm
	
	double m_rxSensitivity = pow(10.0, -106.58 / 10.0) / 1000.0;
	// std::cout << "m_rxSensitivity : pow(10.0, -106.58 / 10.0) / 1000.0 = " << (pow(10.0, -106.58 / 10.0) / 1000.0) << std::endl;

	static const double C = 299792458.0; // speed of light in vacuum
	double m_lambda = C / 1575420000; // GPS signal bandwidth

	double numerator = m_lambda * m_lambda;
	double denominator = 16 * M_PI * M_PI * 1000 * 1000;
	double lossDb = -10 * log10(numerator / denominator);

	double ratio = 10.0 * log10(averagePower / m_rxSensitivity);

	// std::cout << "Ratio : " << ratio << std::endl;
	App->m_LastRatio = ratio; // -

	// mousaab std::cout<<"	Energy Detection completed with status:  " <<status<<" : "<<
	// LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " .... energy level " <<
	// static_cast<uint32_t> (level)<<" .... real received Power (with sebsitivity)=
	// "<<static_cast<double> (ratio)<<"\n"; mousaab
	// std::cout<<"_________________________________________________________\n\n";
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

static void
DataConfirm(McpsDataConfirmParams params)
{
	std::cout << "Start : DataConfirm (NormalDroneApp) " << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds ()<< std::endl;

	std::cout << "Finish : DataConfirm (NormalDroneApp) " << std::endl;
	// std::cout<<"_____________________DataConfirm (NormalDroneApp)
	// ____________________________________\n ";

	/*

	 std::cout<<"________________________ DataConfirm
	(NormalDroneApp)_________________________________\n "; std::cout<<"time :" << Simulator::Now
	().GetSeconds ()<<"\n";

	std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

	std::cout<<"_________________________________________________________\n ";
*/
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

int
ifNodeExist(std::vector<TrustModel> nodeTrust, uint16_t number)
{
	int i = 0;
	
	while ((i < nodeTrust.size()))
	{

		if (nodeTrust[i].number == number)
		{
			return i;
		}
		i++;
	}
	return -1;
}

void
positionEvaluation(std::vector<TrustModel> nodeTrust)
{
	double normalizedPosition = 0;
	// double normalizedPosition = 0;

	double calculatedLevel = 0;

	std::cout << "-- Position Normalization Process -- " << std::endl;

	int minEdges = nodeTrust.size()-1;
	int maxEdges = 0;

	int counter = 0;
	for (int i = 0; i < nodeTrust.size(); i++)
	{
		int element_01 = nodeTrust[i].previousData.size()-1;

		for (int j = 0; j < nodeTrust.size(); j++)
		{
			counter = 0;
			if (i != j)
			{
				Vector tempVector_01;
				tempVector_01.x = nodeTrust[i].previousData[element_01].nodePosition.x;
				tempVector_01.y = nodeTrust[i].previousData[element_01].nodePosition.y;
				tempVector_01.z = nodeTrust[i].previousData[element_01].nodePosition.z;
				
				int element_02 = nodeTrust[j].previousData.size()-1;
				Vector tempVector_02;
				tempVector_02.x = nodeTrust[j].previousData[element_02].nodePosition.x;
				tempVector_02.y = nodeTrust[j].previousData[element_02].nodePosition.y;
				tempVector_02.z = nodeTrust[j].previousData[element_02].nodePosition.z;
				
				double nodesDistances = Distance(tempVector_01, tempVector_02);
				std::cout << " i " << i << " / j " << j << " : " << std::endl;
				std::cout << nodesDistances << std::endl;

				if (nodesDistances > 6000)
				{
					counter++;
				}
			}
		}

		if (counter > maxEdges)
		{
			maxEdges = counter;
			nodeTrust[i].previousData[element_01].maxOfEdges = counter;
		}
		if (counter < minEdges)
		{
			minEdges = counter;
		}

		nodeTrust[i].previousData[element_01].numberOfEdges = counter;

		// nodeTrust[i].previousLevel.level += 0.2*0.1;

		std::cout << "minEdges " << minEdges << " ." << std::endl;
		std::cout << "maxEdges " << maxEdges << " ." << std::endl;
		
	}

	// if ((normalizedDistance >= 0) && (normalizedDistance <= 1)) 
	// calculatedLevel += (normalizedDistance*0.4);

	// std::cout << "Global Worthiness " << calculatedLevel << " ." << std::endl;

	// CollectedData data;
	// data.moment = nodeTrust[index].currentData.moment;
	// data.nodePosition = nodeTrust[index].currentData.nodePosition;
	// data.distance = nodeTrust[index].currentData.distance;
	// data.calculatedDistance = nodeTrust[index].currentData.calculatedDistance;
	// data.receivedPower = nodeTrust[index].currentData.receivedPower;

	// nodeTrust[index].previousData.push_back(data);
}

void
evaluateWorthiness(std::vector<TrustModel> & nodeTrust, int index)
{
	double minDistance = 0;
	double maxDistance = 1000;
	double normalizedDistance = (nodeTrust[index].currentData.distance - minDistance) / (maxDistance - minDistance);

	// double minPower = -105;
	double minPower = -85;
	double maxPower = -36;
	
	double normalizedPower = (nodeTrust[index].currentData.receivedPower - minPower) / (maxPower - minPower);
	
	double differenceDistance = nodeTrust[index].currentData.distance - nodeTrust[index].currentData.calculatedDistance;
	double normalizedDifferenceFlucatuation = 0;

	double normalizedPosition = 0;

	double calculatedLevel = 0;

	std::cout << "-- Normalization Process -- " << std::endl;

	if (differenceDistance == 0)
		calculatedLevel += (1*0.2);
	else
		calculatedLevel += (1.0/(std::abs(differenceDistance)))*0.2;

	if (nodeTrust[index].previousData.size() > 0)
	{
		// Use previous Data		
	}

	if ((normalizedPosition >= 0) && (normalizedPosition <= 1)) 
	{
		calculatedLevel += (normalizedPosition*0.2);
	}

	if ((normalizedPower >= 0) && (normalizedPower <= 1))
	{
		calculatedLevel += (normalizedPower*0.2);
	}

	if ((normalizedDistance >= 0) && (normalizedDistance <= 1)) 
	{
		calculatedLevel += (normalizedDistance*0.4);
	}

// 	{
// 		// nodeTrust[index].currentLevel.level = 
// 		calculatedLevel = 
// 		(normalizedDistance*0.4) + (normalizedPower*0.4) + (normalizedPosition*0.2);
// 	}
	
	std::cout << "Basic Worthiness " << calculatedLevel << " ." << std::endl;
	// else
	
	nodeTrust[index].currentLevel.level = (calculatedLevel*0.1) + (nodeTrust[index].previousLevel.level * 0.9); 
	// nodeTrust[index].previousLevelData.push_back(nodeTrust[index].previousLevel);
	// nodeTrust[index].previousData.push_back(nodeTrust[index].currentData);

	CollectedData data;
	data.moment = nodeTrust[index].currentData.moment;
	data.nodePosition = nodeTrust[index].currentData.nodePosition;
	data.distance = nodeTrust[index].currentData.distance;
	data.calculatedDistance = nodeTrust[index].currentData.calculatedDistance;
	data.receivedPower = nodeTrust[index].currentData.receivedPower;

	nodeTrust[index].previousLevel.level = nodeTrust[index].currentLevel.level;

	nodeTrust[index].previousData.push_back(data);
}

void
displayPreviousData(std::vector<TrustModel> nodeTrust, int index)
{
	std::cout << std::endl;
	std::cout << "=========================================================" << std::endl;
	std::cout << "displayPreviousData" << std::endl;

	// for (int i = 0; i < nodeTrust.size(); i++)
	{
		for (int i = 0; i < nodeTrust[index].previousData.size(); i++)
		{
			std::cout << "=========================================================" << std::endl;
			std::cout << "nodeTrust[index].previousData[i].moment " << nodeTrust[index].previousData[i].moment << " ." << std::endl;
			std::cout << "nodeTrust[index].previousData[i].receivedPower " << nodeTrust[index].previousData[i].receivedPower << " ." << std::endl;
			std::cout << "nodeTrust[index].previousData[i].calculatedDistance " << nodeTrust[index].previousData[i].calculatedDistance << " ." << std::endl;
			std::cout << "nodeTrust[index].previousData[i].distance " << nodeTrust[index].previousData[i].distance << " ." << std::endl;
		}
	}
}

void
displayTrusted(std::vector<TrustModel> nodeTrust)
{
	for (int i = 0; i < nodeTrust.size(); i++)
	{
		std::cout << std::endl;
		std::cout << "displayTrusted" << std::endl;
		std::cout << "=========================================================" << std::endl;
		std::cout << "nodeTrust[" << i << "]" << std::endl;
		std::cout << "nodeTrust[i].number " << nodeTrust[i].number << " ." << std::endl;
		std::cout << "nodeTrust[i].id " << nodeTrust[i].id << " ." << std::endl;
		std::cout << "nodeTrust[i].currentLevel " << nodeTrust[i].currentLevel.level << " ." << std::endl;
	
		std::cout << "nodeTrust[i].currentData " << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.time " << nodeTrust[i].currentData.moment << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.x " << nodeTrust[i].currentData.nodePosition.x << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.y " << nodeTrust[i].currentData.nodePosition.y << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.z " << nodeTrust[i].currentData.nodePosition.z << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.calculatedDistance " << nodeTrust[i].currentData.calculatedDistance << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.distance " << nodeTrust[i].currentData.distance << " ." << std::endl;
		std::cout << "nodeTrust[i].currentData.receivedPower " << nodeTrust[i].currentData.receivedPower << " ." << std::endl;
		
		std::cout << "nodeTrust[i].previousData.size()" << nodeTrust[i].previousData.size() << " ." << std::endl;
		// displayPreviousData(nodeTrust, i);
	}
}

void
DataIndication(NormalDroneApp* App,
			   Ptr<Node> drone,
			   Ptr<RandomWalk2dMobilityModel> RandomMob,
			   McpsDataIndicationParams params,
			   Ptr<Packet> p)
{
	std::cout << "Start : DataIndication (NormalDroneApp) " << std::endl;
	App->DroneReceivedNumber(0);

	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	/****************************recuperation de mon
	 * device*********************************************/
	Ptr<LrWpanNetDevice> MonDevDATA; //  = CreateObject<LrWpanNetDevice>();
	MonDevDATA = drone->GetLrWpanNetDevice(0);

	/****************************recuperation de mon
	 * Adresse*********************************************/
	Mac16Address Address = MonDevDATA->GetMac()->GetShortAddress();
	uint8_t* MonAdd = new uint8_t[2];
	Address.CopyTo(MonAdd);
	
	/***************************** recuperation de ma
	 * position****************************************/
	Vector position;
	Ptr<MobilityModel> mobility = MonDevDATA->GetPhy()->GetMobility();
	position = mobility->GetPosition();
	std::cout	<< "je suis le noeud [" << Address
				<< "] ma position ( IMU ) est : "
				<< " ( X = " << position.x << " , Y = " << position.y << " , Z = " << position.z << " )"
				<< std::endl;

	uint8_t* Add = new uint8_t[2];
	params.m_dstAddr.CopyTo(Add);

	/***************************** affichage du packet****************************************/

	std::cout << "Received Answer of size " << p->GetSize() << "\n";
	uint32_t recvSize = p->GetSize();
	uint8_t* bufferTompo = new uint8_t[recvSize];
	p->CopyData((uint8_t*)bufferTompo, recvSize);

	std::cout << "je viens de recevoir le msg suivant : \n ";
	for (uint32_t i = 0; i < recvSize; i++)
	{
		// printf("T [%i]= %u", i, bufferTompo[i]);
	}

	/*************************************** recuperation de la position d'uav
	 * sender***************************************************************************************************************************/
	Vector SenderPosition;
	SenderPosition.x =
		(bufferTompo[7] << 24) + (bufferTompo[6] << 16) + (bufferTompo[5] << 8) + bufferTompo[4];
	SenderPosition.y =
		(bufferTompo[11] << 24) + (bufferTompo[10] << 16) + (bufferTompo[9] << 8) + bufferTompo[8];
	SenderPosition.z = (bufferTompo[15] << 24) + (bufferTompo[14] << 16) + (bufferTompo[13] << 8) +
					   bufferTompo[12];

	App->m_LastPowerDBM = App->m_LastPowerDBM - 4.20;
	App->m_LastPowerDB = App->m_LastPowerDBM - 30;

	DronParams0 PacketsInfo;
	double distance = App->Distance(position, SenderPosition);
	double RxPower = App->DoCalcRxPower(26.8,distance,1575420000);

	// std::cout << "RxPower : " << RxPower << std::endl;

	PacketsInfo.distance = distance;
	PacketsInfo.energy = App->m_LastEnergy;
	App->m_DronesTable.push_back(PacketsInfo);

	uint32_t x = (uint32_t)position.x;
	uint32_t y = (uint32_t)position.y;
	uint32_t z = (uint32_t)position.z;


	// PositionRequest Message
	// Sending PositionResponse Message
	// && (bufferTompo[1] == 0) // for an attack analysis or not
	if ( (bufferTompo[0] == 0) && (bufferTompo[2] == 0) && (bufferTompo[3] == 1) )
	{
		App->DronePacketNumber(0);

		McpsDataRequestParams parameters;
		parameters.m_srcAddrMode = SHORT_ADDR;
		parameters.m_dstAddrMode = SHORT_ADDR;
		parameters.m_dstPanId = 0;
		// params.m_dstAddr = Mac16Address("77:77"); // Address0; // Mac16Address ("00:02");
		parameters.m_dstAddr = params.m_srcAddr; // Address0; // Mac16Address ("00:02");
		parameters.m_msduHandle = 0;
		parameters.m_txOptions = 0;

		// preparing packet
		uint32_t pktSize = p->GetSize();
		uint8_t* data = new uint8_t[pktSize];
		for (uint32_t i = 0; i < pktSize; ++i)
		data[i] = 0;

		data[0] = (uint8_t)0;
		data[1] = (uint8_t)0;
		data[2] = (uint8_t)0;
		data[3] = (uint8_t)2;

		data[4] = (uint8_t)(x) & 255;
		data[5] = (uint8_t)(x >> 8) & 255;
		data[6] = (uint8_t)(x >> 16) & 255;
		data[7] = (uint8_t)(x >> 24) & 255;
	
		data[8] = (uint8_t)(y) & 255;
		data[9] = (uint8_t)(y >> 8) & 255;
		data[10] = (uint8_t)(y >> 16) & 255;
		data[11] = (uint8_t)(y >> 24) & 255;
	
		data[12] = (uint8_t)(z) & 255;
		data[13] = (uint8_t)(z >> 8) & 255;
		data[14] = (uint8_t)(z >> 16) & 255;
		data[15] = (uint8_t)(z >> 24) & 255;

		Ptr<LrWpanNetDevice> device = App->GetDevice(parameters.m_dstAddr.ConvertToInt());
		uint32_t distance = App->Distance(device->GetPhy()->GetMobility()->GetPosition(), position);

		// std::cout << "sending distance" << distance << std::endl;
		
		data[16] = (uint8_t)(distance) & 255;;
		data[17] = (uint8_t)(distance >> 8) & 255;
		data[18] = (uint8_t)(distance >> 16) & 255;
		data[19] = (uint8_t)(distance >> 24) & 255;

		Ptr<Packet> packet;
		packet = Create<Packet>(data, pktSize);
		
		App->SendPositionResponse(parameters, MonDevDATA, packet);
	}

	// PositionResponse Message
	if ( (bufferTompo[0] == 0) && (bufferTompo[2] == 0) && (bufferTompo[3] == 2) )
	// if ( (bufferTompo[0] == 0) && (bufferTompo[1] == 0) && (bufferTompo[2] == 0) && (bufferTompo[3] == 2) )
	{
		// Calculating distance
		static const double C = 299792458.0; // speed of light in vacuum
		double m_lambda = C / 1575420000; // GPS signal bandwidth
		double numerator = 20*log10(m_lambda);
		double powers = -28 - (App->m_LastPowerDB);
		double denominator = 4 * M_PI;
		double temp = (numerator + powers) / 20;
		temp = pow(10, temp);
		double calculatedDistance = temp / (denominator);

		double tempDistance = (bufferTompo[19] << 24) + (bufferTompo[18] << 16) + (bufferTompo[17] << 8) + bufferTompo[16];
	
		// preparing trust data
		TrustModel trust;

		trust.id = params.m_srcAddr;
		trust.number = params.m_srcAddr.ConvertToInt();
		trust.numberOfMessages++;
	
		CollectedData data;
		data.moment = Simulator::Now().GetSeconds();
		data.nodePosition.x = SenderPosition.x;
		data.nodePosition.y = SenderPosition.y;
		data.nodePosition.z = SenderPosition.z;
		data.distance = tempDistance;
		data.calculatedDistance = calculatedDistance;
		data.receivedPower = App->m_LastPowerDBM;
		trust.currentData = data;

		Ptr<LrWpanNetDevice> device = App->GetDevice(trust.number);
		// std::cout << "device->GetPhy()->GetMobility()->GetPosition()" << std::endl;
		// std::cout << device->GetPhy()->GetMobility()->GetPosition() << std::endl;
		
		int test = ifNodeExist(App->nodeTrust, trust.number);
		int index = test;

		if (test == -1)
		{
			trust.startTime = Simulator::Now().GetSeconds();
			App->nodeTrust.push_back(trust);

			index = ifNodeExist(App->nodeTrust, trust.number);
			App->nodeTrust[index].currentData = data;
			
		}
		else
		{
			App->nodeTrust[index].currentData = data;
			// App->nodeTrust[index].currentData = data;
		}
		// displayTrusted(App->nodeTrust);
		evaluateWorthiness(App->nodeTrust, index);

		displayTrusted(App->nodeTrust);
	}
	

	// Receive TrustRequest Message
	// Sending TrustResponse Message
	if ( (bufferTompo[0] == 0) && (bufferTompo[2] == 1) && (bufferTompo[3] == 0) )
	// if ( (bufferTompo[0] == 0) && (bufferTompo[1] == 0) && (bufferTompo[2] == 1) && (bufferTompo[3] == 0) )
	{
		App->DronePacketNumber(0);
		
		std::cout << "Received Trust Request" << std::endl;

		uint16_t nodeAddress = (bufferTompo[17] << 8) + bufferTompo[16];

		int test = ifNodeExist(App->nodeTrust, nodeAddress);
		
		// std::cout << "nodeAddress" << nodeAddress << std::endl;

		
		if (test == -1)

		{
			// std::cout << "test" << test << std::endl;

			return;
		}

		McpsDataRequestParams parameters;
		parameters.m_srcAddrMode = SHORT_ADDR;
		parameters.m_dstAddrMode = SHORT_ADDR;
		parameters.m_dstPanId = 0;
		parameters.m_dstAddr = params.m_srcAddr; // Address0; // Mac16Address ("00:02");
		parameters.m_msduHandle = 0;
		parameters.m_txOptions = 0;

		// preparing packet
		uint32_t pktSize = p->GetSize();
		uint8_t* data = new uint8_t[pktSize];
		for (uint32_t i = 0; i < pktSize; ++i)
		data[i] = 0;

		data[0] = (uint8_t)0;
		data[1] = (uint8_t)0;
		data[2] = (uint8_t)2;
		data[3] = (uint8_t)0;

		data[16] = bufferTompo[16];
		data[17] = bufferTompo[17];
		data[18] = (uint8_t)0;
		data[19] = (uint8_t)0;

		int level = int((App->nodeTrust[test].previousLevel.level)*100.0);

		std::cout << "sent level" << App->nodeTrust[test].previousLevel.level << std::endl;
		std::cout << "sent level" << level << std::endl;

		data[20] = (uint8_t)(level) & 255;
		data[21] = (uint8_t)(level >> 8) && 255;
		data[22] = (uint8_t)(level >> 16) && 255;
		data[23] = (uint8_t)(level >> 24) && 255;

		Ptr<Packet> packet;
		packet = Create<Packet>(data, pktSize);
		
		App->SendTrustResponse(parameters, MonDevDATA, packet);
	}

	if ( (bufferTompo[0] == 0) && (bufferTompo[2] == 2) && (bufferTompo[3] == 0) )
	// if ( (bufferTompo[0] == 0) && (bufferTompo[1] == 0) && (bufferTompo[2] == 2) && (bufferTompo[3] == 0) )
	{
		std::cout << "Received Trust Response" << std::endl;

		uint16_t nodeAddress = (bufferTompo[17] << 8) + bufferTompo[16];

		int test = ifNodeExist(App->nodeTrust, nodeAddress);
		
		if (test == -1)
			return;

		McpsDataRequestParams parameters;
		parameters.m_srcAddrMode = SHORT_ADDR;
		parameters.m_dstAddrMode = SHORT_ADDR;
		parameters.m_dstPanId = 0;
		parameters.m_dstAddr = params.m_srcAddr; // Address0; // Mac16Address ("00:02");
		parameters.m_msduHandle = 0;
		parameters.m_txOptions = 0;

		// uint8_t address[2];
		uint16_t address = (bufferTompo[16]) + (bufferTompo[17] << 8);

		double level = ((bufferTompo[20]) + (bufferTompo[21] << 8) + (bufferTompo[22] << 16) + (bufferTompo[23] << 24));

		// std::cout << " address : " << address << std::endl;
		// std::cout << " level : " << level << std::endl;
	}


	Mac16Address address = params.m_srcAddr;
	Mac16Address maliciousAddress = Mac16Address('EE::00');

	// uint8_t* address = new uint8_t[2];
	// param.m_dstAddr.CopyTo(address);

	std::cout << "TRUE positive" << std::endl;
	// std::cout << "param.m_dstAddr " << param.m_dstAddr.ConvertToInt() << std::endl;

		
	int test = ifNodeExist(App->nodeTrust, address.ConvertToInt());
	if (address.ConvertToInt() > maliciousAddress.ConvertToInt()) // Malicious 
	{
		if (test == -1)
		{
			// attack not detected
			// false negative
			// App->m_rxAlert(AbsolutPower, CN0, 25); // false negative
			App->TrustPerformance(25); // false negative
		}
		else // exist in my trust system
		{
			double level = App->nodeTrust[test].previousLevel.level;
			if (level > 0.20)
			{
				// attack not detected
				// false negative
				// App->m_rxAlert(AbsolutPower, CN0, 25); // false negative
				if (bufferTompo[1] == 1)
					App->TrustPerformance(25); // false negative
				else
					App->TrustPerformance(20); // true negative
			}
			else // level <= 0.46
			{
				// attack detected
				// true positive
				App->TrustPerformance(10); // true positive
			}
		}
	}
	else // Not Malicious
	{
		if (test == -1)
		{
			// not attack not detected
			// true negative
			App->TrustPerformance(20); // true negative
		}
		else // exist in my trust system
		{
			double level = App->nodeTrust[test].previousLevel.level;
			if (level > 0.20)
			{
				// not attack not detected
				// true negative
				App->TrustPerformance(20); // true negative
			}
			else // level <= 0.46
			{
				// not attack, detected
				// false positive
				App->TrustPerformance(15); // false positive
			}
		}
	}
	
	
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/*********************************************reception d'un msg de
	 * detection*********************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	if ((bufferTompo[0] == 0) && (bufferTompo[1] == 0) && (bufferTompo[2] == 0) &&
		(bufferTompo[3] == 0))
	{
		std::cout << "_____________________________________________________________\n";
		std::cout << "_____________________________________________________________\n";
		std::cout << "_______________j'ai recu un msg de Detection_______________\n";
		std::cout << "_____________________________________________________________\n";
		std::cout << "_____________________________________________________________\n";

		uint32_t detectedPRN = (bufferTompo[23] << 24) + (bufferTompo[22] << 16) +
							   (bufferTompo[21] << 8) + bufferTompo[20];
		uint32_t TTL = (bufferTompo[15] << 24) + (bufferTompo[14] << 16) + (bufferTompo[13] << 8) +
					   bufferTompo[12];

		for (int i = 0; i < 12; i++)
		{
			// std::cout<<" 			App->m_AbsolutGpsPower[i]  =  " <<
			// App->m_AbsolutGpsPower[i]<< "\n";
			if (detectedPRN == authenticPRN[i])
			{
				std::cout << " my PRN index= " << i << "\n ";

				if ((App->m_CurrentUavType[i] == 0))
				{
					// 4: verouillage de reception of Detection Packet
					std::cout << " im a victim : j'ai recu un msg de detection\n ";

					App->m_CurrentUavType[i] = 4;
					std::cout << " verouillage of this PRN detection\n ";

					if (App->m_isLastSignalSpoofing[i] == 1)
					{ //????? ki na7iha ta3tik le vrai taux
						// recalculer le taux
						std::cout << " recalculer le taux\n ";

						App->m_rxDetection(App->m_AbsolutGpsPower[i],
										   App->m_CN0[i],
										   100); // false positive
					}
				}

				// im a witness (verifier si ;e chanel of this prn contient noise)
				if (App->m_CurrentUavType[i] == 2)
				{
					// confirmation: there are a noise
					std::cout << " j'ai deja entendu a noise==> je devienderai un noeud de "
								 "confirmation\n ";

					TTL--;
					PacketsInfo.TTL = TTL;
					if (TTL > 0)
					{
						App->AppLayer(2, params, MonDevDATA, RandomMob, drone, p, PacketsInfo);
						// re-calculer le Taux ????
					}
					else
					{
						std::cout << " fin de TTL \n ";
						// drop the packet
					}
				}

				if ((App->m_CurrentUavType[i] == 1))
				{
					// i know , im an active witness
					std::cout << " im an active witness: je vais rien faire\n ";
				}
			}
		}

		/******************************************************************************************************************************************************************/
		/******************************************************************************************************************************************************************/
		/*********************************************reception d'un msg de
		 * confirmation*********************************************************************************************************/
		/******************************************************************************************************************************************************************/
		/******************************************************************************************************************************************************************/
		if ((bufferTompo[0] == 1) && (bufferTompo[1] == 0) && (bufferTompo[2] == 0) &&
			(bufferTompo[3] == 0))
		{
			std::cout << "_____________________________________________________________\n";
			std::cout << "_____________________________________________________________\n";
			std::cout << "_______________j'ai recu un msg de Confirmation_______________\n";
			std::cout << "_____________________________________________________________\n";
			std::cout << "_____________________________________________________________\n";

			for (int i = 0; i < 12; i++)
			{
				// std::cout<<" 			App->m_AbsolutGpsPower[i]  =  " <<
				// App->m_AbsolutGpsPower[i]<< "\n";
				if (detectedPRN == authenticPRN[i])
				{
					if ((App->m_CurrentUavType[i] == 1))
					{
						// im an active witness , i will do nothing with ur confirmation msg
						std::cout << " im an active witness: je vais rien faire\n ";
					}

					if ((App->m_CurrentUavType[i] == 2))
					{
						// im a forwarder of confirmation Packet (passive witness) ,
						std::cout << " im a formwarded witness: \n ";

						TTL--;
						PacketsInfo.TTL = TTL;
						if (TTL > 0)
						{
							std::cout << "_________________________________________________________"
										 "____\n";
							std::cout << "_________________________________________________________"
										 "____\n";
							std::cout << "_______________forwared it_______________\n";
							std::cout << "_________________________________________________________"
										 "____\n";
							std::cout << "_________________________________________________________"
										 "____\n";

							std::cout << " packet formwarded\n ";

							App->AppLayer(2, params, MonDevDATA, RandomMob, drone, p, PacketsInfo);
							// re-calculer le Taux ????
						}
					}

					if ((App->m_CurrentUavType[i] == 0))
					{
						// calcule of trust level
						std::cout
							<< "_____________________________________________________________\n";
						std::cout
							<< "_____________________________________________________________\n";
						std::cout << "_______________une victim recoi une confirmation gps "
									 "spoofing_______________\n";
						std::cout
							<< "_____________________________________________________________\n";
						std::cout
							<< "_____________________________________________________________\n";

						App->m_rxDetection(App->m_AbsolutGpsPower[i],
										   App->m_CN0[i],
										   100); // false positive
					}
				}
			}
		}

		/******************************************************************************************************************************************************************/
		/******************************************************************************************************************************************************************/
		/*********************************************reception d'un msg de
		 * soupcent*********************************************************************************************************/
		/******************************************************************************************************************************************************************/
		/******************************************************************************************************************************************************************/
		
		if ((bufferTompo[0] == 2) && (bufferTompo[1] == 0) && (bufferTompo[2] == 0) &&
			(bufferTompo[3] == 0))
		{
			std::cout << "_____________________________________________________________\n";
			std::cout << "_____________________________________________________________\n";
			std::cout << "_______________j'ai recu un msg de doupt_______________\n";
			std::cout << "_____________________________________________________________\n";
			std::cout << "_____________________________________________________________\n";

			for (int i = 0; i < 12; i++)
			{
				// std::cout<<" 			App->m_AbsolutGpsPower[i]  =  " <<
				// App->m_AbsolutGpsPower[i]<< "\n";
				if (detectedPRN == authenticPRN[i])
				{
					if ((App->m_CurrentUavType[i] == 1))
					{
						// im an active witness , i will do nothing with ur confirmation msg
						std::cout << " im an active witness: je vais rien faire\n ";
					}

					if ((App->m_CurrentUavType[i] == 2))
					{
					}

					if ((App->m_CurrentUavType[i] == 0))
					{
						// calcule of trust level
						std::cout
							<< "_____________________________________________________________\n";
						std::cout
							<< "_____________________________________________________________\n";
						std::cout << "_______________une victim recoi une doubt gps "
									 "spoofing_______________\n";
						std::cout
							<< "_____________________________________________________________\n";
						std::cout
							<< "_____________________________________________________________\n";

						App->m_rxDetection(App->m_AbsolutGpsPower[i],
										   App->m_CN0[i],
										   100); // false positive
					}
				}
			}
		}

		// App->AppLayer(0,params,MonDevDATA,RandomMob, drone, p,PacketsInfo);
		// App->AppLayer(2,params,MoiDevReceiver,RandomMob, drone, p);
	}

	//   NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
	// std::cout << "Finish : DataIndication (NormalDroneApp) " << std::endl;
}



/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************************    GPS CallBack
 * *********************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
PlmeEdConfirmGPS(NormalDroneApp* App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
{
	// std::cout << "Start : PlmeEdConfirmGPS : (NormalDroneApp)" << std::endl;
	// std::cout << "Finish : PlmeEdConfirmGPS : (NormalDroneApp)" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

uint32_t NormalDroneApp::GetGReceived()
{
	return g_received;
}
uint32_t NormalDroneApp::GetMReceived()
{
	return m_received;
}


void
PlmeEdConfirmGPS_V2(NormalDroneApp* App,
					Ptr<Node> drone,
					LrWpanPhyEnumeration status,
					uint8_t level,
					double averagePower,
					double interference,
					double myPRN)
{
	std::cout << "_____________________________________________________________________________\n";
	std::cout << "Start : PlmeEdConfirmGPS_V2 : (NormalDroneApp)" << std::endl;


// 	std::cout << "Start : PlmeEdConfirmGPS_V2 : (NormalDroneApp)" << std::endl;
// 	std::cout << "time :" << Simulator::Now().GetSeconds()  << std::endl;

	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	/****************************recuperation de l'id de l'uav courant  device  N =1
	 * *********************************************/
	Ptr<LrWpanNetDevice> MoiDev; //  = CreateObject<LrWpanNetDevice>();
	MoiDev = drone->GetLrWpanNetDevice(0);
	// recuperer mon adresse
	Mac16Address Address = MoiDev->GetMac()->GetShortAddress();
	uint8_t* MonAdd = new uint8_t[2];
	Address.CopyTo(MonAdd);

	/****************************recuperation du courant GPS device  N =1
	 * *********************************************/
	Ptr<LrWpanNetDevice> GPSdev; //  = CreateObject<LrWpanNetDevice>();
	GPSdev = drone->GetLrWpanNetDevice(1);

	/***************************** recuperation de ma
	 * position****************************************/

	Vector position;
	Ptr<MobilityModel> mobility = GPSdev->GetPhy()->GetMobility();
	position = mobility->GetPosition();

// 	std::cout 	<< "UAV : [" << Address << "] My Position GPS is :" << std::endl 
// 				<< "x :" << position.x << " , y= " << position.y << ", z = " << position.z 
// 				<< std::endl;


	App->m_LastEnergy = static_cast<uint32_t>(level);
	App->m_interferenceNoise = 10.0 * log10(interference);

	double AbsolutPower = 10.0 * log10(averagePower);

	// cas de Dynamic CN0  non fixe

	// double abspower=  static_cast<double> (AbsolutPower-2);// -2db : a cause de l'attenuation
	// atmospherique
	//  std::cout<<" 			abspower =  " << abspower<< "\n";
	//  std::cout<<" 			PRN from Callback =  " << myPRN<< "\n";

	double CN0;

	for (int j = 1; j < 13; j++)
	{
		int i = j-1;
		Ptr<LrWpanNetDevice> GPSdev_j; //  = CreateObject<LrWpanNetDevice>();
		GPSdev_j = drone->GetLrWpanNetDevice(j);
		// std::cout 	<< "GPSdev_j [" << j << "] : " << GPSdev_j->GetPhy()->m_PrnChannel << std::endl;

		// if (myPRN == authenticPRN[i])
		if (myPRN == GPSdev_j->GetPhy()->m_PrnChannel)
		{
			// std::cout 	<< "GPS DEVICE number : " << i << std::endl;
			// std::cout 	<< "authenticPRN [" << i << "] : " << authenticPRN[i] << std::endl;
			// std::cout 	<< "myPRN [" << i << "] : " << myPRN<< std::endl;
			
			App->m_AbsolutGpsPower[i] = static_cast<double>(
				AbsolutPower - 2); // -2db : a cause de l'attenuation atmospherique
			CN0 = (App->m_AbsolutGpsPower[i]) - (App->m_interferenceNoise);
			App->m_CN0[i] = CN0;

			App->m_rxPacketTrace(App->m_AbsolutGpsPower[i], App->m_CN0[i]);


			// std::cout 	<< "App->m_LastEnergy = " << App->m_LastEnergy << std::endl
			// << "level = " << level << std::endl
			// << "Average Power = " << averagePower << "Watt" << std::endl
			// << "AbsolutPower = " << AbsolutPower << "dB" << std::endl
			// << "interference = " << interference << "Watt" << std::endl
			// << "App->m_interferenceNoise = " << App->m_interferenceNoise << "dB" << std::endl
			// << "App->m_CN0[i] = " << App->m_CN0[i] << "/" << std::endl
			// << "g_received = " << App->GetGReceived() << "/" << std::endl
			// << "m_received = " << App->GetMReceived() << "/" << std::endl;
		}
	}

	std::cout << "Finish : PlmeEdConfirmGPS_V2 : (NormalDroneApp)" << std::endl;

	std::cout << "_____________________________________________________________________________\n";


	// cas de C/N0 fixe
	//	double K_boltzmann = 1.3806 *  pow (10.0, -23) ;
	//	double Temperature=300;
	//	double noise1 =  10.0 * log10 (K_boltzmann*Temperature);

	// double FixeCN0 =( App->m_AbsolutGpsPower) - (noise1);

	// App->m_FixeCN0= FixeCN0;

	// c'est un appel vers DroneExperiment::Cn0VersusTSP (double ReceivingSatellitepower, double
	// CN0) 
	// App->m_rxPacketTrace(App->m_AbsolutGpsPower, App->m_CN0);

	// mousaab std::cout<<"	Energy Detection completed with status:  " <<status<<" : "<<
	// LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " and energy level " <<
	// static_cast<uint32_t> (level)<<" and puissance Recu  = "<<App->m_AbsolutGpsPower<<"  ,
	// Fixenoise= "<<noise1<<"  , FixeCN0 = "<<FixeCN0<<"  dynamic noise = "<<
	// App->m_interferenceNoise<<"   Dynamic CN0 = "<<App->m_CN0<<" \n"; mousaab
	// std::cout<<"_________________________________________________________\n\n";
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

static void
DataConfirmGPS(McpsDataConfirmParams params)
{
	// std::cout << "Start : DataConfirmGPS : (NormalDroneApp)" << std::endl;
	// std::cout << "Finish : DataConfirmGPS : (NormalDroneApp)" << std::endl;

	// std::cout<<"_____________________DataConfirmGPS (NormalDroneApp)
	// ____________________________________\n ";

	/*

   std::cout<<"________________________ DataConfirm
   (NormalDroneApp)_________________________________\n "; std::cout<<"time :" << Simulator::Now
   ().GetSeconds ()<<"\n";

   std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

   std::cout<<"_________________________________________________________\n ";
*/
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
DataIndicationGPS(NormalDroneApp* App,
				  Ptr<Node> drone,
				  Ptr<RandomWalk2dMobilityModel> RandomMob,
				  McpsDataIndicationParams params,
				  Ptr<Packet> p)
{
	std::cout << "_____________________________________________________________________________\n";

	App->SatelliteReceivedNumber(0);

	std::cout << "Start : DataIndicationGPS : (NormalDroneApp)" << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds()  << std::endl;

	uint32_t authenticPRN[] = {
		4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
		2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
		3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
		549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
		28594359,   349684609,  9999122,    224443546};

	/****************************recuperation de mon device of
	 * DATA*********************************************/
	Ptr<LrWpanNetDevice> monDevDATA; //  = CreateObject<LrWpanNetDevice>();
	monDevDATA = drone->GetLrWpanNetDevice(0);

	/****************************recuperation de mon adresse of
	 * DATA*********************************************/
	Mac16Address Address = monDevDATA->GetMac()->GetShortAddress();
	uint8_t* MonAdd = new uint8_t[2];
	Address.CopyTo(MonAdd);
	/***************************** recuperation de ma position
	 * IMU****************************************/
	Vector ma_position;
	Ptr<MobilityModel> mobility = monDevDATA->GetPhy()->GetMobility();
	ma_position = mobility->GetPosition();
	std::cout << "			je suis le noeud [" << Address
			  << "] ma position ( IMU ) est : x :" << ma_position.x << " , y= " << ma_position.y
			  << ", z= " << ma_position.z << "\n\n";

	/***************************** affichage du packet****************************************/

	uint8_t* Add = new uint8_t[2];
	params.m_dstAddr.CopyTo(Add);
	// std::cout<<"Received Answer of size " << p->GetSize ()<<"\n";
	uint32_t recvSize = p->GetSize();
	uint8_t* bufferTompo = new uint8_t[recvSize];
	p->CopyData((uint8_t*)bufferTompo, recvSize);

	// std::cout<<"je viens de recevoir un msg de l'UAV i : \n ";

	for (uint32_t i = 0; i < recvSize; i++)
	{
		// printf ("data [%i]= %u |",i,bufferTompo[i]);
	}

	/***************************** temps du satellite****************************************/

	// uint32_t timeGPS =
	// (bufferTompo[3]<<24)+(bufferTompo[2]<<16)+(bufferTompo[1]<<8)+bufferTompo[0]; double
	// timeGPS_MicroSeconde = timeGPS/1000000.0;

	// std::cout<<"\n			time of satellite =  " << timeGPS<<" miliseconde\n";
	// std::cout<<"			time of satellite =  " << timeGPS_MicroSeconde<<" seconde \n";

	/***************************** adresse du satellite****************************************/
	uint32_t SatelliteID_part1 = (uint32_t)bufferTompo[18] & 255;
	uint32_t SatelliteID_part2 = (uint32_t)bufferTompo[19] & 255;

	// std::cout << " 			satellite ID =  " << SatelliteID_part2 << ":" << SatelliteID_part1 << cout::endl; 
	std::cout << " 			satellite ID =  " << SatelliteID_part2 << ":" << SatelliteID_part1 << std::endl; 
	// 		  << ":55 \n";

	/*************************************** position du
	 * satellite***************************************************************************************************************************/

	Vector SateliitePosition;
	SateliitePosition.x =
		(bufferTompo[7] << 24) + (bufferTompo[6] << 16) + (bufferTompo[5] << 8) + bufferTompo[4];
	SateliitePosition.y =
		(bufferTompo[11] << 24) + (bufferTompo[10] << 16) + (bufferTompo[9] << 8) + bufferTompo[8];
	SateliitePosition.z = (bufferTompo[15] << 24) + (bufferTompo[14] << 16) +
						  (bufferTompo[13] << 8) + bufferTompo[12];
	std::cout << " 			Satellite position     x =  " << SateliitePosition.x
			  << " , y =  " << SateliitePosition.y << ", z =  " << SateliitePosition.z << " m \n";

	/*************************************** distance entre satellite et
	 * drone***************************************************************************************************************************/

	double distance = App->Distance(ma_position, SateliitePosition);
	// double RxPower = App->DoCalcRxPower(26.8,distance,1575420000);
	// App->m_LastRatio= static_cast<double> (RxPower-2); // -2db : a cause de l'attenuation
	// atmospherique
	std::cout << "  			distance = " << distance << "\n";

	/*************************************** PRN du
	 * satellite***************************************************************************************************************************/

	// calcule du PRN
	uint32_t ReceivedPRN;
	ReceivedPRN = (bufferTompo[23] << 24) + (bufferTompo[22] << 16) + (bufferTompo[21] << 8) +
				  bufferTompo[20];
	std::cout << " 			ReceivedPRN =  " << ReceivedPRN << "\n";

	/********************* recuperation du CN0/ absolutpower from ED
	 * indication***************************************************************************************************************************/

	double CN0;
	double AbsolutPower;
	uint32_t index;
	for (int i = 0; i < 12; i++)
	{
		// Ptr<LrWpanNetDevice> gps_dev = drone->GetLrWpanNetDevice(i+1);
		// std::cout << "gps_dev " << i << ":" << (10*log(gps_dev->GetPhy()->GetMPower().averagePower)) << std::endl;

		// std::cout<<" 			App->m_AbsolutGpsPower[i]  =  " << App->m_AbsolutGpsPower[i]<<
		// "\n";
		if (ReceivedPRN == authenticPRN[i])
		{
			index = i;
			CN0 = App->m_CN0[i];
			App->m_LastCN0 = App->m_CN0[i];
			AbsolutPower = App->m_AbsolutGpsPower[i];
			App->m_LastAbsolutGpsPower = App->m_AbsolutGpsPower[i];

			/************************************deverouillage de
			 * detection***************************************************************************************************************************************/
			std::cout << "  		 Deverouillage of detection msg \n";

			App->m_CurrentUavType[i] = 0;

			/************************************marquer le type du last prn
			 * (malicious/normal)***************************************************************************************************************************************/

			if ((SatelliteID_part2 == 15) && (SatelliteID_part1 == 15))
			{
				// 1: the last GPS signal is malicious
				App->m_isLastSignalSpoofing[i] = 1;
			}
			else
			{
				App->m_isLastSignalSpoofing[i] = 0;
			}
		}
	}

	/***************************************************************************************************************************************************************************/
	/************************** verification + detection of spoofing
	 * signal*********************************************************************************************************/
	/***************************************************************************************************************************************************************************/
	/***************************************************************************************************************************************************************************/
	/*****************************************
	 * statistiques***********************************************************************************************************************/
	/***************************************************************************************************************************************************************************/

	/************************Statistique : cas1: absolute
	 * power*********************************************************************************************************************/


	if ((SatelliteID_part2 == 15) && (SatelliteID_part1 == 15))
	{
		if (AbsolutPower > -158)
		{
			App->SNRPerformance(10); // Satellite true positive
		}
		else
		{
			App->SNRPerformance(15); // Satellite false positive
		}
	}
	else 
	{
		if (AbsolutPower > -158)
		{
			App->SNRPerformance(25); // Satellite false positive
		}
		else
		{
			App->SNRPerformance(20); // Satellite true positive
		}
	}

	if (false)
	{
		if ((SatelliteID_part2 == 15) && (SatelliteID_part1 == 15))
		{
			if (AbsolutPower > -153)
			{
				App->m_rxAlert(AbsolutPower, CN0, 10); // true positive
			}
			else
			{
				App->m_rxAlert(AbsolutPower, CN0, 15); // false positive
			}
		}
		else
		{
			if (AbsolutPower > -153)
			{
				App->m_rxAlert(AbsolutPower, CN0, 25); // false negative
			}
			else
			{
				App->m_rxAlert(AbsolutPower, CN0, 20); // true negative
			}
		}
	}

	/**************************Statistique : cas2:
	 * C/N0*********************************************************************************************************************/

	/*

	if ((SatelliteID_part2 ==15) &&(SatelliteID_part1==15)){
		   if (  (CN0 <  42.50) || (CN0>47.50)){
					   App->m_rxAlert (AbsolutPower, CN0,10); // true positive

				}else{
					   App->m_rxAlert (AbsolutPower, CN0,15); // false positive

				}
			}else {

				   if (  (CN0 <  42.50) || (CN0>47.50)){
					   App->m_rxAlert (AbsolutPower, CN0,25); // false negative

				}else{
					   App->m_rxAlert (AbsolutPower, CN0,20); // true negative

				}
			}
*/

	/**************************Statistique : cas3: Absolute Power +
	 * C/N0r*********************************************************************************************************************/

	if ((App->m_methode == 2) || (App->m_methode == 3))
	{
		if ((SatelliteID_part2 == 15) && (SatelliteID_part1 == 15))
		{
			if (((CN0 < 42.50) || (CN0 > 47.50)) || (AbsolutPower > -153))
			{
				App->m_rxAlert(AbsolutPower, CN0, 10); // true positive
			}
			else
			{
				App->m_rxAlert(AbsolutPower, CN0, 15); // false positive
			}
		}
		else
		{
			if (((CN0 < 42.50) || (CN0 > 47.50)) || (AbsolutPower > -153))
			{
				App->m_rxAlert(AbsolutPower, CN0, 25); // false negative
			}
			else
			{
				App->m_rxAlert(AbsolutPower, CN0, 20); // true negative
			}
		}
	}

	std::cout << "... Received power= " << AbsolutPower << "  Dynamic CN0 = " << CN0 << "\n";

	App->gpsSignal.numberSignal++;
	App->gpsSignal.averagePower = ((App->gpsSignal.averagePower * (App->gpsSignal.numberSignal-1)) + AbsolutPower)/App->gpsSignal.numberSignal;

	App->gpsSignal.averageCN = ((App->gpsSignal.averageCN * (App->gpsSignal.numberSignal-1)) + CN0)/App->gpsSignal.numberSignal;
	

	if (AbsolutPower > App->gpsSignal.maxPower)
		App->gpsSignal.maxPower = AbsolutPower;
	
	if (AbsolutPower < App->gpsSignal.minPower)
		App->gpsSignal.minPower = AbsolutPower;

	if (CN0 > App->gpsSignal.maxCN)
		App->gpsSignal.maxCN = CN0;
	
	if (CN0 < App->gpsSignal.minCN)
		App->gpsSignal.minCN = CN0;

	std::cout << "... App->gpsSignal.maxPower = " << App->gpsSignal.maxPower << std::endl;
	std::cout << "... App->gpsSignal.minPower = " << App->gpsSignal.minPower << std::endl;
	std::cout << "... App->gpsSignal.maxCN = " << App->gpsSignal.maxCN << std::endl;
	std::cout << "... App->gpsSignal.minCN = " << App->gpsSignal.minCN << std::endl;
	std::cout << "... App->gpsSignal.averagePower = " << App->gpsSignal.averagePower << std::endl;
	std::cout << "... App->gpsSignal.averageCN = " << App->gpsSignal.averageCN << std::endl;


	DronParams0 PacketsInfo;
	PacketsInfo.distance = distance;
	PacketsInfo.energy = App->m_LastEnergy;
	PacketsInfo.myPRN = ReceivedPRN;
	App->m_DronesTable.push_back(PacketsInfo);

	/***************************************************************************************************************************************************************************/
	/*************************************Envoi des msg de
	 * detection/confirmation:************************************************************************************************************************/
	/***************************************************************************************************************************************************************************/

	// sate : 0= passive witness / potential victim
	// sate : 1= active
	// sate : 2= witness of confirmation

	// if ((AbsolutPower > -153) || (CN0 < 42.50) || (CN0 > 47.50))
	if ((AbsolutPower > -153) || (CN0 < 42.50) || (CN0 > 47.50))
	{
		/******************************************im an active
		 * witness******************************************************************************************************************/
		/******************************************send Detection
		 * Packet******************************************************************************************************************/

		if (AbsolutPower > -153)
		{
			// je l'ai detecter , donc il n'est pas malicious (pour eviter les double)
			std::cout << " power detection: i will send DETECTION PACKET\n ";

			App->m_CurrentUavType[index] = 1;

			App->AppLayer(1, params, monDevDATA, RandomMob, drone, p, PacketsInfo);
		}
		else
		{
			/******************************************im an passive
			 * witness******************************************************************************************************************/
			/***********************************wait for 5 noise in the same
			 * PRN******************************************************************************************************************/

			if ((CN0 < 42.50) || (CN0 > 47.50))
			{
				std::cout << " Noise detection: im waiting for detection msg to confirm it\n ";

				App->m_NoisCounterForPRN[index]++;
				App->m_CurrentUavType[index] = 2;

				if (App->m_NoisCounterForPRN[index] == 3)
				{
					std::cout << " j'ai recu 5 noise , i will send soupcent packet\n ";
					std::cout << "_____________________________________________________________\n";
					std::cout << "_____________________________________________________________\n";
					std::cout << "_______________ send doupt packet_______________\n";
					std::cout << "_____________________________________________________________\n";
					std::cout << "_____________________________________________________________\n";

					// confirmation of noise ==> send soupcent PAcket
					App->m_NoisCounterForPRN[index] = 0;
					App->AppLayer(3, params, monDevDATA, RandomMob, drone, p, PacketsInfo);
				}
			}
			else
			{
				App->m_CurrentUavType[index] = 6;
			}
		}
	}
	else
	{
		std::cout << " j'ai rien detecter , im a potential victim ????\n ";

		App->m_CurrentUavType[index] = 0;
	}

	std::cout << "_____________________________________________________________________________\n";

	//   NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************** Methodes standard d'application
 * ********************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void NormalDroneApp::EnableGPS(Ptr<Node> node)
{
	for (int i = 1; i < 13; i++)
	{
		Ptr<LrWpanNetDevice> dev = node->GetLrWpanNetDevice(0);
		// std::cout << "Node : " << dev->GetMac()->GetShortAddress() << " : ENABLE GPS " << ":" << std::endl;

		Ptr<LrWpanNetDevice> gps;
		gps = node->GetLrWpanNetDevice(i);
		gps->GetPhy()->EnableGPS();
		
		// std::cout << "Node : STATE GPS " << gps->GetPhy()->GetGPSstate() << std::endl;
	}
}

void NormalDroneApp::DisableGPS(Ptr<Node> node)
{
	for (int i = 1; i < 13; i++)
	{
		Ptr<LrWpanNetDevice> dev = node->GetLrWpanNetDevice(0);
		std::cout << "Node : " << dev->GetMac()->GetShortAddress() << " : Disable GPS " << ":" << std::endl;

		Ptr<LrWpanNetDevice> gps;
		gps = node->GetLrWpanNetDevice(i);
		gps->GetPhy()->DisableGPS();
		
		std::cout << "Node : STATE GPS " << gps->GetPhy()->GetGPSstate() << std::endl;
	}
}

void
NormalDroneApp::StartApplication()
{
	if (m_isVictim == 1)
	{
		std::cout << "Start : StartApplication (Victim UAV)" << std::endl;
	}
	else
	{
		std::cout << "Start : StartApplication (Witness UAV)" << std::endl;
	}

	// std::cout << "I am node : " << GetNode()->GetLrWpanNetDevice(1)->GetAddress() << std::endl;
	// GetNode()->GetLrWpanNetDevice(1)->GetPhy()->EnableGPS();

	g_received = 0;
	Simulator::Schedule(Seconds(0.0), &NormalDroneApp::InstallCallBackLrwpan, this);

	for (uint32_t i = 0; i < 12; i++)
	{
		m_NoisCounterForPRN[i] = 0;
	}

	std::cout << "m_pktFreq= : " << m_pktFreq << std::endl;

	double jitter = m_jitter->GetValue();
	// std::cout << " m_jitter->GetValue () = : " << jitter << std::endl;

	// std::cout << "I will activate my GPS : " << Seconds(jitter) << std::endl;
	Simulator::Schedule(Seconds(jitter), &NormalDroneApp::EnableGPS, this, GetNode());

	// std::cout << "I will deactivate my GPS : " << Seconds(1) + Seconds(jitter) + Seconds(jitter) << std::endl;
	// Simulator::Schedule(Seconds(4 + jitter + jitter), &NormalDroneApp::DisableGPS, this, GetNode());

	// std::cout << "I will send my info at : " << m_pktFreq + Seconds(jitter) << std::endl;
	
	// std::cout << "I will send my info at : " << m_pktFreq + m_pktFreq + Seconds((jitter*jitter)) << std::endl;
	m_sendEvent = Simulator::Schedule(m_pktFreq + Seconds(jitter), &NormalDroneApp::SendPositionRequest, this);
	// std::cout << "I will send my info at : " << m_pktFreq + m_pktFreq + m_pktFreq + Seconds((jitter*jitter*jitter)) << std::endl;
	// m_sendEvent = Simulator::Schedule(m_pktFreq + m_pktFreq + m_pktFreq + Seconds((jitter*jitter*jitter)), &NormalDroneApp::SendPositionRequest, this);
	
	if (m_isVictim == 1)
	{

	}
	else
	{
		// m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter),
		// &NormalDroneApp::SendPackets, this);
	}

	Ptr<LrWpanNetDevice> GPSdev; //  = CreateObject<LrWpanNetDevice> ();
	
	for (int i = 1; i < 13; i++)
	{
		// std::cout << "I will activate my GPS : " << Seconds(3) + m_pktFreq + Seconds(jitter) << std::endl;
		GPSdev = GetNode()->GetLrWpanNetDevice(i);
		// GPSdev->GetPhy()->EnableGPS();
		// std::cout << "GPSdev->GetPhy()->GetGPSstate() = " << GPSdev->GetPhy()->GetGPSstate() << std::endl;
		// std::cout << "GPSdev->GetAddress() = "  << i << " : " << GPSdev->GetAddress() << std::endl;
		// GPSdev->GetPhy()->SetRxSensitivity(-200);
		// m_sendEvent = Simulator::Schedule(Seconds(3) + m_pktFreq + Seconds(jitter), &NormalDroneApp::EnableGPS, this, GetNode());
	}

	std::cout << "Finish : StartApplication (Normal UAV)" << std::endl;
}



void
NormalDroneApp::StopApplication()
{
	std::cout << "*********debut StopApplication****************************************\n";

	CancelEvents();

	std::cout << "*********fin StopApplication****************************************\n";
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************** Methodes supplementaires d'application
 * ********************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
NormalDroneApp::InstallCallBackLrwpan()
{
	McpsDataConfirmCallback cb0;
	cb0 = MakeCallback(&DataConfirm);

	McpsDataIndicationCallback cb1;
	cb1 = MakeBoundCallback(&DataIndication, this, GetNode(), m_randomMobility);

	PlmeEdConfirmCallback cb2;
	cb2 = MakeBoundCallback(&PlmeEdConfirm, this, GetNode());

	PlmeEdConfirmCallback_V2 cb2V2;
	cb2V2 = MakeBoundCallback(&PlmeEdConfirm_V2, this, GetNode());

	m_mac->SetMcpsDataConfirmCallback(cb0);
	m_mac->SetMcpsDataIndicationCallback(cb1);

	m_phy->SetPlmeEdConfirmCallback(cb2);
	m_phy->SetPlmeEdConfirmCallback_V2(cb2V2);

	McpsDataConfirmCallback GPScb0;
	GPScb0 = MakeCallback(&DataConfirmGPS);

	McpsDataIndicationCallback GPScb1;
	GPScb1 = MakeBoundCallback(&DataIndicationGPS, this, GetNode(), m_randomMobility);

	PlmeEdConfirmCallback GPScb2;
	GPScb2 = MakeBoundCallback(&PlmeEdConfirmGPS, this, GetNode());

	PlmeEdConfirmCallback_V2 GPScb2V2;
	GPScb2V2 = MakeBoundCallback(&PlmeEdConfirmGPS_V2, this, GetNode());

	m_gps_PRN0->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN0->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN0->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN0->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN1->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN1->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN1->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN1->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN2->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN2->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN2->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN2->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN3->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN3->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN3->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN3->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN4->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN4->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN4->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN4->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN5->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN5->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN5->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN5->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN6->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN6->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN6->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN6->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN7->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN7->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN7->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN7->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN8->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN8->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN8->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN8->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN9->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN9->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN9->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN9->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN10->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN10->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN10->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN10->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);

	m_gps_PRN11->GetMac()->SetMcpsDataConfirmCallback(GPScb0);
	m_gps_PRN11->GetMac()->SetMcpsDataIndicationCallback(GPScb1);
	m_gps_PRN11->GetPhy()->SetPlmeEdConfirmCallback(GPScb2);
	m_gps_PRN11->GetPhy()->SetPlmeEdConfirmCallback_V2(GPScb2V2);
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

uint32_t NormalDroneApp::GetPacketSize()
{
	return m_pktSize;
}


void
NormalDroneApp::AppLayer(int type,
						 McpsDataIndicationParams params,
						 Ptr<LrWpanNetDevice> monDevDATA,
						 Ptr<RandomWalk2dMobilityModel> RandomMob,
						 Ptr<Node> drone,
						 Ptr<Packet> p,
						 DronParams0 PacketsInfo)
{
	if (m_isVictim == 1)
	{
		std::cout << "Start : AppLayer (Normal Drone) (Victim UAV)" << std::endl;
	}
	else
	{
		std::cout << "Start : AppLayer (Normal Drone) (Witness UAV)" << std::endl;
	}

	// std::cout<<"\n______________________ AppLayer
	// (NormalDroneApp)___________________________________\n";
	g_received++;
	std::cout<<"time :" << Simulator::Now ().GetSeconds() << std::endl;
	std::cout<<"g_received :" << g_received << std::endl;

	//	std::cout<<"	g_received ="<< g_received<<" \n";

	if (g_received == 2)
	{
		// Simulator::Schedule (Seconds (0.0), &NormalDroneApp::JumpToTheMoon, this,MoiDevReceiver,
		// RandomMob,drone);
	}

	switch (type)
	{
	case 0: // ACK
		// Simulator::Schedule (Seconds (0.0), &NormalDroneApp::SendAck, this,
		// params,MoiDevReceiver);

		break;
	case 1: // Detection
	/*
		Simulator::Schedule(Seconds(0.0),
							&NormalDroneApp::SendDetection,
							this,
							params,
							monDevDATA,
							PacketsInfo);
	*/
		break;
	case 2: // Confirmation
	/*
		Simulator::Schedule(Seconds(0.0),
							&NormalDroneApp::SendConfirmation,
							this,
							params,
							monDevDATA,
							PacketsInfo);
	*/
		break;

	case 3: // Jump to the moon
		// Simulator::Schedule (Seconds (0.0), &NormalDroneApp::SendDoupt, this,
		// params,monDevDATA,PacketsInfo);

		break;
	case 4:

		break;

	case 5: // PositionRequest Message

		break;
	}

	std::cout << "Finish : AppLayer (Normal UAV)" << std::endl;

	// std::cout<<"_________________________________________________________\n\n";
}

void
NormalDroneApp::ScheduleNextPositionRequest()
{
	// std::cout << "I will send my info at : " << (m_pktFreq + Seconds((jitter))) << std::endl;

	double jitter = m_jitter->GetValue();
	
	Simulator::Schedule(m_pktFreq + Seconds(jitter), &NormalDroneApp::SendPositionRequest, this);
}

void
NormalDroneApp::SendDetection(McpsDataIndicationParams params,
							  Ptr<LrWpanNetDevice> monDevDATA,
							  DronParams0 PacketsInfo)
{
	std::cout << "\n______________________ Send Detection "
				 "(drone-adhoc)___________________________________\n";
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	/****************************recuperation de mon
	 * Adresse*********************************************/
	Mac16Address Address = monDevDATA->GetMac()->GetShortAddress();
	uint8_t* Add = new uint8_t[2];
	Address.CopyTo(Add);
	/************************recuperation de ma position*******************************/
	Vector ma_position;
	Ptr<MobilityModel> mobility = monDevDATA->GetPhy()->GetMobility();
	ma_position = mobility->GetPosition();

	uint32_t x = (uint32_t)ma_position.x;
	uint32_t y = (uint32_t)ma_position.y;
	// uint32_t z= (uint32_t)ma_position.z	;

	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	uint32_t TTL = 3;
	uint32_t ReceivedPRN = PacketsInfo.myPRN;

	// 1 : detection
	// 2 :
	data[0] = (uint8_t)(0) & 255;
	data[1] = (uint8_t)(0) & 255;
	data[2] = (uint8_t)(0) & 255;
	data[3] = (uint8_t)(0) & 255;

	data[4] = (uint8_t)(x) & 255;
	data[5] = (uint8_t)(x >> 8) & 255;
	data[6] = (uint8_t)(x >> 16) & 255;
	data[7] = (uint8_t)(x >> 24) & 255;

	data[8] = (uint8_t)(y) & 255;
	data[9] = (uint8_t)(y >> 8) & 255;
	data[10] = (uint8_t)(y >> 16) & 255;
	data[11] = (uint8_t)(y >> 24) & 255;

	data[12] = (uint8_t)(TTL) & 255;
	data[13] = (uint8_t)(TTL >> 8) & 255;
	data[14] = (uint8_t)(TTL >> 16) & 255;
	data[15] = (uint8_t)(TTL >> 24) & 255;

	data[16] = (uint8_t)5;
	data[17] = (uint8_t)5;
	data[18] = (uint8_t)(Add[0]) & 15;
	data[19] = (uint8_t)(Add[0] >> 4) & 15;

	// cas2 : it send an authentic prn :

	data[20] = (uint8_t)(ReceivedPRN) & 255;
	data[21] = (uint8_t)(ReceivedPRN >> 8) & 255;
	data[22] = (uint8_t)(ReceivedPRN >> 16) & 255;
	data[23] = (uint8_t)(ReceivedPRN >> 24) & 255;

	// data[24]= (uint8_t) (TTL)&255;
	// data[25]= (uint8_t) (TTL >> 8)&255;
	// data[26]= (uint8_t) (TTL >> 16)&255;
	// data[27]= (uint8_t) (TTL >> 24)&255;

	McpsDataRequestParams paramsAnswer;

	paramsAnswer.m_srcAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstPanId = 0;
	paramsAnswer.m_dstAddr = Mac16Address("ff:ff"); // params.m_srcAddr; // Mac16Address ("00:02");
	paramsAnswer.m_msduHandle = 0;
	paramsAnswer.m_txOptions = 0;

	Ptr<Packet> DetectionPacket;

	DetectionPacket = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0),
						&LrWpanMac::McpsDataRequest,
						monDevDATA->GetMac(),
						paramsAnswer,
						DetectionPacket);

	std::cout << "_________________________________________________________\n\n";
}

void
NormalDroneApp::SendConfirmation(McpsDataIndicationParams params,
								 Ptr<LrWpanNetDevice> monDevDATA,
								 DronParams0 PacketsInfo)
{
	std::cout << "\n______________________ Send Confirmation "
				 "(drone-adhoc)___________________________________\n";
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	/****************************recuperation de mon
	 * Adresse*********************************************/
	Mac16Address Address = monDevDATA->GetMac()->GetShortAddress();
	uint8_t* Add = new uint8_t[2];
	Address.CopyTo(Add);
	/************************recuperation de ma position*******************************/
	Vector ma_position;
	Ptr<MobilityModel> mobility = monDevDATA->GetPhy()->GetMobility();
	ma_position = mobility->GetPosition();

	uint32_t x = (uint32_t)ma_position.x;
	uint32_t y = (uint32_t)ma_position.y;
	// uint32_t z= (uint32_t)ma_position.z	;

	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	uint32_t TTL = PacketsInfo.TTL;
	uint32_t ReceivedPRN = PacketsInfo.myPRN;

	// 1 : detection
	// 2 :
	data[0] = (uint8_t)(1) & 255;
	data[1] = (uint8_t)(0) & 255;
	data[2] = (uint8_t)(0) & 255;
	data[3] = (uint8_t)(0) & 255;

	data[4] = (uint8_t)(x) & 255;
	data[5] = (uint8_t)(x >> 8) & 255;
	data[6] = (uint8_t)(x >> 16) & 255;
	data[7] = (uint8_t)(x >> 24) & 255;

	data[8] = (uint8_t)(y) & 255;
	data[9] = (uint8_t)(y >> 8) & 255;
	data[10] = (uint8_t)(y >> 16) & 255;
	data[11] = (uint8_t)(y >> 24) & 255;

	data[12] = (uint8_t)(TTL) & 255;
	data[13] = (uint8_t)(TTL >> 8) & 255;
	data[14] = (uint8_t)(TTL >> 16) & 255;
	data[15] = (uint8_t)(TTL >> 24) & 255;

	data[16] = (uint8_t)5;
	data[17] = (uint8_t)5;
	data[18] = (uint8_t)(Add[0]) & 15;
	data[19] = (uint8_t)(Add[0] >> 4) & 15;

	// cas2 : it send an authentic prn :

	data[20] = (uint8_t)(ReceivedPRN) & 255;
	data[21] = (uint8_t)(ReceivedPRN >> 8) & 255;
	data[22] = (uint8_t)(ReceivedPRN >> 16) & 255;
	data[23] = (uint8_t)(ReceivedPRN >> 24) & 255;

	// data[24]= (uint8_t) (TTL)&255;
	// data[25]= (uint8_t) (TTL >> 8)&255;
	// data[26]= (uint8_t) (TTL >> 16)&255;
	// data[27]= (uint8_t) (TTL >> 24)&255;

	McpsDataRequestParams paramsAnswer;

	paramsAnswer.m_srcAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstPanId = 0;
	paramsAnswer.m_dstAddr = Mac16Address("ff:ff"); // params.m_srcAddr; // Mac16Address ("00:02");
	paramsAnswer.m_msduHandle = 0;
	paramsAnswer.m_txOptions = 0;

	Ptr<Packet> DetectionPacket;

	DetectionPacket = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0),
						&LrWpanMac::McpsDataRequest,
						monDevDATA->GetMac(),
						paramsAnswer,
						DetectionPacket);

	std::cout << "_________________________________________________________\n\n";
}

void
NormalDroneApp::SendDoupt(McpsDataIndicationParams params,
						  Ptr<LrWpanNetDevice> monDevDATA,
						  DronParams0 PacketsInfo)
{
	std::cout
		<< "\n______________________ Send Doupt (drone-adhoc)___________________________________\n";
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	/****************************recuperation de mon
	 * Adresse*********************************************/
	Mac16Address Address = monDevDATA->GetMac()->GetShortAddress();
	uint8_t* Add = new uint8_t[2];
	Address.CopyTo(Add);
	/************************recuperation de ma position*******************************/
	Vector ma_position;
	Ptr<MobilityModel> mobility = monDevDATA->GetPhy()->GetMobility();
	ma_position = mobility->GetPosition();

	uint32_t x = (uint32_t)ma_position.x;
	uint32_t y = (uint32_t)ma_position.y;
	// uint32_t z= (uint32_t)ma_position.z	;

	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	uint32_t TTL = 0;
	uint32_t ReceivedPRN = PacketsInfo.myPRN;

	// 1 : detection
	// 2 :
	data[0] = (uint8_t)(2) & 255;
	data[1] = (uint8_t)(0) & 255;
	data[2] = (uint8_t)(0) & 255;
	data[3] = (uint8_t)(0) & 255;

	data[4] = (uint8_t)(x) & 255;
	data[5] = (uint8_t)(x >> 8) & 255;
	data[6] = (uint8_t)(x >> 16) & 255;
	data[7] = (uint8_t)(x >> 24) & 255;

	data[8] = (uint8_t)(y) & 255;
	data[9] = (uint8_t)(y >> 8) & 255;
	data[10] = (uint8_t)(y >> 16) & 255;
	data[11] = (uint8_t)(y >> 24) & 255;

	data[12] = (uint8_t)(TTL) & 255;
	data[13] = (uint8_t)(TTL >> 8) & 255;
	data[14] = (uint8_t)(TTL >> 16) & 255;
	data[15] = (uint8_t)(TTL >> 24) & 255;

	data[16] = (uint8_t)5;
	data[17] = (uint8_t)5;
	data[18] = (uint8_t)(Add[0]) & 15;
	data[19] = (uint8_t)(Add[0] >> 4) & 15;

	// cas2 : it send an authentic prn :

	data[20] = (uint8_t)(ReceivedPRN) & 255;
	data[21] = (uint8_t)(ReceivedPRN >> 8) & 255;
	data[22] = (uint8_t)(ReceivedPRN >> 16) & 255;
	data[23] = (uint8_t)(ReceivedPRN >> 24) & 255;

	// data[24]= (uint8_t) (TTL)&255;
	// data[25]= (uint8_t) (TTL >> 8)&255;
	// data[26]= (uint8_t) (TTL >> 16)&255;
	// data[27]= (uint8_t) (TTL >> 24)&255;

	McpsDataRequestParams paramsAnswer;

	paramsAnswer.m_srcAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstPanId = 0;
	paramsAnswer.m_dstAddr = Mac16Address("ff:ff"); // params.m_srcAddr; // Mac16Address ("00:02");
	paramsAnswer.m_msduHandle = 0;
	paramsAnswer.m_txOptions = 0;

	Ptr<Packet> DetectionPacket;

	DetectionPacket = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0),
						&LrWpanMac::McpsDataRequest,
						monDevDATA->GetMac(),
						paramsAnswer,
						DetectionPacket);

	std::cout << "_________________________________________________________\n\n";
}

void
NormalDroneApp::SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver)
{
	std::cout
		<< "\n______________________ SendAck (drone-adhoc)___________________________________\n";
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	uint8_t* Add = new uint8_t[2];
	params.m_dstAddr.CopyTo(Add);
	McpsDataRequestParams paramsAnswer;
	paramsAnswer.m_srcAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstAddrMode = SHORT_ADDR;
	paramsAnswer.m_dstPanId = 0;
	paramsAnswer.m_dstAddr = params.m_srcAddr; // Mac16Address ("00:02");
	paramsAnswer.m_msduHandle = 0;
	paramsAnswer.m_txOptions = 0;

	Ptr<Packet> Answer;
	// preparation des donnees a envoyer
	uint8_t* data = new uint8_t[4];
	for (uint32_t i = 0; i < 4; ++i)
		data[i] = 0;

	data[0] = Add[0];
	data[1] = Add[1];

	Answer = Create<Packet>(data, 4);

	Simulator::Schedule(Seconds(0.2),
						&LrWpanMac::McpsDataRequest,
						MoiDevReceiver->GetMac(),
						paramsAnswer,
						Answer);

	std::cout << "_________________________________________________________\n\n";
}

void
NormalDroneApp::JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver,
							  Ptr<RandomWalk2dMobilityModel> RandomMob,
							  Ptr<Node> drone)
{
	std::cout << "\n______________________ ChangeTheMobility "
				 "(drone-adhoc)___________________________________\n";
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	Ptr<ConstantPositionMobilityModel> ConstantMob = CreateObject<ConstantPositionMobilityModel>();

	Ptr<HierarchicalMobilityModel> Mob = drone->GetObject<HierarchicalMobilityModel>();

	RandomMob->SetAttribute("Mode", StringValue("Time"));
	RandomMob->SetAttribute("Time", StringValue("0.5s"));
	RandomMob->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
	RandomMob->SetAttribute("Bounds", StringValue(WalkBounds(0.0, 200.0, 0.0, 200.0)));

	// Mob->SetParent(PointerValue (RandomMob));

	// Mob->SetChild(PointerValue (RandomMob));
	//  RandomMob->StopEvent();
	Mob->SetPosition(Vector(1, 1, 384e6)); // position absolut
	// ConstantMob->SetPosition(Vector (20,20,0)); // position relative

	std::cout << "_________________________________________________________\n\n";
}

void NormalDroneApp::SendTrustResponse(McpsDataRequestParams params, Ptr<LrWpanNetDevice> myNetDevice, Ptr<Packet> p)
{
	DronePacketNumber(0);
	// std::cout << "Start : SendTrustResponse" << std::endl;

	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
	
	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, myNetDevice->GetMac(), params, p);
}

void 
NormalDroneApp::SendTrustRequest(std::vector<TrustModel>& nodeTrust, int index)
{
	DronePacketNumber(0);
	// std::cout << "Start : SendTrustRequest" << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	Ptr<LrWpanNetDevice> myNetDevice; //  = CreateObject<LrWpanNetDevice>();
	myNetDevice = GetNode()->GetLrWpanNetDevice(0);

	Mac16Address myAddress = myNetDevice->GetMac()->GetShortAddress();

	McpsDataRequestParams params;
	params.m_srcAddrMode = SHORT_ADDR;
	params.m_dstAddrMode = SHORT_ADDR;
	params.m_dstPanId = 0;
	params.m_dstAddr = Mac16Address("FF:FF");
	params.m_msduHandle = 0;
	params.m_txOptions = 0;

	// preparing data
	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	data[0] = (uint8_t)0;
	data[1] = (uint8_t)0;
	data[2] = (uint8_t)1;
	data[3] = (uint8_t)0;

	Mac16Address add = nodeTrust[index].id;
	
	uint8_t* address = new uint8_t[2];
	add.CopyTo(address);


	// data[16] = (uint8_t)(address[0]) & 255;
	// data[17] = (uint8_t)(address[1] >> 8) & 255;
	std::cout << "Address" << add << std::endl;
	data[16] = (uint8_t)(address[1]);
	data[17] = (uint8_t)(address[0]);
	data[18] = (uint8_t)0;
	data[19] = (uint8_t)0;

	Ptr<Packet> p;
	p = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, myNetDevice->GetMac(), params, p);
}

void 
NormalDroneApp::SendPositionRequest()
{
	DronePacketNumber(0);
	// std::cout << "Start : SendPositionRequest" << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	numberOfRequests++;
	if (numberOfRequests > 1)
	{
		positionEvaluation(nodeTrust);
		if (nodeTrust.size() > 0)
			SendTrustRequest(nodeTrust, 0);
	}
	// App->AppLayer(2, params, MonDevDATA, RandomMob, drone, p, PacketsInfo);

	Ptr<LrWpanNetDevice> myNetDevice; //  = CreateObject<LrWpanNetDevice>();
	myNetDevice = GetNode()->GetLrWpanNetDevice(0);

	Mac16Address myAddress = myNetDevice->GetMac()->GetShortAddress();

	McpsDataRequestParams params;
	params.m_srcAddrMode = SHORT_ADDR;
	params.m_dstAddrMode = SHORT_ADDR;
	params.m_dstPanId = 0;
	params.m_dstAddr = Mac16Address("FF:FF");
	params.m_msduHandle = 0;
	params.m_txOptions = 0;

	// preparing data
	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	data[0] = (uint8_t)0;
	data[1] = (uint8_t)0;
	data[2] = (uint8_t)0;
	data[3] = (uint8_t)1;

	Ptr<Packet> p;
	p = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, myNetDevice->GetMac(), params, p);
	// Simulator::Schedule(frequency + Seconds(jitter), &NormalDroneApp::ScheduleNextPositionRequest, 2*frequency, 2*jitter);
	ScheduleNextPositionRequest();

}

void NormalDroneApp::SendPositionResponse(McpsDataRequestParams params, Ptr<LrWpanNetDevice> myNetDevice, Ptr<Packet> p)
{
	DronePacketNumber(0);
	// std::cout << "Start : SendPositionResponse" << std::endl;

	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
	
	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, myNetDevice->GetMac(), params, p);
}

void
NormalDroneApp::SendPackets()
{
	DronePacketNumber(0);
	// std::cout << "**********************debut SendPackets*********************************\n";

	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

	/****************************recuperation du mon
	 * device*********************************************/
	Ptr<LrWpanNetDevice> MonDevDATA; //  = CreateObject<LrWpanNetDevice>();
	MonDevDATA = GetNode()->GetLrWpanNetDevice(0);

	/****************************recuperation de mon
	 * Adresse*********************************************/
	Mac16Address Address = MonDevDATA->GetMac()->GetShortAddress();
	uint8_t* Add = new uint8_t[2];
	Address.CopyTo(Add);
	/************************recuperation de ma position*******************************/
	Vector ma_position;
	Ptr<MobilityModel> mobility = MonDevDATA->GetPhy()->GetMobility();
	ma_position = mobility->GetPosition();
	/********************************affichage des informations *********************************/
	std::cout << "	je suis le noeud [" << Address << "] ma position IMU est : x :" << ma_position.x
			  << " , y= " << ma_position.y << " , z= " << ma_position.z << "   \n\n";

	/****************************recuperation des Neighbors devices
	 * *********************************************/
	std::vector<Ptr<LrWpanNetDevice>> voisins = GetNode()->GetALLLrWpanNetDevice();

	// Ptr<LrWpanNetDevice> devReceiver = CreateObject<LrWpanNetDevice>();

	// int MonDevDATA->GetPhy()->GetTxPowerSpectralDensity()->GetValues().size

	std::cout << "Msg ennvoye a tt le monde" << std::endl;

	McpsDataRequestParams params;
	params.m_srcAddrMode = SHORT_ADDR;
	params.m_dstAddrMode = SHORT_ADDR;
	params.m_dstPanId = 0;
	// params.m_dstAddr = Mac16Address("77:77"); // Address0; // Mac16Address ("00:02");
	params.m_dstAddr = Mac16Address("FF:FF"); // Address0; // Mac16Address ("00:02");
	params.m_msduHandle = 0;
	params.m_txOptions = 0;

	Ptr<LrWpanNetDevice> GPSdev; //  = CreateObject<LrWpanNetDevice> ();
	GPSdev = GetNode()->GetLrWpanNetDevice(1);

	std::cout << "GPSdev->GetPhy()->GetGPSstate() = " << GPSdev->GetPhy()->GetGPSstate() << std::endl;
	// GPSdev->GetPhy()->GetCurrentSignalPsd();
	// Simulator::Schedule (Seconds (0.0025), &LrWpanPhy::PlmeEdRequest, GPSdev->GetPhy ());
	// Simulator::Schedule(Seconds (0.1), &LrWpanPhy::PlmeEdRequest);
	// Simulator::Schedule(Seconds (0.1), &LrWpanPhy::PlmeSatelliteEdRequest, Seconds(5), spectrum);
	// GPSdev->GetPhy()->PlmeEdRequest();

	double time_MicroSeconde = Simulator::Now().GetSeconds() * 1000000;
	// preparation des donnees a envoyer (first 3 subframes durant 30 seconde)
	uint32_t time = (uint32_t)time_MicroSeconde; // time en miliseconde

	uint32_t x = (uint32_t)ma_position.x;
	uint32_t y = (uint32_t)ma_position.y;
	uint32_t z = (uint32_t)ma_position.z;

	uint8_t* data = new uint8_t[m_pktSize];
	for (uint32_t i = 0; i < m_pktSize; ++i)
		data[i] = 0;

	data[0] = (uint8_t)(time) & 255;
	data[1] = (uint8_t)(time >> 8) & 255;
	data[2] = (uint8_t)(time >> 16) & 255;
	data[3] = (uint8_t)(time >> 24) & 255;

	data[4] = (uint8_t)(x) & 255;
	data[5] = (uint8_t)(x >> 8) & 255;
	data[6] = (uint8_t)(x >> 16) & 255;
	data[7] = (uint8_t)(x >> 24) & 255;

	data[8] = (uint8_t)(y) & 255;
	data[9] = (uint8_t)(y >> 8) & 255;
	data[10] = (uint8_t)(y >> 16) & 255;
	data[11] = (uint8_t)(y >> 24) & 255;

	data[12] = (uint8_t)(z) & 255;
	data[13] = (uint8_t)(z >> 8) & 255;
	data[14] = (uint8_t)(z >> 16) & 255;
	data[15] = (uint8_t)(z >> 24) & 255;

	data[16] = (uint8_t)5;
	data[17] = (uint8_t)5;
	data[18] = (uint8_t)(Add[0]) & 15;
	data[19] = (uint8_t)(Add[0] >> 4) & 15;

	Ptr<Packet> p;

	p = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, MonDevDATA->GetMac(), params, p);
	ScheduleNextTx(0);

	std::cout << "***********************************************************************\n";
	std::cout << "**********************fin SendPackets*********************************\n";
	std::cout << "***********************************************************************\n\n";
}

void
NormalDroneApp::ScheduleNextTx(double type)
{
	// std::cout<<"*********debut ScheduleNextTx****************************************\n";

	m_sendEvent = Simulator::Schedule(m_pktFreq, &NormalDroneApp::SendPackets, this);

	// std::cout<<"*********fin ScheduleNextTx****************************************\n";
}

std::string
NormalDroneApp::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
	NS_LOG_INFO("Tahar_Final debut + fin WalkBounds");

	return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" +
		   std::to_string(maxY);
}

double
NormalDroneApp::Distance(Vector position1, Vector position2)
{
	double distance = 0;
	// std::cout<<"\n(x1 :" << position1.x <<"   ,   ";
	// std::cout<<"y1 :" << position1.y <<"   ,   ";
	// std::cout<<"z1 :" << position1.z <<")";

	// std::cout<<"   (x2 :" << position2.x <<"  ,  ";
	// std::cout<<" y2 :" << position2.y <<"  ,  ";
	// std::cout<<" z2 :" << position2.z <<")";

	distance = sqrt((position1.x - position2.x) * (position1.x - position2.x) +
					(position1.y - position2.y) * (position1.y - position2.y) +
					(position1.z - position2.z) * (position1.z - position2.z));

	return distance;
}

double
NormalDroneApp::DoCalcRxPower(double txPowerDbm, double distance, double frequency)
{
	/*
	 * Friis free space equation:
	 * where Pt, Gr, Gr and P are in Watt units
	 * L is in meter units.
	 *
	 *    P     Gt * Gr * (lambda^2)
	 *   --- = ---------------------
	 *    Pt     (4 * pi * d)^2 * L
	 *
	 * Gt: tx gain (unit-less)
	 * Gr: rx gain (unit-less)
	 * Pt: tx power (W)
	 * d: distance (m)
	 * L: system loss
	 * lambda: wavelength (m)
	 *
	 * Here, we ignore tx and rx gain and the input and output values
	 * are in dB or dBm:
	 *
	 *                           lambda^2
	 * rx = tx +  10 log10 (-------------------)
	 *                       (4 * pi * d)^2 * L
	 *
	 * rx: rx power (dB)
	 * tx: tx power (dB)
	 * d: distance (m)
	 * L: system loss (unit-less)
	 * lambda: wavelength (m)
	 */

	static const double C = 299792458.0; // speed of light in vacuum
	double m_lambda = C / frequency;

	if (distance < 3 * m_lambda)
	{
		NS_LOG_WARN(
			"distance not within the far field region => inaccurate propagation loss value");
	}
	if (distance <= 0)
	{
		return txPowerDbm;
	}
	double numerator = m_lambda * m_lambda;
	double denominator = 16 * M_PI * M_PI * distance * distance;
	double lossDb = -10 * log10(numerator / denominator);

	NS_LOG_DEBUG("distance=" << distance << "m, loss=" << lossDb << "dB");
	return txPowerDbm - lossDb;
}

void
NormalDroneApp::CancelEvents()
{
	std::cout << "*********debut CancelEvents****************************************\n";

	// Cancel any pending events
	Simulator::Cancel(m_sendEvent);
	std::cout << "*********fin CancelEvents****************************************\n";
}

olsr::NeighborSet
NormalDroneApp::GetNeighbors()
{
	return m_state->GetNeighbors();
}

olsr::OlsrState*
NormalDroneApp::GetOlsrState()
{
	std::cout << "*********debut GetOlsrState****************************************\n";

	Ptr<Ipv4RoutingProtocol> ipv4Routing = GetNode()->GetObject<Ipv4>()->GetRoutingProtocol();
	Ptr<Ipv4ListRouting> ipv4ListRouting = DynamicCast<Ipv4ListRouting>(ipv4Routing);
	Ptr<olsr::RoutingProtocol> olsrProtocol;
	int16_t priority;
	for (uint32_t i = 0; i < ipv4ListRouting->GetNRoutingProtocols(); i++)
	{
		Ptr<Ipv4RoutingProtocol> proto = ipv4ListRouting->GetRoutingProtocol(i, priority);
		olsrProtocol = DynamicCast<olsr::RoutingProtocol>(proto);
		if (olsrProtocol)
		{
			break; // found the protocol we are looking for
		}
	}
	std::cout << "*********fin GetOlsrState****************************************\n";

	return &(olsrProtocol->m_state);
}

/*

//wifi start app
 void
  NormalDroneApp::StartApplication ()
  {

	std::cout<<"*********debut StartApplication****************************************\n";

	// Specific start up code goes here.
	if(!m_socket)
	{
	  m_socket = Socket::CreateSocket (GetNode (), m_tid);
	  Ipv4Address ipAddr = GetNode ()->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
	  std::cout<<"   node numero = : "<< GetNode ()<<" adress= "<<ipAddr<<"\n";

	  m_socket->Bind (InetSocketAddress (ipAddr, 89));
	  // m_socket->ShutdownRecv ();
	  m_socket->SetRecvCallback (MakeCallback (&NormalDroneApp::RecvPacket, this));
	}

	if(m_state == 0)
	{
	  m_state = GetOlsrState();
	}

	CancelEvents ();
	 double jitter = m_jitter->GetValue ();
	 std::cout<<" m_jitter->GetValue () = : "<<jitter<<"\n";

	m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter), &NormalDroneApp::SendPackets,
this);

	std::cout<<"*********fin StartApplication****************************************\n";
}
*/

/*
  //wifi stop app
void
NormalDroneApp::StopApplication ()
{
	std::cout<<"*********debut StopApplication****************************************\n";

  // Specific stopping code goes here.
  CancelEvents ();
  if(m_socket != 0)
  {
	m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
	m_socket->Close ();
  }
  if(m_state != 0)
  {
	m_state = 0;
  }

	std::cout<<"*********fin StopApplication****************************************\n";

}

*/

/*
//wifi sendpacket
  void
  NormalDroneApp::SendPackets ()
  {

	std::cout<<"*********debut SendPackets****************************************\n";
	std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";
	const olsr::NeighborSet neighbors = GetNeighbors ();
	Ipv4Address ipAddr = GetNode ()->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();

	std::cout<<"  le noeud = "<<ipAddr<<" envoie un msg aux noeuds : \n";
	for( auto neighborIter = neighbors.begin (); neighborIter < neighbors.end (); neighborIter++)
	{
	  Ipv4Address neighborAddress = neighborIter->neighborMainAddr;
	  std::cout<<"     --> node : " << neighborAddress<<"\n" ;


	  // preparation des donnees a envoyer
	  uint8_t* data = new uint8_t[m_pktSize];
	  for (uint32_t i = 0; i < m_pktSize; ++i) data[i] = 0;


	  if (Simulator::Now ().GetSeconds ()>4.0){
		data[0]=2;// dans le 1er octet 0x02
		data[1]=0; //  dans le 2eme octet 0x0f

	  }else{
		data[0]=0;// dans le 1er octet 0x02
		data[1]=15; //  dans le 2eme octet 0x0f

	  }




	  //creer le packet
	  Ptr<Packet> packet = Create<Packet> (data, m_pktSize);

	  std::cout<<"          nombre de paquets envoyes avant : "<< m_sent<<"\n";

	  //envoyer le packet
	  if(m_socket->SendTo (packet, 0, InetSocketAddress (neighborAddress, 89)) >= 0)
	  {
		m_sent++;
		std::cout<<"          nombre de paquets envoyes apres : "<< m_sent<<"\n";
		m_txPacketTrace (packet, neighborAddress);
	  }
	}

	ScheduleNextTx (0);
	// Logic for sending packets to neighbors

	std::cout<<"*********fin SendPackets****************************************\n";

  }
*/

/*

  void
  NormalDroneApp::RecvPacket (Ptr<Socket> socket)
  {
	  std::cout<<"*********debut RecvPacket****************************************\n";

	Ptr<Packet> packet;
	Address addr;
	while((packet = socket->RecvFrom (addr)))
	{
	  Ipv4Address neighborAddress = InetSocketAddress::ConvertFrom (addr).GetIpv4 ();
	  std::cout << "       address received ="<< neighborAddress<<"\n";

	  uint32_t recvSize = packet->GetSize ();
	  uint32_t *buffer = new uint32_t [recvSize];
	  packet->CopyData((uint8_t *)buffer, recvSize);

	  //pktMsg contient seulement les 4 1ere octet du data
	  uint32_t pktMsg;
	  std::cout << "       buffer[0] received  ="<< buffer[0]<<"\n";

	  // convertir les 4 1er octets : bin to decimal
	  pktMsg = (buffer[3] << 24) + (buffer[2] << 16) + (buffer[1] << 8) + buffer[0];//22= 1000000010
	  std::cout << "       pktMsg received  ="<< pktMsg<<"\n";





	  if (buffer[0]==2){
		  std::cout << "  -----------------------buffer = 2 changement de
  mobility------------------------------\n";



			  MobilityHelper mobility;
			  Ptr<Node> node   = GetNode();

			  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
			  mobility.Install (node);

			  int speed=0;

			  Ptr<ConstantVelocityMobilityModel> mob=
  node->GetObject<ConstantVelocityMobilityModel>(); mob->SetVelocity(Vector(speed, 0, 0));

	  }

	  //

	  m_received++;
	  m_rxPacketTrace (packet, neighborAddress);
	}


	  std::cout<<"*********fin  RecvPacket****************************************\n";

  }
  */
} // namespace ns3
