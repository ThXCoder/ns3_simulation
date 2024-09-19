/*
 * Malicious.cpp
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#include "Malicious.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/olsr-state.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/wifi-module.h"
#include "ns3/seq-ts-header.h"
#include "ns3/mobility-module.h"

#include <ns3/mac16-address.h>
// #include <ns3/lr-wpan-mac.h>
#include <ns3/lr-wpan-module.h>

#include "ns3/mobility-module.h"

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE("Malicious");

	NS_OBJECT_ENSURE_REGISTERED(Malicious);

	TypeId
	Malicious::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::Malicious")
								.SetParent<Application>()
								.SetGroupName("Applications")
								.AddConstructor<Malicious>()
								.AddAttribute("PacketSize", "The size of the malicious packet sent to neighbors.",
											  UintegerValue(4),
											  MakeUintegerAccessor(&Malicious::m_pktSize),
											  MakeUintegerChecker<uint32_t>(1))
								.AddAttribute("PacketFrequency", "The time interval of sending a single packet.",
											  TimeValue(Seconds(3)),
											  MakeTimeAccessor(&Malicious::m_pktFreq),
											  MakeTimeChecker())
								.AddAttribute("Protocol", "The type of protocol to use. (e.g. UdpSocketFactory)",
											  TypeIdValue(UdpSocketFactory::GetTypeId()),
											  MakeTypeIdAccessor(&Malicious::m_tid),
											  MakeTypeIdChecker())
								.AddAttribute("JITTER", "The UniformRandomVariable used to create jitter when starting.",
											  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
											  MakePointerAccessor(&Malicious::m_jitter),
											  MakePointerChecker<RandomVariableStream>())
								.AddAttribute("Mob", "Random mobility pointer",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_randomMobility),
											  MakePointerChecker<RandomWalk2dMobilityModel>())
								.AddAttribute("Phy", "Physic layer",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_phy),
											  MakePointerChecker<LrWpanPhy>())

								.AddAttribute("Mac", "Physic layer",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_mac),
											  MakePointerChecker<LrWpanMac>())

								.AddAttribute("GPS_PRN0", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN0),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN1", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN1),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN2", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN2),
											  MakePointerChecker<LrWpanNetDevice>())
								.AddAttribute("GPS_PRN3", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN3),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN4", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN4),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN5", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN5),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN6", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN6),
											  MakePointerChecker<LrWpanNetDevice>())
								.AddAttribute("GPS_PRN7", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN7),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN8", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN8),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN9", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN9),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("GPS_PRN10", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN10),
											  MakePointerChecker<LrWpanNetDevice>())
								.AddAttribute("GPS_PRN11", "GPS complet device = phy + mac",
											  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
											  MakePointerAccessor(&Malicious::m_gps_PRN11),
											  MakePointerChecker<LrWpanNetDevice>())

								.AddAttribute("MyPRN", "PRN used by malicious UAV",
											  UintegerValue(4294967295),
											  MakeUintegerAccessor(&Malicious::m_myPRN),
											  MakeUintegerChecker<uint32_t>(1))

								.AddTraceSource("Rx", "Received malicious avoidance packet.",
												MakeTraceSourceAccessor(&Malicious::m_rxPacketTrace),
												"ns3::Malicious::maliciousPacketRxTracedCallback")

								.AddTraceSource("Tx", "Sent malicious avoidance packet.",
												MakeTraceSourceAccessor(&Malicious::m_txPacketTrace),
												"ns3::Malicious::maliciousPacketTxTracedCallback");

		return tid;
	}

	Malicious::Malicious()
		: m_state(0),
		  m_socket(0),
		  m_sent(0),
		  m_received(0)
	{

		NS_LOG_FUNCTION(this);
		// m_state = GetOlsrState ();
	}

	Malicious::~Malicious()
	{

		NS_LOG_FUNCTION(this);
	}

	void
	Malicious::DoDispose(void)
	{
		NS_LOG_FUNCTION(this);

		// Do any cleaning up here.

		// Application::DoDispose();
	}

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/************************************* Methodes d'indications ******************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/********************************************CallBack**********************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	void PlmeEdConfirm(Malicious *App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
	{
	}

	void PlmeEdConfirm_V2(Malicious *App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level, double averagePower, double m_InterferenceAvg, double MyPRN)
	{
		// mousaab std::cout<<"\n_____________________PlmeEdConfirm_V2 (Malicious)___________________________________\n";
		// mousaab std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

		/****************************recuperation du courant device*********************************************/
		Ptr<LrWpanNetDevice> dev = CreateObject<LrWpanNetDevice>();
		dev = drone->GetLrWpanNetDevice(0);

		/****************************recuperation du courant GPS device*********************************************/
		Ptr<LrWpanNetDevice> GPSdev = CreateObject<LrWpanNetDevice>();
		GPSdev = drone->GetLrWpanNetDevice(1);

		/****************************recuperation de l'adress du sender*********************************************/
		Mac16Address Address = dev->GetMac()->GetShortAddress();
		uint8_t *MonAdd = new uint8_t[2];
		Address.CopyTo(MonAdd);
		/***************************** recuperation de ma position****************************************/

		Vector position;
		Ptr<MobilityModel> mobility = dev->GetPhy()->GetMobility();
		position = mobility->GetPosition();
		// mousaab std::cout<<"	je suis le noeud ["<< Address<<"] ma position reel (from IMU) est : x :" << position.x<<" , y= "<<position.y<<" , z= "<<position.z<<"    \n\n";

		App->m_LastEnergy = static_cast<uint32_t>(level);

		double m_rxSensitivity = pow(10.0, -106.58 / 10.0) / 1000.0;

		double ratio = 10.0 * log10(averagePower / m_rxSensitivity);
		App->m_LastRatio = ratio; // -

		// mousaab std::cout<<"	Energy Detection completed with status:  " <<status<<" : "<< LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " and energy level " << static_cast<uint32_t> (level)<<" and ratio ns3= "<<static_cast<double> (ratio)<<"\n";
		// mousaab std::cout<<"_________________________________________________________\n\n";
	}

	static void DataConfirm(McpsDataConfirmParams params)
	{
		/*

		std::cout<<"________________________ DataConfirm (Malicious)_________________________________\n ";
		 std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

	   std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

		   std::cout<<"_________________________________________________________\n ";
   */
	}

	void DataIndication(Malicious *App, Ptr<Node> drone, Ptr<RandomWalk2dMobilityModel> RandomMob, McpsDataIndicationParams params, Ptr<Packet> p)
	{
		std::cout << "_____________________DataIndication (Malicious) ____________________________________\n ";

		std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

		/****************************recuperation de mon device*********************************************/
		Ptr<LrWpanNetDevice> MoiDevReceiver = CreateObject<LrWpanNetDevice>();
		Ptr<LrWpanNetDevice> MoiGPSDevReceiver = CreateObject<LrWpanNetDevice>();

		MoiDevReceiver = drone->GetLrWpanNetDevice(0);
		MoiGPSDevReceiver = drone->GetLrWpanNetDevice(1);

		/****************************recuperation de mon Adresse*********************************************/
		Mac16Address Address = MoiDevReceiver->GetMac()->GetShortAddress();
		uint8_t *MonAdd = new uint8_t[2];
		Address.CopyTo(MonAdd);
		/***************************** recuperation de ma position****************************************/

		Vector ma_position;
		Ptr<MobilityModel> mobility = MoiDevReceiver->GetPhy()->GetMobility();
		ma_position = mobility->GetPosition();
		std::cout << "	je suis le noeud [" << Address << "] ma position ( IMU ) est : x :" << ma_position.x << " , y= " << ma_position.y << "\n\n";

		uint8_t *Add = new uint8_t[2];
		params.m_dstAddr.CopyTo(Add);
		/***************************** affichage du packet****************************************/

		std::cout << "Received Answer of size " << p->GetSize() << "\n";
		uint32_t recvSize = p->GetSize();
		uint8_t *bufferTompo = new uint8_t[recvSize];
		p->CopyData((uint8_t *)bufferTompo, recvSize);

		// std::cout<<"je viens de recevoir un msg de l'UAV i : \n ";
		for (uint32_t i = 0; i < recvSize; i++)
		{
			// printf ("   T [%i]= %u"  ,i,bufferTompo[i]);
		}

		/*************************************** verifier la compatibilite entre la position et l'energie recu***************************************************************************************************************************/

		Vector SenderPosition;
		SenderPosition.x = (bufferTompo[7] << 24) + (bufferTompo[6] << 16) + (bufferTompo[5] << 8) + bufferTompo[4];
		SenderPosition.y = (bufferTompo[11] << 24) + (bufferTompo[10] << 16) + (bufferTompo[9] << 8) + bufferTompo[8];
		SenderPosition.z = (bufferTompo[15] << 24) + (bufferTompo[14] << 16) + (bufferTompo[13] << 8) + bufferTompo[12];

		double distance = App->Distance(ma_position, SenderPosition);
		std::cout << " \n distance = " << distance << "  energy = " << App->m_LastEnergy << "\n";

		/************************************************ savegarder l'energie et la distance dans param******************************************************************************************************************/

		DronParams1 PacketsInfo;
		PacketsInfo.distance = distance;
		PacketsInfo.energy = App->m_LastEnergy;

		App->m_DronesTable.push_back(PacketsInfo);

		int isDetectiomMSG = 0;

		if ((bufferTompo[0] == 0) && (bufferTompo[1] == 0) && (bufferTompo[2] == 0) && (bufferTompo[3] == 0))
		{
			isDetectiomMSG = 1;
			std::cout << "_____________________________________________________________\n";
			std::cout << "_______________________ mmm detection msg fa9o biya_______________________\n";
			std::cout << "_____________________________________________________________\n";
		}
		else
		{
		}

		/******************************************************************************************************************************************************************/
		// appel le gps spoofing pour envoyer un packet falsifier

		if (isDetectiomMSG == 1)
		{
		}
		else
		{
			App->AppLayer(0, params, MoiGPSDevReceiver, RandomMob, drone, p, PacketsInfo);
		}

		std::cout << "_____________________________________________________________\n";

		//   NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
	}

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/********************************************    GPS CallBack   *********************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	void PlmeEdConfirmGPS(Malicious *App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
	{
	}

	void PlmeEdConfirmGPS_V2(Malicious *App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level, double averagePower, double interference, double myPRN)
	{
		// mousaab std::cout<<"\n______________________PlmeEdConfirmGPS_V2  (Malicious)___________________________________\n";
		// mousaab std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";
		uint32_t authenticPRN[] = {4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755, 2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210, 3275539260, 2523502185, 92934875, 923840985, 90283402, 54324234, 542489234, 549024, 1212, 44359, 70398234, 3239, 834873258, 60324892, 28594359, 349684609, 9999122, 224443546};

		/****************************recuperation de l'id de l'uav courant  device  N =1 *********************************************/
		Ptr<LrWpanNetDevice> MoiDev = CreateObject<LrWpanNetDevice>();
		MoiDev = drone->GetLrWpanNetDevice(0);
		// recuperer mon adresse
		Mac16Address Address = MoiDev->GetMac()->GetShortAddress();
		uint8_t *MonAdd = new uint8_t[2];
		Address.CopyTo(MonAdd);

		/****************************recuperation du courant GPS device  N =1 *********************************************/
		Ptr<LrWpanNetDevice> GPSdev = CreateObject<LrWpanNetDevice>();
		GPSdev = drone->GetLrWpanNetDevice(1);

		Vector position;
		Ptr<MobilityModel> mobility = GPSdev->GetPhy()->GetMobility();
		position = mobility->GetPosition();

		// mousaab 
		std::cout<<"	 je suis l'uav : ["<< Address <<"] ma position IMU est : x :" << position.x<<" , y= "<<position.y<< ", z = "<< position.z<< "\n\n";

		App->m_LastEnergy = static_cast<uint32_t>(level);
		App->m_interferenceNoise = 10.0 * log10(interference);

		double AbsolutPower = 10.0 * log10(averagePower);

		// cas de Dynamic CN0  non fixe

		double CN0;
		for (int i = 0; i < 12; i++)
		{
			if (myPRN == authenticPRN[i])
			{

				App->m_AbsolutGpsPower[i] = static_cast<double>(AbsolutPower - 2); // -2db : a cause de l'attenuation atmospherique
				CN0 = (App->m_AbsolutGpsPower[i]) - (App->m_interferenceNoise);
				App->m_CN0[i] = CN0;
			}
		}

		// cas de C/N0 fixe
		//	double K_boltzmann = 1.3806 *  pow (10.0, -23) ;
		//	double Temperature=300;
		//	double noise1 =  10.0 * log10 (K_boltzmann*Temperature);

		// nab3eth seuelemnt dans le cas des drone legitimes
		// App->m_rxPacketTrace (App->m_AbsolutGpsPower, App->m_CN0);

		// mousaab std::cout<<"	Energy Detection completed with status:  " <<status<<" : "<< LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " and energy level " << static_cast<uint32_t> (level)<<" and puissance Recu  = "<<App->m_AbsolutGpsPower<<"  ,  Fixenoise= "<<noise1<<"  , FixeCN0 = "<<FixeCN0<<"  dynamic noise = "<<	App->m_interferenceNoise<<"   Dynamic CN0 = "<<App->m_CN0<<" \n";
		// mousaab std::cout<<"_________________________________________________________\n\n";
	}

	static void DataConfirmGPS(McpsDataConfirmParams params)
	{
		// std::cout<<"_____________________DataConfirmGPS (Malicious) ____________________________________\n ";

		/*

		std::cout<<"________________________ DataConfirm (Malicious)_________________________________\n ";
		 std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

	   std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

		   std::cout<<"_________________________________________________________\n ";
	*/
	}

	void DataIndicationGPS(Malicious *App, Ptr<Node> drone, Ptr<RandomWalk2dMobilityModel> RandomMob, McpsDataIndicationParams params, Ptr<Packet> p)
	{
		// mousaab std::cout<<"_____________________DataIndicationGPS (Malicious) ____________________________________\n ";

		// mousaab std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

		// std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

		/****************************recuperation de mon device*********************************************/
		Ptr<LrWpanNetDevice> MoiDevReceiver = CreateObject<LrWpanNetDevice>();
		MoiDevReceiver = drone->GetLrWpanNetDevice(0);

		/****************************recuperation de l'adress du sender*********************************************/
		Mac16Address Address = MoiDevReceiver->GetMac()->GetShortAddress();
		uint8_t *MonAdd = new uint8_t[2];
		Address.CopyTo(MonAdd);
		/***************************** recuperation de ma position****************************************/

		Vector position;
		Ptr<MobilityModel> mobility = MoiDevReceiver->GetPhy()->GetMobility();
		position = mobility->GetPosition();
		// mousaab std::cout<<"			je suis le noeud ["<< Address<<"] ma position ( IMU ) est : x :" << position.x<<" , y= "<<position.y<< ", z= "<<position.z<<"\n\n";

		/***************************** affichage du packet****************************************/

		uint8_t *Add = new uint8_t[2];
		params.m_dstAddr.CopyTo(Add);
		// std::cout<<"Received Answer of size " << p->GetSize ()<<"\n";
		uint32_t recvSize = p->GetSize();
		uint8_t *bufferTompo = new uint8_t[recvSize];
		p->CopyData((uint8_t *)bufferTompo, recvSize);

		// mousaab  std::cout<<"je viens de recevoir un msg de l'UAV i : \n ";

		for (uint32_t i = 0; i < recvSize; i++)
		{
			//  printf ("bufferTompo [%i]= %u\n",i,bufferTompo[i]);
		}

		// uint32_t timeGPS = (bufferTompo[3]<<24)+(bufferTompo[2]<<16)+(bufferTompo[1]<<8)+bufferTompo[0];
		// double timeGPS_MicroSeconde = timeGPS/1000000.0;

		// std::cout<<"\n			time of satellite =  " << timeGPS<<" miliseconde\n";
		// std::cout<<"			time of satellite =  " << timeGPS_MicroSeconde<<" seconde \n";

		/*************************************** verifier la compatibilite entre la position et l'energie recu***************************************************************************************************************************/

		Vector SenderPosition;
		SenderPosition.x = (bufferTompo[7] << 24) + (bufferTompo[6] << 16) + (bufferTompo[5] << 8) + bufferTompo[4];
		SenderPosition.y = (bufferTompo[11] << 24) + (bufferTompo[10] << 16) + (bufferTompo[9] << 8) + bufferTompo[8];
		SenderPosition.z = (bufferTompo[15] << 24) + (bufferTompo[14] << 16) + (bufferTompo[13] << 8) + bufferTompo[12];

		// mousaab uint32_t SatelliteID_part1 = (uint32_t)bufferTompo[18]&255;
		// uint32_t SatelliteID_part2 = (uint32_t)bufferTompo[19]&255;

		// mousaab std::cout<<" 			satellite ID =  " <<  SatelliteID_part2<< ""<<SatelliteID_part1<<":00 \n";

		double distance = App->Distance(position, SenderPosition);
		// double RxPower = App->DoCalcRxPower(26.8,distance,1575420000);
		//				App->m_LastRatio= static_cast<double> (RxPower-2); // -2db : a cause de l'attenuation atmospherique

		// std::cout<<"\n 			Satellite position     x =  " << SenderPosition.x<<" , y =  " << SenderPosition.y<< ", z =  " << SenderPosition.z<<" m \n";
		// std::cout<<" \n 			distance = " << distance<<"... LrWpan energy = "<<App->m_LastEnergy<<   "... C/N0 = " << App->m_FixeCN0<<"... Received power= "<<App->m_AbsolutGpsPower<<"\n";

		DronParams1 PacketsInfo;

		PacketsInfo.distance = distance;

		PacketsInfo.energy = App->m_LastEnergy;
		App->m_DronesTable.push_back(PacketsInfo);
	}

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/********************************** Methodes standard d'application ********************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	void
	Malicious::StartApplication()
	{
		std::cout << "\n\n*********debut StartApplication  ( malicious )****************************************\n";
		g_received = 0;
		Simulator::Schedule(Seconds(0.0), &Malicious::InstallCallBackLrwpan, this);
		std::cout << " m_pktFreq= : " << m_pktFreq << "\n";

		double jitter = m_jitter->GetValue();
		std::cout << " m_jitter->GetValue () = : " << jitter << "\n";

		// m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter), &Malicious::SendGpsSpoofingSignal, this);

		std::cout << "*********fin StartApplication****************************************\n";
	}

	void
	Malicious::StopApplication()
	{
		std::cout << "*********debut StopApplication****************************************\n";

		CancelEvents();

		std::cout << "*********fin StopApplication****************************************\n";
	}

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/********************************** Methodes supplementaires d'application ********************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	void
	Malicious::InstallCallBackLrwpan()
	{
 		std::cout << "*** Start : InstallCallBackLrwpan (Malicious) ***" << std::endl;
		McpsDataConfirmCallback cb0;
		McpsDataIndicationCallback cb1;
		PlmeEdConfirmCallback cb2;
		PlmeEdConfirmCallback_V2 cb2V2;
		cb0 = MakeCallback(&DataConfirm);

		cb1 = MakeBoundCallback(&DataIndication, this, GetNode(), m_randomMobility);

		cb2 = MakeBoundCallback(&PlmeEdConfirm, this, GetNode());

		cb2V2 = MakeBoundCallback(&PlmeEdConfirm_V2, this, GetNode());

		m_mac->SetMcpsDataConfirmCallback(cb0);
		m_mac->SetMcpsDataIndicationCallback(cb1);

		m_phy->SetPlmeEdConfirmCallback(cb2);
		m_phy->SetPlmeEdConfirmCallback_V2(cb2V2);

		McpsDataConfirmCallback GPScb0;
		McpsDataIndicationCallback GPScb1;
		PlmeEdConfirmCallback GPScb2;
		PlmeEdConfirmCallback_V2 GPScb2V2;

		GPScb0 = MakeCallback(&DataConfirmGPS);

		GPScb1 = MakeBoundCallback(&DataIndicationGPS, this, GetNode(), m_randomMobility);

		GPScb2 = MakeBoundCallback(&PlmeEdConfirmGPS, this, GetNode());

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

 		std::cout << "*** Finish : InstallCallBackLrwpan (Malicious) ***" << std::endl;

	}

	/******************************************************************************************************************************************************************/
	/******************************************************************************************************************************************************************/

	void
	Malicious::AppLayer(int type, McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> drone, Ptr<Packet> p, DronParams1 PacketsInfo)
	{
		std::cout << "\n______________________ AppLayer (Malicious)___________________________________\n";
		std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
		g_received++;

		std::cout << "	g_received =" << g_received << " \n";

		switch (type)
		{
		case 0: // spoofing
				// desactiver le recepteur gps afin d'envoyer le le msg spoofing (basculer vers le mode emeteur)
			// MoiDevReceiver->GetPhy()->DisableGPS();

			Simulator::Schedule(Seconds(0.0), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.001), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.002), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.003), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.004), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.005), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);
			Simulator::Schedule(Seconds(0.006), &Malicious::SendGpsSpoofingSignal, this, params, MoiDevReceiver, PacketsInfo);

			break;
		case 1: // Packet

			break;
		case 2: // Jump to the moon
			Simulator::Schedule(Seconds(0.0), &Malicious::JumpToTheMoon, this, MoiDevReceiver, RandomMob, drone);

			break;
		case 3: // constant mobility to random walk 2d

			break;
		case 4:

			break;
		}

		std::cout << "_________________________________________________________\n\n";
	}

	void
	Malicious::SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver)
	{
		std::cout << "\n______________________ SendAck (drone-adhoc)___________________________________\n";
		std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

		uint8_t *Add = new uint8_t[2];
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
		uint8_t *data = new uint8_t[4];
		for (uint32_t i = 0; i < 4; ++i)
			data[i] = 0;

		data[0] = Add[0];
		data[1] = Add[1];

		Answer = Create<Packet>(data, 4);

		Simulator::Schedule(Seconds(0.2),
							&LrWpanMac::McpsDataRequest,
							MoiDevReceiver->GetMac(), paramsAnswer, Answer);

		std::cout << "_________________________________________________________\n\n";
	}

	void
	Malicious::JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> drone)
	{
		std::cout << "\n______________________ ChangeTheMobility (drone-adhoc)___________________________________\n";
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

	void
	Malicious::SendPackets()
	{
		std::cout << "\n\n***********************************************************************\n";
		std::cout << "**********************debut SendPackets (Malicius ) *********************************\n";
		std::cout << "***********************************************************************\n\n";

		std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

		/****************************recuperation du current device*********************************************/
		Ptr<LrWpanNetDevice> dev = CreateObject<LrWpanNetDevice>();
		dev = GetNode()->GetLrWpanNetDevice(0); // 0: carte reseau normal, 1: carte reseau gps receiver

		/****************************recuperation de l'adress du sender*********************************************/
		Mac16Address Address = dev->GetMac()->GetShortAddress();
		uint8_t *Add = new uint8_t[2];
		Address.CopyTo(Add);
		/************************recuperation de la position du sender*******************************/
		Vector position;
		Ptr<MobilityModel> mobility = dev->GetPhy()->GetMobility();
		position = mobility->GetPosition();
		/********************************affichage des informations *********************************/
		std::cout << "	je suis le noeud malveillant [" << Address << "] ma vrai position  est : x :" << position.x << " , y= " << position.y << "\n\n";

		/****************************recuperation des Neighbors devices *********************************************/
		std::vector<Ptr<LrWpanNetDevice>> voisins = GetNode()->GetALLLrWpanNetDevice();
		Ptr<LrWpanNetDevice> devReceiver = CreateObject<LrWpanNetDevice>();

		// choisir le 1er drone comme etant un receveur
		devReceiver = voisins[1];
		/****************************recuperation de l'adress du receiver*********************************************/
		Mac16Address Address0 = devReceiver->GetMac()->GetShortAddress();
		uint8_t *Addr0 = new uint8_t[2];
		Address0.CopyTo(Addr0);

		// std::cout<<" \n 	Msg ennvoye au drone :  ["<< Address0<<"] \n";

		std::cout << " \n 	Msg ennvoye a tt le monde <usurpation d'identite> :  \n";

		McpsDataRequestParams params;
		params.m_srcAddrMode = SHORT_ADDR;
		params.m_dstAddrMode = SHORT_ADDR;
		params.m_dstPanId = 0;
		params.m_dstAddr = Mac16Address("ff:ff"); // Address0; // Mac16Address ("00:02");
		params.m_msduHandle = 0;
		params.m_txOptions = 0;

		Ptr<Packet> p;
		// preparation des donnees a envoyer
		uint8_t *data = new uint8_t[m_pktSize];
		for (uint32_t i = 0; i < m_pktSize; ++i)
			data[i] = 0;

		data[0] = 8;  // Add[0];
		data[1] = 0;  // Add[1];
		data[2] = 10; //(uint8_t)position.x;
		data[3] = 10; //(uint8_t)position.y;

		p = Create<Packet>(data, m_pktSize);

		Simulator::Schedule(Seconds(0),
							&LrWpanMac::McpsDataRequest,
							dev->GetMac(), params, p);
		ScheduleNextTx(0);
		std::cout << "***********************************************************************\n";
		std::cout << "**********************fin SendPackets*********************************\n";
		std::cout << "***********************************************************************\n\n";
	}

	void
	Malicious::SendGpsSpoofingSignal(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> monGPSDevReceiver, DronParams1 PacketsInfo)
	{
		std::cout << "\n\n***********************************************************************\n";
		std::cout << "**********************debut SendPacketsv(GPS Spoofing)*********************************\n";
		std::cout << "***********************************************************************\n\n";

		std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

		/************************recuperation de ma position*******************************/
		Vector MaliciousPosition;
		Ptr<MobilityModel> mobility = monGPSDevReceiver->GetPhy()->GetMobility();
		MaliciousPosition = mobility->GetPosition();

		/****************************recuperation de mon DEV for DATA*********************************************/
		Ptr<LrWpanNetDevice> monDATAdev = CreateObject<LrWpanNetDevice>();
		monDATAdev = GetNode()->GetLrWpanNetDevice(0);

		/****************************recuperation de lmon Adessse for DATA*********************************************/
		Mac16Address Address = monDATAdev->GetMac()->GetShortAddress();
		uint8_t *myAdd = new uint8_t[2];
		Address.CopyTo(myAdd);
		std::cout << "	je suis le noeud malveillant [..." << Address << "..] ma vrai position  est : x :" << MaliciousPosition.x << " , y= " << MaliciousPosition.y << "\n\n";

		/********************************recuperer la distance entre malicious et la victime *********************************/
		std::cout << " \n PacketsInfo.distance = " << PacketsInfo.distance << "  \n";

		double distance = PacketsInfo.distance;
		/********************************calculer la puissance d'emission*********************************/

		static const double C = 299792458.0; // speed of light in vacuum
		double m_lambda = C / 1575420000;

		double numerator = m_lambda * m_lambda;
		double denominator = 16 * M_PI * M_PI * distance * distance;
		double lossDb = -10 * log10(numerator / denominator);
		// std::cout<<"lossDb: " << lossDb <<"\n";
		double SenderPowerDBw = -153.6 + lossDb + 2;
		// double SenderPowerDBw =-157.6 + lossDb +2;
		double SenderPowerDBm = SenderPowerDBw + 30;
		// std::cout<<"powerDBm :" << SenderPowerDBm <<"\n";

		/********************************preparation pour l'envoie*********************************/

		LrWpanSpectrumValueHelper svh;
		Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(SenderPowerDBm, 11);

		// affectation de la puisance + channnel
		monGPSDevReceiver->GetMac()->GetPhy()->SetTxPowerSpectralDensity(psd);

		std::cout << " \n 	Msg ennvoye a tt le monde GPS spoofing :  \n";

		// destination en cas je dois envoyer la reponse a un seul UAV (oriented antenna)
		uint8_t *Add = new uint8_t[2];
		params.m_dstAddr.CopyTo(Add);

		Ptr<Packet> p;

		McpsDataRequestParams paramsAnswer;
		paramsAnswer.m_srcAddrMode = SHORT_ADDR;
		paramsAnswer.m_dstAddrMode = SHORT_ADDR;
		paramsAnswer.m_dstPanId = 0;
		paramsAnswer.m_dstAddr = Mac16Address("ff:ff"); // Mac16Address ("00:02");
		paramsAnswer.m_msduHandle = 0;
		paramsAnswer.m_txOptions = 0;

		/********************************recuperation du temps (spoofing satellite time) ********************************/

		double time_MicroSeconde = Simulator::Now().GetSeconds() * 1000000;
		// preparation des donnees a envoyer (first 3 subframes durant 30 seconde)
		uint32_t time = (uint32_t)time_MicroSeconde; // time en miliseconde

		/********************************recuperation du malicious position ********************************/

		uint32_t x = (uint32_t)MaliciousPosition.x;
		uint32_t y = (uint32_t)MaliciousPosition.y;
		uint32_t z = (uint32_t)MaliciousPosition.z;

		/********************************affectation of DATA********************************/

		uint8_t *data = new uint8_t[m_pktSize];
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

		// cas2 : it send an authentic prn :

		data[20] = (uint8_t)(m_myPRN) & 255;
		data[21] = (uint8_t)(m_myPRN >> 8) & 255;
		data[22] = (uint8_t)(m_myPRN >> 16) & 255;
		data[23] = (uint8_t)(m_myPRN >> 24) & 255;

		/***************************************************************/
		/***************************************************************/
		/*
		//authentic PRN code :
		uint32_t authentic[]= {4294967295, 2863311530, 3435973836,2576980377,4042322160,2779096485,3284386755,2526451350,4278255360,2857740885,3425946675,2573637990,4027576335,2774181210,3275539260};

		//uint32_t authentic[]= {13234425, 24924213, 8432419,1040493,139933,141103037,170400210,20330};
		int unwanted_len =8;
		bool found;
		uint32_t val=0;
		do {
			val= static_cast<uint32_t> (rand()%    (   static_cast<uint32_t> (pow (2,32))-1));
			found= true;

			for (int i = 0; i<unwanted_len;i++){
				if (authentic[i]==val){
					found=false;
				}
			}
		}while (!found);

	*/
		/************************* creation du packet et l'envoyer**************************************/

		p = Create<Packet>(data, m_pktSize);

		Simulator::Schedule(Seconds(0),
							&LrWpanMac::McpsDataRequest,
							monGPSDevReceiver->GetMac(), paramsAnswer, p);

		// Simulator::Schedule (Seconds (0.2), &Malicious::EnableGpsReceiver, this,MoiDevReceiver);

		// ScheduleNextTx (0);
		std::cout << "***********************************************************************\n";
		std::cout << "**********************fin SendPackets*********************************\n";
		std::cout << "***********************************************************************\n\n";
	}

	void
	Malicious::ScheduleNextTx(double type)
	{
		// std::cout<<"*********debut ScheduleNextTx****************************************\n";

		m_sendEvent = Simulator::Schedule(m_pktFreq, &Malicious::SendPackets, this);

		// std::cout<<"*********fin ScheduleNextTx****************************************\n";
	}

	void
	Malicious::EnableGpsReceiver(Ptr<LrWpanNetDevice> MoiDevReceiver)
	{
		// std::cout<<"*********debut ScheduleNextTx****************************************\n";

		MoiDevReceiver->GetPhy()->EnableGPS();

		// std::cout<<"*********fin ScheduleNextTx****************************************\n";
	}

	double
	Malicious::Distance(Vector position1, Vector position2)
	{
		double distance = 0;

		// std::cout<<"\n(x1 :" << position1.x <<"   ,   ";
		// std::cout<<"y1 :" << position1.y <<"   ,   ";
		// std::cout<<"z1 :" << position1.z <<")";

		// std::cout<<"   (x2 :" << position2.x <<"  ,  ";
		// std::cout<<" y2 :" << position2.y <<"  ,  ";
		// std::cout<<" z2 :" << position2.z <<")";

		distance = sqrt((position1.x - position2.x) * (position1.x - position2.x) + (position1.y - position2.y) * (position1.y - position2.y) + (position1.z - position2.z) * (position1.z - position2.z));

		return distance;
	}

	double
	Malicious::DoCalcRxPower(double txPowerDbm, double distance, double frequency)
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
			NS_LOG_WARN("distance not within the far field region => inaccurate propagation loss value");
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

	std::string
	Malicious::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
	{
		NS_LOG_INFO("bada debut + fin WalkBounds");

		return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" + std::to_string(maxY);
	}

	void
	Malicious::CancelEvents()
	{
		std::cout << "*********debut CancelEvents****************************************\n";

		// Cancel any pending events
		Simulator::Cancel(m_sendEvent);
		std::cout << "*********fin CancelEvents****************************************\n";
	}

	olsr::NeighborSet
	Malicious::GetNeighbors()
	{

		return m_state->GetNeighbors();
	}

	olsr::OlsrState *
	Malicious::GetOlsrState()
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
			if (olsrProtocol != nullptr)
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
	  Malicious::StartApplication ()
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
		  m_socket->SetRecvCallback (MakeCallback (&Malicious::RecvPacket, this));
		}

		if(m_state == 0)
		{
		  m_state = GetOlsrState();
		}

		CancelEvents ();
		 double jitter = m_jitter->GetValue ();
		 std::cout<<" m_jitter->GetValue () = : "<<jitter<<"\n";

		m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter), &Malicious::SendPackets, this);

		std::cout<<"*********fin StartApplication****************************************\n";
	}
	*/

	/*
	  //wifi stop app
	void
	Malicious::StopApplication ()
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
	  Malicious::SendPackets ()
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
	  Malicious::RecvPacket (Ptr<Socket> socket)
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
			  std::cout << "  -----------------------buffer = 2 changement de mobility------------------------------\n";



				  MobilityHelper mobility;
				  Ptr<Node> node   = GetNode();

				  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
				  mobility.Install (node);

				  int speed=0;

				  Ptr<ConstantVelocityMobilityModel> mob= node->GetObject<ConstantVelocityMobilityModel>();
				  mob->SetVelocity(Vector(speed, 0, 0));

		  }

		  //

		  m_received++;
		  m_rxPacketTrace (packet, neighborAddress);
		}


		  std::cout<<"*********fin  RecvPacket****************************************\n";

	  }
	  */
}
