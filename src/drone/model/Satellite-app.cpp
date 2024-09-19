/*
 * normal-drone-app.cpp
 *
 *  Created on: September 2024
 *      Author: Tahar_Final
 */

#include "Satellite-app.h"

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

NS_LOG_COMPONENT_DEFINE("SatelliteApp");

NS_OBJECT_ENSURE_REGISTERED(SatelliteApp);

TypeId
SatelliteApp::GetTypeId(void)
{
	static TypeId tid =
		TypeId("ns3::SatelliteApp")
			.SetParent<Application>()
			.SetGroupName("Applications")
			.AddConstructor<SatelliteApp>()
			.AddAttribute("PacketSize",
						  "The size of the SatelliteApp packet sent to neighbors.",
						  UintegerValue(4),
						  MakeUintegerAccessor(&SatelliteApp::m_pktSize),
						  MakeUintegerChecker<uint32_t>(1))
			.AddAttribute("TxPower",
						  "The power of the SatelliteApp.",
						  DoubleValue(57.0),
						  MakeDoubleAccessor(&SatelliteApp::m_txPower),
						  MakeDoubleChecker<double>(1))
			.AddAttribute("PRN",
						  "The PRN code  of the SatelliteApp.",
						  UintegerValue(224443546),
						  MakeUintegerAccessor(&SatelliteApp::m_PRN),
						  MakeUintegerChecker<uint32_t>(1))

			.AddAttribute("PacketFrequency",
						  "The time interval of sending a single packet.",
						  TimeValue(Seconds(3)),
						  MakeTimeAccessor(&SatelliteApp::m_pktFreq),
						  MakeTimeChecker())
			.AddAttribute("Protocol",
						  "The type of protocol to use. (e.g. UdpSocketFactory)",
						  TypeIdValue(UdpSocketFactory::GetTypeId()),
						  MakeTypeIdAccessor(&SatelliteApp::m_tid),
						  MakeTypeIdChecker())
			.AddAttribute("JITTER",
						  "The UniformRandomVariable used to create jitter when starting.",
						  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
						  MakePointerAccessor(&SatelliteApp::m_jitter),
						  MakePointerChecker<RandomVariableStream>())
			.AddAttribute("Mob",
						  "Random mobility pointer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&SatelliteApp::m_randomMobility),
						  MakePointerChecker<RandomWalk2dMobilityModel>())
			.AddAttribute("Phy",
						  "Physic layer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&SatelliteApp::m_phy),
						  MakePointerChecker<LrWpanPhy>())
			.AddAttribute("GPS",
						  "GPS complet device = phy + mac",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&SatelliteApp::m_gps),
						  MakePointerChecker<LrWpanNetDevice>())
			.AddAttribute("Mac",
						  "Physic layer",
						  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
						  MakePointerAccessor(&SatelliteApp::m_mac),
						  MakePointerChecker<LrWpanMac>())
			
			.AddAttribute("SatelliteNumber",
						  "Sent NormalDroneApp avoidance packet.",
						  CallbackValue(),
						  MakeCallbackAccessor(&SatelliteApp::SatelliteNumber),
						  MakeCallbackChecker())

			.AddTraceSource("Rx",
							"Received SatelliteApp avoidance packet.",
							MakeTraceSourceAccessor(&SatelliteApp::m_rxPacketTrace),
							"ns3::SatelliteApp::SatelliteAppPacketRxTracedCallback")

			.AddTraceSource("Tx",
							"Sent SatelliteApp avoidance packet.",
							MakeTraceSourceAccessor(&SatelliteApp::m_txPacketTrace),
							"ns3::SatelliteApp::SatelliteAppPacketTxTracedCallback");

	return tid;
}

SatelliteApp::SatelliteApp()
	: m_state(0),
	  m_socket(0),
	  m_sent(0),
	  m_received(0)
{
	NS_LOG_FUNCTION(this);
	// m_state = GetOlsrState ();
}

SatelliteApp::~SatelliteApp()
{
	NS_LOG_FUNCTION(this);
}

void
SatelliteApp::DoDispose(void)
{
	NS_LOG_FUNCTION(this);

	// Do any cleaning up here.

	Application::DoDispose();
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/************************************* Methodes d'indications
 * ******************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************************CallBack**********************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
PlmeEdConfirm(SatelliteApp* App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
{
}

void
PlmeEdConfirm_V2(SatelliteApp* App,
				 Ptr<Node> drone,
				 LrWpanPhyEnumeration status,
				 uint8_t level,
				 double ratio,
				 double m_InterferenceAvg,
				 double MyPRN)
{
	//	std::cout<<"\n______________________PlmeEdConfirm_V2
	//(SatelliteApp)___________________________________\n";
}

static void
DataConfirm(McpsDataConfirmParams params)
{
	// std::cout << "Start : DataConfirm (Satellite)" << std::endl;
	// std::cout << "Finish : DataConfirm (Satellite)" << std::endl;

	//	std::cout<<"_____________________DataConfirm (SatelliteApp)
	//____________________________________\n ";
}

void
DataIndication(SatelliteApp* App,
			   Ptr<Node> drone,
			   Ptr<RandomWalk2dMobilityModel> RandomMob,
			   McpsDataIndicationParams params,
			   Ptr<Packet> p)
{
	// std::cout << "Start : DataIndication (Satellite)" << std::endl;
	// std::cout << "Finish : DataIndication (Satellite)" << std::endl;

	// std::cout<<"_____________________DataIndication (SatelliteApp)
	// ____________________________________\n ";

	//   NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************** Methodes standard d'application
 * ********************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
SatelliteApp::StartApplication()
{
	std::cout << "Start : StartApplication (Satellite)" << std::endl;

	g_received = 0;
	// pas de callback car le satellite ne repond pas
	Simulator::Schedule(Seconds(0.0), &SatelliteApp::InstallCallBackLrwpan, this);

	double jitter = m_jitter->GetValue();
	std::cout << "Time = : " << Simulator::Now() << "\n";
	std::cout << "m_pktFreq= : " << m_pktFreq << "\n";
	std::cout << "m_PRN : " << m_PRN << "\n";
	// std::cout << " m_jitter->GetValue () = : " << jitter << "\n";

	m_sendEvent = Simulator::Schedule(Seconds(0), &SatelliteApp::SendPackets, this);
	// m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter), &SatelliteApp::SendPackets,
	// this);

	std::cout << "Finish : StartApplication (Satellite)" << std::endl;
}

void
SatelliteApp::StopApplication()
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
SatelliteApp::InstallCallBackLrwpan()
{
	McpsDataConfirmCallback cb0;
	cb0 = MakeCallback(&DataConfirm);

	McpsDataIndicationCallback cb1;
	cb1 = MakeBoundCallback(&DataIndication, this, GetNode(), m_randomMobility);

	PlmeEdConfirmCallback cb2;
	cb2 = MakeBoundCallback(&PlmeEdConfirm, this, GetNode());

	PlmeEdConfirmCallback_V2 cb2V2;
	cb2V2 = MakeBoundCallback(&PlmeEdConfirm_V2, this, GetNode());

	m_mac->SetMcpsDataIndicationCallback(cb1);
	m_mac->SetMcpsDataConfirmCallback(cb0);

	m_phy->SetPlmeEdConfirmCallback(cb2);
	m_phy->SetPlmeEdConfirmCallback_V2(cb2V2);
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
SatelliteApp::AppLayer(int type,
					   McpsDataIndicationParams params,
					   Ptr<LrWpanNetDevice> MoiDevReceiver,
					   Ptr<RandomWalk2dMobilityModel> RandomMob,
					   Ptr<Node> drone,
					   Ptr<Packet> p,
					   DronParams0 PacketsInfo)
{
	std::cout << "Start : AppLayer (SatelliteApp)" << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
	g_received++;

	std::cout << "g_received =" << g_received << " \n";

	if (g_received == 2)
	{
		// Simulator::Schedule (Seconds (0.0), &SatelliteApp::JumpToTheMoon, this,MoiDevReceiver,
		// RandomMob,drone);
	}

	switch (type)
	{
	case 0: // ACK
		// Simulator::Schedule (Seconds (0.0), &SatelliteApp::SendAck, this, params,MoiDevReceiver);

		break;
	case 1: // Packet

		break;
	case 2: // Jump to the moon
	/*
		Simulator::Schedule(Seconds(0.0),
							&SatelliteApp::JumpToTheMoon,
							this,
							MoiDevReceiver,
							RandomMob,
							drone);
	*/

		break;
	case 3: // constant mobility to random walk 2d

		break;
	case 4:

		break;
	}

	std::cout << "Finish : AppLayer (SatelliteApp)" << std::endl;
}

void
SatelliteApp::SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver)
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
SatelliteApp::JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver,
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

void
SatelliteApp::SendPackets()
{
	SatelliteNumber(0);
	std::cout << "Start : SendPackets (Satellite)" << std::endl;
	std::cout << "time :" << Simulator::Now().GetSeconds()<< std::endl;

	/****************************recuperation du current
	 * device*********************************************/
	Ptr<LrWpanNetDevice> dev;// = CreateObject<LrWpanNetDevice>();
	dev = GetNode()->GetLrWpanNetDevice(0);
	/****************************recuperation de l'adress du
	 * sender*********************************************/
	Mac16Address Address = dev->GetMac()->GetShortAddress();
	uint8_t* Add = new uint8_t[2];
	Address.CopyTo(Add);
	
/*
	std::cout << "Satellite : " << Address << std::endl;
	std::cout << "Satellite : " << dev->GetAddress() << std::endl;
	std::cout<<"Add :" << ((uint8_t)Add[1]&15) << " / " << ((Add[1]>>4)&15) << std::endl;
	std::cout << "Satellite : " << dev->IsBroadcast() << std::endl;
	std::cout << "Satellite : " << dev->GetBroadcast() << std::endl;
*/

	/************************recuperation de la position du sender*******************************/
	Vector position;
	Ptr<MobilityModel> mobility = dev->GetPhy()->GetMobility();
	position = mobility->GetPosition();
	// std::cout<<"Position : |" << std::to_string(position.x) + "|" + std::to_string(position.z) + "|" + std::to_string(position.z) + "|" << std::endl;

	/********************************affichage des informations *********************************/
	// std::cout<<"	je suis le satellite ["<<Address<<"] ,  ma vrai position  est : x :" <<
	// position.x<<" , y= "<<position.y<<" , z = "<<position.z<<"\n\n";

	if (m_txPower == 58.6 || m_txPower == 57.0)
	{
		// on ne change pas la puissance, car c un legitime satellite
	}
	else
	{
		m_txPower = m_txPower + 1;

		// je change la puissance du spoofing
		LrWpanSpectrumValueHelper svh;

		Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(m_txPower, 11);

		// affectation de la puisance + channnel
		dev->GetMac()->GetPhy()->SetTxPowerSpectralDensity(psd);
	}

	// std::cout<<"satellite power :" << m_txPower<<"\n";

	// std::cout<<" \n 	Signal GPS :  \n";

	McpsDataRequestParams params;
	params.m_srcAddrMode = SHORT_ADDR;
	params.m_dstAddrMode = SHORT_ADDR;
	params.m_dstPanId = 0;
	params.m_dstAddr = Mac16Address("ff:ff"); // Address0; // Mac16Address ("00:02");
	// params.m_dstAddr = Mac16Address("FF:FF"); // Address0; // Mac16Address ("00:02");
	params.m_msduHandle = 0;
	params.m_txOptions = 0;

	Ptr<Packet> p;

	double time_MicroSeconde = Simulator::Now().GetSeconds() * 1000000;
	// preparation des donnees a envoyer (first 3 subframes durant 30 seconde)
	uint32_t time = (uint32_t)time_MicroSeconde; // time en miliseconde

	uint32_t x = (uint32_t)position.x;
	uint32_t y = (uint32_t)position.y;
	uint32_t z = (uint32_t)position.z;

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

	/*
	 *
		uint32_t power= (uint32_t)m_txPower	;

		data[16]= (uint8_t) (power )&255;
		data[17]= (uint8_t) (power >> 8)&255;
		data[18]= (uint8_t) (power >> 16)&255;
		data[19]= (uint8_t) (power >> 24)&255;
	*/

	// envoyer l'adresse du satellite a la place du Tx power
	data[16] = (uint8_t)5;
	data[17] = (uint8_t)5;
	data[18] = (uint8_t)(Add[0]) & 15;
	data[19] = (uint8_t)(Add[0] >> 4) & 15;

	// std::cout<<"Add :" << ((uint8_t)(Add[0]) & 15) << " / " << ((Add[0] >> 4) & 15) << std::endl;

	/***************************************************************/

	data[20] = (uint8_t)(m_PRN) & 255;
	data[21] = (uint8_t)(m_PRN >> 8) & 255;
	data[22] = (uint8_t)(m_PRN >> 16) & 255;
	data[23] = (uint8_t)(m_PRN >> 24) & 255;

	/***************************************************************/

	p = Create<Packet>(data, m_pktSize);

	Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, dev->GetMac(), params, p);
	ScheduleNextTx(0.01);
	
	// std::cout<<"***********************************************************************\n";
	// std::cout<<"**********************fin SendPackets*********************************\n";
	// std::cout<<"***********************************************************************\n\n";

	// std::cout << "Finish : SendPackets (Satellite)" << std::endl;
}

void
SatelliteApp::ScheduleNextTx(double time)
{
	// std::cout<<"*********debut ScheduleNextTx****************************************\n";

	m_sendEvent = Simulator::Schedule(Seconds(time), &SatelliteApp::SendPackets, this);

	// std::cout<<"*********fin ScheduleNextTx****************************************\n";
}

std::string
SatelliteApp::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
	NS_LOG_INFO("Tahar_Final debut + fin WalkBounds");

	return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" +
		   std::to_string(maxY);
}

double
SatelliteApp::Distance(Vector position1, Vector position2)
{
	double distance = 0;

	distance = sqrt((position1.x - position2.x) * (position1.x - position2.x) +
					(position1.y - position2.y) * (position1.y - position2.y));

	return distance;
}

void
SatelliteApp::CancelEvents()
{
	std::cout << "*********debut CancelEvents****************************************\n";

	// Cancel any pending events
	Simulator::Cancel(m_sendEvent);
	std::cout << "*********fin CancelEvents****************************************\n";
}

olsr::NeighborSet
SatelliteApp::GetNeighbors()
{
	return m_state->GetNeighbors();
}

olsr::OlsrState*
SatelliteApp::GetOlsrState()
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
  SatelliteApp::StartApplication ()
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
	  m_socket->SetRecvCallback (MakeCallback (&SatelliteApp::RecvPacket, this));
	}

	if(m_state == 0)
	{
	  m_state = GetOlsrState();
	}

	CancelEvents ();
	 double jitter = m_jitter->GetValue ();
	 std::cout<<" m_jitter->GetValue () = : "<<jitter<<"\n";

	m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter), &SatelliteApp::SendPackets,
this);

	std::cout<<"*********fin StartApplication****************************************\n";
}
*/

/*
//wifi stop app
void
SatelliteApp::StopApplication ()
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
  SatelliteApp::SendPackets ()
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
  SatelliteApp::RecvPacket (Ptr<Socket> socket)
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
