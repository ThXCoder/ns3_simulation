/*
 * normal-drone-app.cpp
 *
 *  Created on: May 24, 2018
 *      Author: bada
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
#include <ns3/mac16-address.h>
// #include <ns3/lr-wpan-mac.h>
#include "ns3/mobility-module.h"
#include <ns3/lr-wpan-module.h>

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
    std::cout << "*** Start : DoDispose ***" << std::endl;

    // Application::DoDispose();

    std::cout << "*** Finish : DoDispose ***" << std::endl;
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
PlmeEdConfirm(NormalDroneApp* App, Ptr<Node> drone, LrWpanPhyEnumeration status, uint8_t level)
{
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
    std::cout << "*** Start : PlmeEdConfirm_V2 ***" << std::endl;

    // mousaab std::cout<<"\n______________________PlmeEdConfirm_V2
    // (NormalDroneApp)___________________________________\n"; mousaab std::cout<<"time :" <<
    // Simulator::Now ().GetSeconds ()<<"\n";

    /****************************recuperation du courant
     * device*********************************************/
    Ptr<LrWpanNetDevice> dev;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
    dev = drone->GetLrWpanNetDevice(0);

    /**************w**************recuperation du courant GPS
     * device*********************************************/
    Ptr<LrWpanNetDevice> GPSdev;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
    GPSdev = drone->GetLrWpanNetDevice(1);

    /****************************recuperation de l'adress du
     * sender*********************************************/
    Mac16Address Address = dev->GetMac()->GetShortAddress();
    uint8_t* MonAdd = new uint8_t[2];
    Address.CopyTo(MonAdd);
    /***************************** recuperation de ma
     * position****************************************/

    Vector position;
    Ptr<MobilityModel> mobility = dev->GetPhy()->GetMobility();
    position = mobility->GetPosition();
    // mousaab
    std::cout << "\nje suis le noeud (Normal) .PlmeEdConfirm_V2() : [" << Address
              << "] ma position reel (from IMU) est : x :" << position.x << " , y= " << position.y
              << " , z= " << position.z << "    \n\n";

    App->m_LastEnergy = static_cast<uint32_t>(level);

    double m_rxSensitivity = pow(10.0, -106.58 / 10.0) / 1000.0;

    double ratio = 10.0 * log10(averagePower / m_rxSensitivity);
    App->m_LastRatio = ratio; // -

/*
    std::cout<<"	Energy Detection completed with status:  " << status <<" : "<<
    LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " .... energy level " <<
    (static_cast<uint32_t> (level)) << " .... real received Power (with sebsitivity)="
	<<(static_cast<double> (ratio)) << "\n" ;
*/
    std::cout<<"_________________________________________________________\n\n";

    std::cout << "*** Finish : PlmeEdConfirm_V2 ***" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

static void
DataConfirm(McpsDataConfirmParams params)
{
    std::cout << "*** Start : DataConfirm ***" << std::endl;

    // std::cout<<"_____________________DataConfirm (NormalDroneApp)
    // ____________________________________\n ";

    /*

     std::cout<<"________________________ DataConfirm
    (NormalDroneApp)_________________________________\n "; std::cout<<"time :" << Simulator::Now
    ().GetSeconds ()<<"\n";

    std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

    std::cout<<"_________________________________________________________\n ";
*/
    std::cout << "*** Finish : DataConfirm ***" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
DataIndication(NormalDroneApp* App,
               Ptr<Node> drone,
               Ptr<RandomWalk2dMobilityModel> RandomMob,
               McpsDataIndicationParams params,
               Ptr<Packet> p)
{
    std::cout << "*** Start : DataIndication ***" << std::endl;

    std::cout << "_____________________DataIndication (NormalDroneApp) "
                 "____________________________________\n ";

    std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    /****************************recuperation de mon
     * device*********************************************/
    Ptr<LrWpanNetDevice> MonDevDATA;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
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
    std::cout << "	je suis le noeud [" << Address
              << "] ma position ( IMU ) est : x :" << position.x << " , y= " << position.y
              << " , z= " << position.z << "\n\n";

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
        printf("   T [%i]= %u", i, bufferTompo[i]);
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

    std::cout << " \n sender position   x = " << SenderPosition.x << "  y = " << SenderPosition.y
              << " , z = " << SenderPosition.z << "\n";

    double distance = App->Distance(position, SenderPosition);
    std::cout << " \n distance = " << distance << "  energy = " << App->m_LastEnergy << "\n";
    DronParams0 PacketsInfo;

    PacketsInfo.distance = distance;
    PacketsInfo.energy = App->m_LastEnergy;
    App->m_DronesTable.push_back(PacketsInfo);

    // activer le GPS pour verifier la position
    Ptr<LrWpanNetDevice> MoiGPSDevReceiver;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
    MoiGPSDevReceiver = drone->GetLrWpanNetDevice(1);

    // activer le recepteur gps
    // MoiGPSDevReceiver->GetPhy()->EnableGPS();

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
    std::cout << "_____________________________________________________________\n\n";

    //   NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
    std::cout << "*** Finish : DataIndication ***" << std::endl;
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
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
PlmeEdConfirmGPS_V2(NormalDroneApp* App,
                    Ptr<Node> drone,
                    LrWpanPhyEnumeration status,
                    uint8_t level,
                    double averagePower,
                    double interference,
                    double myPRN)
{
    std::cout << "*** Start : PlmeEdConfirmGPS_V2 ***" << std::endl;

    // std::cout<<"\n______________________PlmeEdConfirmGPS_V2
    // (NormalDroneApp)___________________________________\n"; std::cout<<"time :" << Simulator::Now
    // ().GetSeconds ()<<"\n";
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    /****************************recuperation de l'id de l'uav courant  device  N =1
     * *********************************************/
    Ptr<LrWpanNetDevice> MoiDev;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
    MoiDev = drone->GetLrWpanNetDevice(0);
    // recuperer mon adresse
    Mac16Address Address = MoiDev->GetMac()->GetShortAddress();
    uint8_t* MonAdd = new uint8_t[2];
    Address.CopyTo(MonAdd);

    /****************************recuperation du courant GPS device  N =1
     * *********************************************/
    Ptr<LrWpanNetDevice> GPSdev;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
    GPSdev = drone->GetLrWpanNetDevice(1);

    /***************************** recuperation de ma
     * position****************************************/

    Vector position;
    Ptr<MobilityModel> mobility = GPSdev->GetPhy()->GetMobility();
    position = mobility->GetPosition();

    // std::cout<<"	 je suis l'uav : ["<< Address <<"] ma position IMU est : x :" << position.x<<" ,
    // y= "<<position.y<< ", z = "<< position.z<< "\n\n";

    App->m_LastEnergy = static_cast<uint32_t>(level);
    App->m_interferenceNoise = 10.0 * log10(interference);

    double AbsolutPower = 10.0 * log10(averagePower);

    // cas de Dynamic CN0  non fixe

    // double abspower=  static_cast<double> (AbsolutPower-2);// -2db : a cause de l'attenuation
    // atmospherique
    //  std::cout<<" 			abspower =  " << abspower<< "\n";
    //  std::cout<<" 			PRN from Callback =  " << myPRN<< "\n";

    double CN0;

    for (int i = 0; i < 12; i++)
    {
        if (myPRN == authenticPRN[i])
        {
            App->m_AbsolutGpsPower[i] = static_cast<double>(
                AbsolutPower - 2); // -2db : a cause de l'attenuation atmospherique
            CN0 = (App->m_AbsolutGpsPower[i]) - (App->m_interferenceNoise);
            App->m_CN0[i] = CN0;
        }
    }

    // cas de C/N0 fixe
    //	double K_boltzmann = 1.3806 *  pow (10.0, -23) ;
    //	double Temperature=300;
    //	double noise1 =  10.0 * log10 (K_boltzmann*Temperature);

    // double FixeCN0 =( App->m_AbsolutGpsPower) - (noise1);

    // App->m_FixeCN0= FixeCN0;

    // c'est un appel vers DroneExperiment::Cn0VersusTSP (double ReceivingSatellitepower, double
    // CN0) App->m_rxPacketTrace (App->m_AbsolutGpsPower, App->m_CN0);

    // mousaab std::cout<<"	Energy Detection completed with status:  " <<status<<" : "<<
    // LrWpanHelper::LrWpanPhyEnumerationPrinter (status) << " and energy level " <<
    // static_cast<uint32_t> (level)<<" and puissance Recu  = "<<App->m_AbsolutGpsPower<<"  ,
    // Fixenoise= "<<noise1<<"  , FixeCN0 = "<<FixeCN0<<"  dynamic noise = "<<
    // App->m_interferenceNoise<<"   Dynamic CN0 = "<<App->m_CN0<<" \n"; mousaab
    // std::cout<<"_________________________________________________________\n\n";
    std::cout << "*** Finish : PlmeEdConfirmGPS_V2 ***" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

static void
DataConfirmGPS(McpsDataConfirmParams params)
{
    std::cout << "*** Start : DataConfirmGPS ***" << std::endl;

    // std::cout<<"_____________________DataConfirmGPS (NormalDroneApp)
    // ____________________________________\n ";

    /*

   std::cout<<"________________________ DataConfirm
   (NormalDroneApp)_________________________________\n "; std::cout<<"time :" << Simulator::Now
   ().GetSeconds ()<<"\n";

   std::cout<<	"	DataConfirm = " << params.m_status<<"\n" ;

   std::cout<<"_________________________________________________________\n ";
*/
    std::cout << "*** Finish : DataConfirmGPS ***" << std::endl;
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
    std::cout << "*** Start : DataIndicationGPS ***" << std::endl;

    std::cout << "\n_____________________DataIndicationGPS (NormalDroneApp) "
                 "____________________________________\n ";
    std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};
    /****************************recuperation de mon device of
     * DATA*********************************************/
    Ptr<LrWpanNetDevice> monDevDATA;
    // Tahar:New = CreateObject<LrWpanNetDevice>();
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

    std::cout << " 			satellite ID =  " << SatelliteID_part2 << "" << SatelliteID_part1
              << ":00 \n";

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
    uint32_t index = 0;
    for (int i = 0; i < 12; i++)
    {
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

    if (App->m_methode == 1)
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
    std::cout << "*** Finish : DataIndicationGPS ***" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/********************************** Methodes standard d'application
 * ********************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
NormalDroneApp::StartApplication()
{
    std::cout << "*** Start : StartApplication ***" << std::endl;

    Ptr<LrWpanNetDevice> monDevDATA = GetNode()->GetLrWpanNetDevice(0);

    Mac16Address address = monDevDATA->GetMac()->GetShortAddress();

    Vector ma_position;
    Ptr<MobilityModel> mobility = monDevDATA->GetPhy()->GetMobility();
    ma_position = mobility->GetPosition();

    if (m_isVictim == 1)
    {
        std::cout << "Start : StartApplication : (Victim) " << std::endl;
    }
    else
    {
        std::cout << "Start : StartApplication : (Witness) " << std::endl;
    }

    std::cout << "Drone : " << monDevDATA->GetAddress() << std::endl;
    std::cout << "Position : " << ma_position.x << "|" << ma_position.y << "|" << ma_position.z
              << std::endl;

    g_received = 0;

    Simulator::Schedule(Seconds(0.0), &NormalDroneApp::InstallCallBackLrwpan, this);

    for (uint32_t i = 0; i < 12; i++)
    {
        m_NoisCounterForPRN[i] = 0;
    }

    double jitter = m_jitter->GetValue();
    std::cout << " m_pktFreq= : " << m_pktFreq << "\n";

    std::cout << " m_jitter->GetValue () = : " << jitter << "\n";

    if (m_isVictim == 1)
    {
        m_sendEvent =
            Simulator::Schedule(m_pktFreq + Seconds(jitter), &NormalDroneApp::SendPackets, this);
    }
    else
    {
        // m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (jitter),
        // &NormalDroneApp::SendPackets, this);
    }

    std::cout << "*** Finish : StartApplication ***" << std::endl;
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
    std::cout << "*** Start : InstallCallBackLrwpan ***" << std::endl;

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

    std::cout << "*** Finish : InstallCallBackLrwpan ***" << std::endl;
}

/******************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************/

void
NormalDroneApp::AppLayer(int type,
                         McpsDataIndicationParams params,
                         Ptr<LrWpanNetDevice> monDevDATA,
                         Ptr<RandomWalk2dMobilityModel> RandomMob,
                         Ptr<Node> drone,
                         Ptr<Packet> p,
                         DronParams0 PacketsInfo)
{
    std::cout << "*** Start : AppLayer ***" << std::endl;

    // std::cout<<"\n______________________ AppLayer
    // (NormalDroneApp)___________________________________\n";
    //	std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";
    g_received++;

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
        Simulator::Schedule(Seconds(0.0),
                            &NormalDroneApp::SendDetection,
                            this,
                            params,
                            monDevDATA,
                            PacketsInfo);

        break;
    case 2: // Confirmation
        Simulator::Schedule(Seconds(0.0),
                            &NormalDroneApp::SendConfirmation,
                            this,
                            params,
                            monDevDATA,
                            PacketsInfo);

        break;

    case 3: // Jump to the moon
        // Simulator::Schedule (Seconds (0.0), &NormalDroneApp::SendDoupt, this,
        // params,monDevDATA,PacketsInfo);

        break;
    case 4:

        break;
    }

    std::cout << "*** Finish : AppLayer ***" << std::endl;
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
    // Tahar : to return
    uint32_t z = (uint32_t)ma_position.z;

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
    // Tahar : to return to
    uint32_t z = (uint32_t)ma_position.z;

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

    Ptr<ConstantPositionMobilityModel> ConstantMob;
    // Tahar:New = CreateObject<ConstantPositionMobilityModel>();

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
NormalDroneApp::SendPackets()
{
    std::cout << "# Start : SendPackets() (Normal) #" << std::endl;

    std::cout << "time :" << Simulator::Now().GetSeconds() << "\n";

    /****************************recuperation du mon
     * device*********************************************/
    // Modified By Tahar
    // Ptr<LrWpanNetDevice> MonDevDATA = CreateObject<LrWpanNetDevice>();
    // MonDevDATA = GetNode()->GetLrWpanNetDevice(0);
    Ptr<LrWpanNetDevice> MonDevDATA = GetNode()->GetLrWpanNetDevice(0);

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
    /*
    std::cout << "	je suis le noeud [" << Address << "] ma position IMU est : x :" <<
    mobility->GetPosition().x
              << " , y= " << mobility->GetPosition().y << " , z= " << mobility->GetPosition().z << "
    \n\n";
    */

    std::cout << "	je suis le noeud [" << Address << std::endl;
    std::cout << "	je suis le noeud [" << MonDevDATA->GetAddress() << std::endl;
    std::cout << "	je suis le noeud [" << MonDevDATA->GetMac() << std::endl;

    /****************************recuperation des Neighbors devices
     * *********************************************/
    std::vector<Ptr<LrWpanNetDevice>> voisins = GetNode()->GetALLLrWpanNetDevice();

    // Modified By Tahar
    // Ptr<LrWpanNetDevice> devReceiver = CreateObject<LrWpanNetDevice>();
    // Ptr<LrWpanNetDevice> devReceiver;

    std::cout << " \n 	Msg ennvoye a tt le  monde  \n";

    McpsDataRequestParams params;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstPanId = 0;
    params.m_dstAddr = Mac16Address("ff:ff"); // Mac16Address("ff:ff"); // Address0; // Mac16Address ("00:02");
    params.m_msduHandle = 0;
    params.m_txOptions = 0;

    /*
        Ptr<LrWpanNetDevice> GPSdev = CreateObject<LrWpanNetDevice> ();
            GPSdev= GetNode ()->GetLrWpanNetDevice(1);
          Simulator::Schedule (Seconds (0.0025), &LrWpanPhy::PlmeEdRequest, GPSdev->GetPhy ());
    */

    double time_MicroSeconde = Simulator::Now().GetSeconds() * 1000000;
    // preparation des donnees a envoyer (first 3 subframes durant 30 seconde)
    uint32_t time = (uint32_t)time_MicroSeconde; // time en miliseconde

    std::cout << "My positions : " << std::endl
              << ma_position.x << std::endl
              << ma_position.y << std::endl
              << ma_position.z << std::endl;

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

	std::cout << data << std::endl;

    Ptr<Packet> p = Create<Packet>(data, m_pktSize);

    Simulator::Schedule(Seconds(0), &LrWpanMac::McpsDataRequest, MonDevDATA->GetMac(), params, p);

    ScheduleNextTx(0);

    std::cout << "# Finish : SendPackets() (Normal) #" << std::endl;
}

void
NormalDroneApp::ScheduleNextTx(double type)
{
    std::cout << "Start : ScheduleNextTx" << std::endl;

    m_sendEvent = Simulator::Schedule(m_pktFreq, &NormalDroneApp::SendPackets, this);

    std::cout << "Finish : ScheduleNextTx" << std::endl;
}

std::string
NormalDroneApp::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
    NS_LOG_INFO("bada debut + fin WalkBounds");

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
