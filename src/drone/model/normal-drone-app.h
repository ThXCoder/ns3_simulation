/*
 * normal-drone-app.h
 *
 *  Created on: September 2024
 *      Author: Tahar_Final
 */

#ifndef SRC_DRONE_MODEL_NORMAL_DRONE_APP_H_
#define SRC_DRONE_MODEL_NORMAL_DRONE_APP_H_

#include "ns3/application.h"
#include "ns3/olsr-state.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/event-id.h"
#include "ns3/packet-loss-counter.h"
#include "ns3/mobility-module.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <unordered_map>

namespace ns3
{
  class Socket;

  struct DronParams0
  {
    uint32_t energy;
    uint32_t distance;
    uint32_t myPRN;
    uint32_t TTL;
  };

  struct Position
  {
    double x = -1.0;
    double y = -1.0;
    double z = -10.0;
  };

  struct CollectedData
  {
    
    // Position nodePosition;
    bool state = false;
    
    int numberOfEdges = 0;
    int maxOfEdges = 0;

    double moment = 0;

    Position nodePosition;
    double distance = -1;
    double calculatedDistance = -1;
    double receivedPower = 0; // in dBm
  };

  struct TrustLevel
  {
    double level = 0.5;
  };

  struct TrustModel
  {
    uint16_t number;
    Mac16Address id;
    uint32_t numberOfMessages = 0;

    bool state = false; // start calculating Trust

    double startTime = -1;
    double previousExchangeTime = -1;

    TrustLevel currentLevel;
    TrustLevel previousLevel;
    
    std::vector<TrustLevel> previousLevelData;
    
    CollectedData currentData;
    std::vector<CollectedData> previousData;

    bool overlap = false;
  };

  struct GpsSignal
  {
    double numberSignal = 0;

    double maxPower = -200;
    double maxCN = 0;
    double minPower = -100;
    double minCN = 60;
    
    double averagePower = 0;
    double averageCN = 0;
  };

  class NormalDroneApp : public Application
  {
  public:
    static TypeId GetTypeId(void);

    NormalDroneApp();

    virtual ~NormalDroneApp();

    void EnableGPS(Ptr<Node> node);
    void DisableGPS(Ptr<Node> node);

    uint32_t numberOfRequests = 0;

    GpsSignal gpsSignal;
    TrustModel trust;
    std::vector<TrustModel> nodeTrust;

    void SendPackets(void);

    // void ScheduleNextPositionRequest(Time frequency, double jitter);
    void ScheduleNextPositionRequest();
    void ScheduleNextTx(double);
    void InstallCallBackLrwpan();
    void AppLayer(int type, McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> node, Ptr<Packet> p, DronParams0 PacketsInfo);
    void SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver);

    void SendDetection(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, DronParams0 PacketsInfo);
    void SendConfirmation(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, DronParams0 PacketsInfo);
    void SendDoupt(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, DronParams0 PacketsInfo);

    // void SendPositionRequest(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> myNetDevice, DronParams0 PacketsInfo);
    void SendPositionRequest();
    void SendPositionResponse(McpsDataRequestParams params, Ptr<LrWpanNetDevice> myNetDevice, Ptr<Packet> p);
    void SendTrustRequest(std::vector<TrustModel>& nodeTrust, int index);
    void SendTrustResponse(McpsDataRequestParams params, Ptr<LrWpanNetDevice> myNetDevice, Ptr<Packet> p);

    uint32_t GetPacketSize();

    void JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> node);
    double Distance(Vector position1, Vector position2);

    double DoCalcRxPower(double txPowerDbm, double distance, double frequency);

    typedef void (*CollisionPacketTxTracedCallback)(const Ptr<Packet> &packet, const Ipv4Address &dest);
    typedef void (*CollisionPacketRxTracedCallback)(double, double);

    uint32_t GetGReceived();
    uint32_t GetMReceived();

    uint32_t m_LastEnergy;
    double m_LastRatio;
    double m_interferenceNoise;

    double m_LastPowerWatt = 0.0;
    double m_LastPowerDB = 0.0;
    double m_LastPowerDBM = 0.0;

    double m_AbsolutGpsPower[12];
    double m_CN0[12];
    double m_NoisCounterForPRN[12];

    double m_LastAbsolutGpsPower;
    double m_LastCN0;

    double m_isLastSignalSpoofing[12];

    double m_CurrentUavType[12];

    double m_FixeCN0;

    uint32_t m_methode;

    std::vector<DronParams0> m_DronesTable;
    TracedCallback<double, double> m_rxPacketTrace;
    TracedCallback<double, double, double> m_rxAlert;
    TracedCallback<double, double, double> m_rxDetection;

    TracedCallback<double> SNRPerformance;
    TracedCallback<double> TrustPerformance;

    TracedCallback<double> DronePacketNumber;
	  TracedCallback<double> SatelliteNumber;
	  TracedCallback<double> DroneReceivedNumber;
	  TracedCallback<double> SatelliteReceivedNumber;

    // TracedCallback<Ptr<LrWpanNetDevice>, int> One;

    // Callback
    Callback<Ptr<LrWpanNetDevice>, uint16_t> GetDevice;

    // Ptr<LrWpanNetDevice> (*GetDevice)(int i);

    uint32_t m_isVictim;

  protected:
    virtual void DoDispose(void);

  private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    std::string WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY);

    void CancelEvents(void);
    void RecvPacket(Ptr<Socket> socket);

    olsr::NeighborSet GetNeighbors(void);
    olsr::OlsrState *GetOlsrState(void);

    uint32_t m_pktSize;
    Time m_pktFreq;
    olsr::OlsrState *m_state;
    EventId m_sendEvent;
    EventId m_sendAnswer;

    TypeId m_tid;
    Ptr<Socket> m_socket;
    Ptr<RandomVariableStream> m_jitter;
    Ptr<RandomWalk2dMobilityModel> m_randomMobility;
    Ptr<LrWpanPhy> m_phy;

    Ptr<LrWpanNetDevice> m_gps_PRN0;
    Ptr<LrWpanNetDevice> m_gps_PRN1;
    Ptr<LrWpanNetDevice> m_gps_PRN2;
    Ptr<LrWpanNetDevice> m_gps_PRN3;

    Ptr<LrWpanNetDevice> m_gps_PRN4;
    Ptr<LrWpanNetDevice> m_gps_PRN5;
    Ptr<LrWpanNetDevice> m_gps_PRN6;
    Ptr<LrWpanNetDevice> m_gps_PRN7;

    Ptr<LrWpanNetDevice> m_gps_PRN8;
    Ptr<LrWpanNetDevice> m_gps_PRN9;
    Ptr<LrWpanNetDevice> m_gps_PRN10;
    Ptr<LrWpanNetDevice> m_gps_PRN11;

    Ptr<LrWpanMac> m_mac;
    uint32_t g_received;

    // Ptr<LrWpanNetDevice> m_container;

    // Ptr<NodeContainer>  m_container;
  	// std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesTable;

    uint32_t m_sent;
    uint32_t m_received;

    TracedCallback<const Ptr<Packet> &, const Ipv4Address &> m_txPacketTrace;
  };
}

#endif /* SRC_DRONE_MODEL_NORMAL_DRONE_APP_H_ */
