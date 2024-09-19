/*
 * Malicious.h
 *
 *  Created on: September 2024
 *      Author: Tahar_Final
 */

#ifndef SRC_DRONE_MODEL_MALICIOUS_H_
#define SRC_DRONE_MODEL_MALICIOUS_H_

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

  struct DronParams1
  {
    uint32_t energy;
    uint32_t distance;
  };

  class Malicious : public Application
  {
  public:
    static TypeId GetTypeId(void);

    Malicious();

    virtual ~Malicious();

    void SendPositionResponse(McpsDataRequestParams params, Ptr<LrWpanNetDevice> myNetDevice, Ptr<Packet> p);
    void SendPackets(void);

    void SendGpsSpoofingSignal(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, DronParams1 PacketsInfo);
    void prepareNextAttack(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, DronParams1 PacketsInfo);

    void ScheduleNextTx(double);
    void EnableGpsReceiver(Ptr<LrWpanNetDevice> MoiDevReceiver);
    void InstallCallBackLrwpan();
    void AppLayer(int type, McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> node, Ptr<Packet> p, DronParams1 PacketsInfo);
    void SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver);
    void JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver, Ptr<RandomWalk2dMobilityModel> RandomMob, Ptr<Node> node);
    double Distance(Vector position1, Vector position2);

    double DoCalcRxPower(double txPowerDbm, double distance, double frequency);

    typedef void (*CollisionPacketTxTracedCallback)(const Ptr<Packet> &packet, const Ipv4Address &dest);
    typedef void (*CollisionPacketRxTracedCallback)(double, double);

    Callback<Ptr<LrWpanNetDevice>, uint16_t> GetDevice;

    enum UAV_MODE {
      NORMAL,
      ATTACK
    };
    UAV_MODE uavMode = UAV_MODE::NORMAL;
    // UAV_MODE uavMode = UAV_MODE::ATTACK;

    // Total number of UAVs (Victim + Malicious + Normal)
    int number_uav = -1;
    int number_victim = -1;

    uint32_t m_LastEnergy;
    double m_LastRatio;
    double m_interferenceNoise;

    double m_AbsolutGpsPower[12];
    double m_CN0[12];

    double m_FixeCN0;

    std::vector<DronParams1> m_DronesTable;
    TracedCallback<double, double> m_rxPacketTrace;
    
    TracedCallback<double> DronePacketNumber;
	  TracedCallback<double> SatelliteNumber;
	  TracedCallback<double> DroneReceivedNumber;
	  TracedCallback<double> SatelliteReceivedNumber;

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
    Ptr<LrWpanMac> m_mac;

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

    uint32_t m_myPRN;
    uint32_t g_received;

    // Ptr<NodeContainer>  m_container;
    uint32_t m_sent;
    uint32_t m_received;

    TracedCallback<const Ptr<Packet> &, const Ipv4Address &> m_txPacketTrace;
  };
}

#endif /* SRC_DRONE_MODEL_MALICIOUS_H_ */
