/*
 * normal-drone-app.h
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#ifndef SRC_DRONE_HELPER_SATELLITEHELPER_H_
#define SRC_DRONE_HELPER_SATELLITEHELPER_H_

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/event-id.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nstime.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-state.h"
#include "ns3/packet-loss-counter.h"
#include "ns3/ptr.h"

#include <unordered_map>

namespace ns3
{
class Socket;

struct DronParams0
{
    uint32_t energy;
    uint32_t distance;
};

class SatelliteApp : public Application
{
  public:
    static TypeId GetTypeId(void);

    SatelliteApp();

    virtual ~SatelliteApp();

    void SendPackets(void);

    void ScheduleNextTx(double);
    void InstallCallBackLrwpan();
    void AppLayer(int type,
                  McpsDataIndicationParams params,
                  Ptr<LrWpanNetDevice> MoiDevReceiver,
                  Ptr<RandomWalk2dMobilityModel> RandomMob,
                  Ptr<Node> node,
                  Ptr<Packet> p,
                  DronParams0 PacketsInfo);
    void SendAck(McpsDataIndicationParams params, Ptr<LrWpanNetDevice> MoiDevReceiver);
    void JumpToTheMoon(Ptr<LrWpanNetDevice> MoiDevReceiver,
                       Ptr<RandomWalk2dMobilityModel> RandomMob,
                       Ptr<Node> node);
    double Distance(Vector position1, Vector position2);

    typedef void (*CollisionPacketTxTracedCallback)(const Ptr<Packet>& packet,
                                                    const Ipv4Address& dest);
    typedef void (*CollisionPacketRxTracedCallback)(const Ptr<Packet>& packet,
                                                    const Ipv4Address& dest);

    uint32_t m_LastEnergy;
    double m_LastRatio;
    double m_interferenceNoise;

    std::vector<DronParams0> m_DronesTable;

    olsr::NeighborSet GetNeighbors(void);
    olsr::OlsrState* GetOlsrState(void);

  protected:
    virtual void DoDispose(void);

  private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    std::string WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY);

    void CancelEvents(void);
    void RecvPacket(Ptr<Socket> socket);

    uint32_t m_pktSize;
    double m_txPower;
    uint32_t m_PRN;

    Time m_pktFreq;
    olsr::OlsrState* m_state;
    EventId m_sendEvent;
    EventId m_sendAnswer;

    TypeId m_tid;
    Ptr<Socket> m_socket;
    Ptr<RandomVariableStream> m_jitter;
    Ptr<RandomWalk2dMobilityModel> m_randomMobility;
    Ptr<LrWpanPhy> m_phy;
    Ptr<LrWpanNetDevice> m_gps;

    Ptr<LrWpanMac> m_mac;
    uint32_t g_received;

    // Ptr<NodeContainer>  m_container;
    uint32_t m_sent;
    uint32_t m_received;

    TracedCallback<const Ptr<Packet>&, const Ipv4Address&> m_rxPacketTrace;
    TracedCallback<const Ptr<Packet>&, const Ipv4Address&> m_txPacketTrace;
};
} // namespace ns3

#endif /* SRC_DRONE_MODEL_NORMAL_DRONE_APP_H_ */
