/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 */
#include "lr-wpan-phy.h"

#include "lr-wpan-constants.h"
#include "lr-wpan-error-model.h"
#include "lr-wpan-lqi-tag.h"
#include "lr-wpan-net-device.h"
#include "lr-wpan-spectrum-signal-parameters.h"
#include "lr-wpan-spectrum-value-helper.h"

#include <ns3/abort.h>
#include <ns3/antenna-model.h>
#include <ns3/double.h>
#include <ns3/error-model.h>
#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device.h>
#include <ns3/node.h>
#include <ns3/packet-burst.h>
#include <ns3/packet.h>
#include <ns3/pointer.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>
#include <ns3/spectrum-channel.h>
#include <ns3/spectrum-value.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LrWpanPhy");

NS_OBJECT_ENSURE_REGISTERED(LrWpanPhy);

// Table 22 in section 6.4.1 of ieee802.15.4
const uint32_t LrWpanPhy::aMaxPhyPacketSize = 127; // max PSDU in octets
const uint32_t LrWpanPhy::aTurnaroundTime = 12;    // RX-to-TX or TX-to-RX in symbol periods

// IEEE802.15.4-2006 Table 2 in section 6.1.2 (kb/s and ksymbol/s)
// The index follows LrWpanPhyOption
static const LrWpanPhyDataAndSymbolRates dataSymbolRates[IEEE_802_15_4_INVALID_PHY_OPTION]{
    {20.0, 20.0},
    {40.0, 40.0},
    {20.0, 20.0},
    {250.0, 12.5},
    {250.0, 50.0},
    {250.0, 62.5},
    {100.0, 25.0},
    {250.0, 62.5},
    {250.0, 62.5},
};


/**
 * The preamble, SFD, and PHR lengths in symbols for the different PHY options.
 * See Table 19 and Table 20 in section 6.3 IEEE 802.15.4-2006, IEEE 802.15.4c-2009, IEEE
 * 802.15.4d-2009.
 * The PHR is 1 octet and it follows phySymbolsPerOctet in Table 23.
 * The index follows LrWpanPhyOption.
 */
const LrWpanPhyPpduHeaderSymbolNumber ppduHeaderSymbolNumbers[IEEE_802_15_4_INVALID_PHY_OPTION]{
    {32.0, 8.0, 8.0},
    {32.0, 8.0, 8.0},
    {32.0, 8.0, 8.0},
    {2.0, 1.0, 0.4},
    {6.0, 1.0, 1.6},
    {8.0, 2.0, 2.0},
    {8.0, 2.0, 2.0},
    {8.0, 2.0, 2.0},
    {8.0, 2.0, 2.0},
};

std::ostream&
operator<<(std::ostream& os, const LrWpanPhyEnumeration& state)
{
    switch (state)
    {
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY:
        os << "BUSY";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_RX:
        os << "BUSY_RX";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_TX:
        os << "BUSY_TX";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_FORCE_TRX_OFF:
        os << "FORCE_TRX_OFF";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_IDLE:
        os << "IDLE";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_INVALID_PARAMETER:
        os << "INVALID_PARAMETER";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_RX_ON:
        os << "RX_ON";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_SUCCESS:
        os << "SUCCESS";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_TRX_OFF:
        os << "TRX_OFF";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_TX_ON:
        os << "TX_ON";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE:
        os << "UNSUPPORTED";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_READ_ONLY:
        os << "READ_ONLY";
        break;
    case LrWpanPhyEnumeration::IEEE_802_15_4_PHY_UNSPECIFIED:
        os << "UNSPECIFIED";
        break;
    }
    return os;
};

std::ostream&
operator<<(std::ostream& os, const TracedValue<LrWpanPhyEnumeration>& state)
{
    LrWpanPhyEnumeration s = state;
    return os << s;
};


TypeId
LrWpanPhy::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::LrWpanPhy")
            .SetParent<SpectrumPhy>()
            .SetGroupName("LrWpan")
            .AddConstructor<LrWpanPhy>()
            .AddAttribute("PostReceptionErrorModel",
                          "An optional packet error model can be added to the receive "
                          "packet process after any propagation-based (SNR-based) error "
                          "models have been applied. Typically this is used to force "
                          "specific packet drops, for testing purposes.",
                          PointerValue(),
                          MakePointerAccessor(&LrWpanPhy::m_postReceptionErrorModel),
                          MakePointerChecker<ErrorModel>())
            .AddTraceSource("TrxStateValue",
                            "The state of the transceiver",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_trxState),
                            "ns3::TracedValueCallback::LrWpanPhyEnumeration")
            .AddTraceSource("TrxState",
                            "The state of the transceiver",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_trxStateLogger),
                            "ns3::LrWpanPhy::StateTracedCallback")
            .AddTraceSource("PhyTxBegin",
                            "Trace source indicating a packet has "
                            "begun transmitting over the channel medium",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyTxBeginTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PhyTxEnd",
                            "Trace source indicating a packet has been "
                            "completely transmitted over the channel.",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyTxEndTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PhyTxDrop",
                            "Trace source indicating a packet has been "
                            "dropped by the device during transmission",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyTxDropTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PhyRxBegin",
                            "Trace source indicating a packet has begun "
                            "being received from the channel medium by the device",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyRxBeginTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PhyRxEnd",
                            "Trace source indicating a packet has been "
                            "completely received from the channel medium "
                            "by the device",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyRxEndTrace),
                            "ns3::Packet::SinrTracedCallback")
            .AddTraceSource("PhyRxDrop",
                            "Trace source indicating a packet has been "
                            "dropped by the device during reception",
                            MakeTraceSourceAccessor(&LrWpanPhy::m_phyRxDropTrace),
                            "ns3::Packet::TracedCallback");
    return tid;
}

LrWpanPhy::LrWpanPhy(void)
    : m_edRequest(),
      m_edSatelliteRequest(),
      m_EndRxSatelliteRequest(),
      m_setTRXState()
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::LrWpanPhy
    ///*************\n";
    m_SatelliteRxMode = MaxPeakWithoutRAZ;
    m_isSatellite = 0;
    m_GPSstate = 1;
    m_trxState = IEEE_802_15_4_PHY_TRX_OFF;
    m_trxStatePending = IEEE_802_15_4_PHY_IDLE;

    // default PHY PIB attributes
    m_phyPIBAttributes.phyCurrentChannel = 11;
    m_phyPIBAttributes.phyTransmitPower = 0;
    m_phyPIBAttributes.phyCurrentPage = 0;
    for (uint32_t i = 0; i < 32; i++)
    {
        m_phyPIBAttributes.phyChannelsSupported[i] = 0x07ffffff;
    }
    m_phyPIBAttributes.phyCCAMode = 1;

    SetMyPhyOption();

    m_edPower.averagePower = 0.0;
    m_edPower.lastUpdate = Seconds(0.0);
    m_edPower.measurementLength = Seconds(0.0);

    // default -110 dBm in W for 2.4 GHz
    m_rxSensitivity = pow(10.0, -106.58 / 10.0) / 1000.0;
    LrWpanSpectrumValueHelper psdHelper;
    m_txPsd = psdHelper.CreateTxPowerSpectralDensity(m_phyPIBAttributes.phyTransmitPower,
                                                     m_phyPIBAttributes.phyCurrentChannel);

    //////IDS //mousaab std::cout<<"	1		    <m_phyPIBAttributes.phyTransmitPower = "<<
    ///(uint32_t)m_phyPIBAttributes.phyTransmitPower<<"      >> \n";
    //////IDS //mousaab std::cout<<"	1		    <m_phyPIBAttributes.phyCurrentChannel = "<<
    ///(uint32_t)m_phyPIBAttributes.phyCurrentChannel<<"      >> \n";
    //////IDS //mousaab std::cout<<"	1		    <m_txPsd = "<< *m_txPsd<<"      >> \n";

    m_noise = psdHelper.CreateNoisePowerSpectralDensity(m_phyPIBAttributes.phyCurrentChannel);

    //////IDS //mousaab std::cout<<"			    <m_noise = "<< *m_noise<<"      >> \n";

    m_signal = Create<LrWpanInterferenceHelper>(m_noise->GetSpectrumModel());

    m_rxLastUpdate = Seconds(0);
    Ptr<Packet> none_packet = 0;
    Ptr<LrWpanSpectrumSignalParameters> none_params = 0;
    m_currentRxPacket = std::make_pair(none_params, true);
    m_currentTxPacket = std::make_pair(none_packet, true);
    m_errorModel = nullptr;

    m_random = CreateObject<UniformRandomVariable>();
    m_random->SetAttribute("Min", DoubleValue(0.0));
    m_random->SetAttribute("Max", DoubleValue(1.0));

    ChangeTrxState(IEEE_802_15_4_PHY_TRX_OFF);
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::LrWpanPhy
    ///*************\n";
}

LrWpanPhy::~LrWpanPhy(void)
{
}

void
LrWpanPhy::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    // This method ensures that the local mobility model pointer holds
    // a pointer to the Node's aggregated mobility model (if one exists)
    // in the case that the user has not directly called SetMobility()
    // on this LrWpanPhy during simulation setup.  If the mobility model
    // needs to be added or changed during simulation runtime, users must
    // call SetMobility() on this object.

    if (!m_mobility)
    {
        NS_ABORT_MSG_UNLESS(m_device && m_device->GetNode(),
                            "Either install a MobilityModel on this object or ensure that this "
                            "object is part of a Node and NetDevice");
        m_mobility = m_device->GetNode()->GetObject<MobilityModel>();
        if (!m_mobility)
        {
            NS_LOG_WARN("Mobility not found, propagation models might not work properly");
        }
    }
}

void
LrWpanPhy::DoDispose(void)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::DoDispose
    ///*************\n";

    NS_LOG_FUNCTION(this);

    // Cancel pending transceiver state change, if one is in progress.
    m_setTRXState.Cancel();
    m_trxState = IEEE_802_15_4_PHY_TRX_OFF;
    m_trxStatePending = IEEE_802_15_4_PHY_IDLE;

    m_mobility = nullptr;
    m_device = nullptr;
    m_channel = nullptr;
    m_antenna = nullptr;
    m_txPsd = nullptr;
    m_noise = nullptr;
    m_signal = nullptr;
    m_errorModel = nullptr;
    m_currentRxPacket.first = nullptr;
    m_currentTxPacket.first = nullptr;
    m_postReceptionErrorModel = nullptr;

    m_ccaRequest.Cancel();
    m_edRequest.Cancel();
    m_setTRXState.Cancel();
    m_pdDataRequest.Cancel();

    m_random = nullptr;

    m_pdDataIndicationCallback = MakeNullCallback<void, uint32_t, Ptr<Packet>, uint8_t>();
    m_pdDataConfirmCallback = MakeNullCallback<void, LrWpanPhyEnumeration>();
    m_plmeCcaConfirmCallback = MakeNullCallback<void, LrWpanPhyEnumeration>();
    m_plmeEdConfirmCallback = MakeNullCallback<void, LrWpanPhyEnumeration, uint8_t>();
    m_plmeEdConfirmCallback_V2 =
        MakeNullCallback<void, LrWpanPhyEnumeration, uint8_t, double, double, double>();
    m_plmeGetAttributeConfirmCallback = MakeNullCallback<void,
                                                         LrWpanPhyEnumeration,
                                                         LrWpanPibAttributeIdentifier,
                                                         Ptr<LrWpanPhyPibAttributes>>();
    m_plmeSetTRXStateConfirmCallback = MakeNullCallback<void, LrWpanPhyEnumeration>();
    m_plmeSetAttributeConfirmCallback =
        MakeNullCallback<void, LrWpanPhyEnumeration, LrWpanPibAttributeIdentifier>();

    SpectrumPhy::DoDispose();
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::DoDispose
    ///*************\n";
}

/*
Ptr<Node>
LrWpanPhy::GetNode (void) const
{
     //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::GetNode
*************\n";

  NS_LOG_FUNCTION (this);
  //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::GetNode
*************\n";

  return m_attachedNode;

}
*/

Ptr<NetDevice>
LrWpanPhy::GetDevice() const
{
    return m_device;
}

Ptr<MobilityModel>
LrWpanPhy::GetMobility() const
{
    return m_mobility;
}

void
LrWpanPhy::SetSatelliteReceptionMode(SatelliteReceptionMode SatelliteRxMode)
{
    m_SatelliteRxMode = SatelliteRxMode;
}

/*

void
LrWpanPhy::SetNode (Ptr<Node>  n)
{
     //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::SetNode
*************\n";

  NS_LOG_FUNCTION (this << n);
  //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::SetNode
*************\n";

  m_attachedNode = n;
}

*/

void
LrWpanPhy::SetDevice(Ptr<NetDevice> d)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::SetDevice
    ///*************\n";

    NS_LOG_FUNCTION(this << d);
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::SetDevice
    ///*************\n";

    m_device = d;
}

void
LrWpanPhy::SetMobility(Ptr<MobilityModel> m)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::SetMobility
    ///*************\n";

    NS_LOG_FUNCTION(this << m);
    m_mobility = m;
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::SetMobility
    ///*************\n";
}

void
LrWpanPhy::SetChannel(Ptr<SpectrumChannel> c)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::SetChannel
    ///*************\n";

    NS_LOG_FUNCTION(this << c);
    m_channel = c;
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::SetChannel
    ///*************\n";
}

Ptr<SpectrumChannel>
LrWpanPhy::GetChannel(void)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::GetChannel
    ///*************\n";

    NS_LOG_FUNCTION(this);
    return m_channel;
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::GetChannel
    ///*************\n";
}

Ptr<const SpectrumModel>
LrWpanPhy::GetRxSpectrumModel(void) const
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut
    /// LrWpanPhy::GetRxSpectrumModel  *************\n";

    NS_LOG_FUNCTION(this);

    //////IDS //mousaab std::cout<<"\n\n		*********************  fin
    /// LrWpanPhy::GetRxSpectrumModel  *************\n";

    if (m_txPsd)
    {
        return m_txPsd->GetSpectrumModel();
    }
    else
    {
        return 0;
    }
}

Ptr<SpectrumValue>
LrWpanPhy::GetTxPowerSpectralDensity(void) const
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut
    /// LrWpanPhy::GetTxPowerSpectralDensity  *************\n";

    NS_LOG_FUNCTION(this);

    //////IDS //mousaab std::cout<<"\n\n		*********************  fin
    /// LrWpanPhy::GetTxPowerSpectralDensity  *************\n";

    return m_txPsd;
}

Ptr<AntennaModel>
LrWpanPhy::GetRxAntenna(void) const
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::GetRxAntenna
    ///*************\n";

    NS_LOG_FUNCTION(this);
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::GetRxAntenna
    ///*************\n";

    return m_antenna;
}

Ptr<Object>
LrWpanPhy::GetAntenna() const
{
    return m_antenna;
}

void
LrWpanPhy::SetAntenna(Ptr<AntennaModel> a)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy::SetAntenna
    ///*************\n";

    NS_LOG_FUNCTION(this << a);
    m_antenna = a;
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin LrWpanPhy::SetAntenna
    ///*************\n";
}

void
LrWpanPhy::StartSatelliteRx(Ptr<SpectrumSignalParameters> spectrumRxParams)
{
    // IDS //mousaab
    // std::cout<<"=============================================================================================\n";
    // IDS //mousaab std::cout<<"	m_signal->GetSignalPsd () :" << *(m_signal->GetSignalPsd
    // ())<<"\n";

    NS_LOG_FUNCTION(this << spectrumRxParams);
    LrWpanSpectrumValueHelper psdHelper;

    if (!m_edRequest.IsExpired())
    {
        // Update the average receive power during ED.

        Time now = Simulator::Now();
        // IDS //mousaab std::cout<<" Energy detection REQUEST is not expired\n";

        ////IDS //mousaab std::cout<<"			    <  m_interferenceAndNoise = "<<
        ///*(m_interferenceAndNoise)<<"      >> \n"; /IDS //mousaab std::cout<<" <before
        /// m_InterferenceAvg = "<<  m_InterferenceAvg<<"      >> \n";

        m_edPower.averagePower +=
            LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                     m_phyPIBAttributes.phyCurrentChannel) *
            (now - m_edPower.lastUpdate).GetTimeStep() / m_edPower.measurementLength.GetTimeStep();
        m_InterferenceAvg +=
            LrWpanSpectrumValueHelper::TotalAvgPower(m_interferenceAndNoise,
                                                     m_phyPIBAttributes.phyCurrentChannel) *
            (now - m_edPower.lastUpdate).GetTimeStep() / m_edPower.measurementLength.GetTimeStep();
        // double AvgNoise = 10 * log10 (m_InterferenceAvg);

        m_edPower.lastUpdate = now;
        ////IDS //mousaab std::cout<<"			    < after    m_InterferenceAvg = "<<
        /// m_InterferenceAvg<<"      >> \n\n";

        ////IDS //mousaab std::cout<<"			    < after   log10 of  m_InterferenceAvg = "<<
        /// AvgNoise<<"      >> \n\n";
    }

    Ptr<LrWpanSpectrumSignalParameters> lrWpanRxParams =
        DynamicCast<LrWpanSpectrumSignalParameters>(spectrumRxParams);

    if (lrWpanRxParams == nullptr)
    {
        CheckInterference();
        m_signal->AddSignal(spectrumRxParams->psd);

        // Update peak power if CCA is in progress.
        if (!m_ccaRequest.IsExpired())
        {
            double power =
                LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                         m_phyPIBAttributes.phyCurrentChannel);

            if (m_ccaPeakPower < power)
            {
                m_ccaPeakPower = power;
            }
        }
        Simulator::Schedule(spectrumRxParams->duration, &LrWpanPhy::EndRx, this, spectrumRxParams);
        return;
    }

    // recuperation du packet
    Ptr<Packet> p = (lrWpanRxParams->packetBurst->GetPackets()).front();
    NS_ASSERT(p != nullptr);
    uint32_t recvSize = p->GetSize();
    uint8_t* bufferTompo = new uint8_t[recvSize];
    p->CopyData((uint8_t*)bufferTompo, recvSize);

    // calcule du PRN
    uint32_t ReceivedPRN;
    ReceivedPRN = (bufferTompo[34] << 24) + (bufferTompo[33] << 16) + (bufferTompo[32] << 8) +
                  bufferTompo[31];

    // l'id + position reel du satellite
    // IDS  uint32_t SatelliteID_part1 = (uint32_t)bufferTompo[29]&255;
    // uint32_t SatelliteID_part2 = (uint32_t)bufferTompo[30]&255;

    // recuperer les position du satellite et UAV
    Ptr<MobilityModel> txMobility = spectrumRxParams->txPhy->GetMobility();
    // IDS Vector SatellitePosition =txMobility->GetPosition ();

    Ptr<MobilityModel> rxMobility = GetMobility();
    // IDS Vector UavPosition =rxMobility->GetPosition ();

    // calculer l'angle
    Angles txAngles(rxMobility->GetPosition(), txMobility->GetPosition());

    // calculer la puissance recue
    double NewAbsolutPower =
        10 * log10(LrWpanSpectrumValueHelper::TotalAvgPower(lrWpanRxParams->psd,
                                                            m_phyPIBAttributes.phyCurrentChannel)) -
        2;

    // IDS uint32_t adressUAV_Parte1 =  (0<<24)+(0<<16)+(0<<8)+m_attachedAdresse[0];
    // IDS uint32_t adressUAV_Parte2 =  (0<<24)+(0<<16)+(0<<8)+m_attachedAdresse[1];

    // IDS //mousaab std::cout<<" 		+++++++++++++++++	UAV ID = 00: " << unsigned
    // (m_attachedAdresse[0])<< "-"<<unsigned (m_attachedAdresse[1])<<"  ++++++++++++++++\n";

    // IDS //mousaab std::cout<<" 		+++++++++++++++++	satellite ID =  " << SatelliteID_part2<<
    // ""<<SatelliteID_part1<<":00  ++++++++++++++++\n";

    // IDS //mousaab std::cout<<"			    SatellitePosition.x = "<<  SatellitePosition.x<<" >>
    // \n"; IDS //mousaab std::cout<<"			    SatellitePosition.y = "<< SatellitePosition.y<<"
    // >> \n"; IDS //mousaab std::cout<<"			    SatellitePosition.z = "<<
    // SatellitePosition.z<<"      >> \n";

    // IDS //mousaab std::cout<<"			    UavPosition.x = "<<  UavPosition.x<<"      >> \n";
    // IDS //mousaab std::cout<<"			    UavPosition.y = "<<  UavPosition.y<<"      >> \n";
    // IDS //mousaab std::cout<<"			    UavPosition.z = "<<  UavPosition.z<<"      >> \n";
    // IDS //mousaab std::cout<<"			 receiving packet with power: " << 10 *
    // log10(LrWpanSpectrumValueHelper::TotalAvgPower (lrWpanRxParams->psd,
    // m_phyPIBAttributes.phyCurrentChannel)) -2 << "dBm\n";

    // IDS //mousaab std::cout<<"			txAngles.phi = " << txAngles.phi << " dB \n";
    // IDS //mousaab std::cout<<"			txAngles.theta = " << txAngles.theta << " dB \n";

    if (ReceivedPRN == m_PrnChannel)
    {
        // mousaab std::cout<<"==================================== Debut LrWpanPhy::
        // StartRxSatellite  ====================================\n"; mousaab std::cout<<"time :" <<
        // Simulator::Now ().GetSeconds ()<<"\n";

        // mousaab std::cout<<"------------>c'est mon canal\n";

        // mousaab std::cout<<" 		+++++++++++++++++	satellite ID =  " << SatelliteID_part2<<
        // ""<<SatelliteID_part1<<":00  ++++++++++++++++\n";

        if (m_trxState == IEEE_802_15_4_PHY_RX_ON && !m_setTRXState.IsRunning())
        {
            // mousaab std::cout<<"------------>channel free\n";

            // savegarder la puissance du 1er signale dans la variable globale
            m_AbsolutPower = NewAbsolutPower;

            // calculer la correlation
            uint32_t PrnCodeTable[] = {
                4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
                2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
                3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
                549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
                28594359,   349684609,  9999122,    224443546};
            uint32_t localPrn = 0;
            uint32_t PrnXor;
            uint32_t SumOf32BitsPrnXor;
            uint32_t MaxSum_PrnXorLocalPrn = 0;

            for (uint32_t i = 0; i < 32; i++)
            {
                PrnXor = ~(ReceivedPRN) ^ PrnCodeTable[i];
                PrnXor = PrnXor - ((PrnXor >> 1) & 0x55555555);
                PrnXor = (PrnXor & 0x33333333) + ((PrnXor >> 2) & 0x33333333);
                SumOf32BitsPrnXor = (((PrnXor + (PrnXor >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;

                if (SumOf32BitsPrnXor > MaxSum_PrnXorLocalPrn)
                {
                    MaxSum_PrnXorLocalPrn = SumOf32BitsPrnXor;
                    localPrn = PrnCodeTable[i];
                }
            }
            // savegarder le PRN + la correlation du signal tracker
            m_PrnOfMaxPeak = localPrn;
            double Correlation = MaxSum_PrnXorLocalPrn / 32.0;
            m_Correlation = Correlation;

            // calculer le 1er peak
            Ptr<SpectrumValue> FirstSignal = lrWpanRxParams->psd;
            // IDS //mousaab std::cout<<"			    <FirstSignal avant multiplication  = "<<
            // *(FirstSignal)<<"      >> \n";
            (*FirstSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 - 2] *=
                Correlation;
            (*FirstSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 - 1] *=
                Correlation;
            (*FirstSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400] *=
                Correlation;
            (*FirstSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 + 1] *=
                Correlation;
            (*FirstSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 + 2] *=
                Correlation;
            // IDS //mousaab std::cout<<"			    < FirstSignal apres multiplication  = "<<
            // *(FirstSignal)<<"      >> \n";

            // affecter le 1er signal correle'
            m_signal->AddSignal(FirstSignal);
            ////IDS //mousaab std::cout<<"			    <m_signal  = "<<
            ///*(m_signal->GetSignalPsd())<<"      >> \n";

            // calculer la 1ere interference =0 ;
            Ptr<SpectrumValue> interferenceAndNoise = m_signal->GetSignalPsd();
            *interferenceAndNoise -= *FirstSignal;
            // IDS //mousaab std::cout<<"			    <interference without Noise  = "<<
            // *(interferenceAndNoise)<<"      >> \n"; calculer le guassien noise/ T= 300 kalven
            LrWpanSpectrumValueHelper psdHelper;
            double noiseFactor = 1;
            m_noise =
                psdHelper.CreateSatelliteNoise(m_phyPIBAttributes.phyCurrentChannel, noiseFactor);
            // ajouter le guassien noise a  l'interference
            *interferenceAndNoise += *m_noise;
            // IDS //mousaab std::cout<<"			    <interference with Noise  = "<<
            // *(interferenceAndNoise)<<"      >> \n"; savegarder le noise dans une variable globale
            m_interferenceAndNoise = interferenceAndNoise->Copy();

            // calculer les differents parametres de securite
            // IDS double noise = 10 * log10 (LrWpanSpectrumValueHelper::TotalAvgPower
            // (m_interferenceAndNoise, m_phyPIBAttributes.phyCurrentChannel));
            double CurrentPower = 10 * log10(LrWpanSpectrumValueHelper::TotalAvgPower(
                                           m_signal->GetSignalPsd(),
                                           m_phyPIBAttributes.phyCurrentChannel)) -
                                  2;
            // IDS double sinr = LrWpanSpectrumValueHelper::TotalAvgPower (m_signal->GetSignalPsd(),
            // m_phyPIBAttributes.phyCurrentChannel) / LrWpanSpectrumValueHelper::TotalAvgPower
            // (m_interferenceAndNoise, m_phyPIBAttributes.phyCurrentChannel); IDS double CN0 =
            // CurrentPower- noise;

            // savegarder la puissance correlee en dBw du PRN tracker dans une variable globale
            m_MaxGpsSignalPeak = CurrentPower;

            // savegarder les parametres du signale tracker
            m_spectrumSatelliteRxParams = spectrumRxParams->Copy();

            // changer l'etat du support physique
            ChangeTrxState(IEEE_802_15_4_PHY_BUSY_RX);

            // savegarder les parametres du packet tracker
            m_currentRxPacket = std::make_pair(lrWpanRxParams, false);
            m_phyRxBeginTrace(p);
            m_rxLastUpdate = Simulator::Now();

            // lancer le calcule d'energie (qui contien un appel pour terminer la reception )
            Simulator::ScheduleNow(&LrWpanPhy::PlmeSatelliteEdRequest,
                                   this,
                                   spectrumRxParams->duration,
                                   spectrumRxParams);

            // IDS //mousaab std::cout<<"	                 IEEE_802_15_4_PHY_RX_ON  \n";
            ////IDS //mousaab std::cout<<"			    < m_PrnOfMaxPeak = "<< m_PrnOfMaxPeak<<" >>
            ///\n"; /IDS //mousaab std::cout<<"			    < m_MaxGpsSignalPeak = "<<
            /// m_MaxGpsSignalPeak<<"      >> \n";
            // IDS //mousaab std::cout<<"		m_PrnOfMaxPeak  "<<m_PrnOfMaxPeak <<"      >> \n";
            // IDS //mousaab std::cout<<"		ReceivedPRN = "<< ReceivedPRN<<"      >> \n";
            ////IDS //mousaab std::cout<<"		MaxSum_PrnXorLocalPrn  "<<MaxSum_PrnXorLocalPrn <<"
            ///>> \n";
            // IDS //mousaab std::cout<<"		Correlation  "<<Correlation <<"      >> \n";
            // IDS //mousaab std::cout<<"\n			    < noise= "<< noise<<"      >> \n";
            // IDS //mousaab std::cout<<"			    < CurrentPower*correlation= "<<
            // CurrentPower<<"      >> \n"; IDS //mousaab std::cout<<"\n			    < sinr
            // (CurrentPower/interference)= "<< sinr<<"      >> \n"; IDS //mousaab std::cout<<"
            // < 10 log (sinr )= "<< 10 * log10(sinr)<<"      >> \n"; IDS //mousaab std::cout<<"\n
            // < CN0 (CurrentPower- noise)= "<< CN0<<"      >> \n"; IDS //mousaab std::cout<<"
            // < m_PrnOfMaxPeak final= "<< m_PrnOfMaxPeak<<"      >> \n"; IDS //mousaab std::cout<<"
            // < m_MaxGpsSignalPeak final= "<< m_MaxGpsSignalPeak<<"      >> \n";
            //////IDS //mousaab std::cout<<"		^^^^^^^^^ deveice belong to  GNSS System
            ///(satellite / UAV GPS receiver)   \n";
        }
        else
        {
            if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
            {
                // mousaab std::cout<<"------------>channel busy\n";

                // IDS //mousaab std::cout<<"33333 collision , il est entraine de recevoir un autre
                // paquet \n"; IDS //mousaab std::cout<<"			    <ancien  m_PrnOfMaxPeak =
                // "<< m_PrnOfMaxPeak<<"      >> \n"; IDS //mousaab std::cout<<"
                // <ReceivedPRN = "<< ReceivedPRN<<"      >> \n"; IDS //mousaab std::cout<<"
                // <ancien  m_MaxGpsSignalPeak = "<< m_MaxGpsSignalPeak<<"      >> \n"; IDS
                // //mousaab std::cout<<"			    <NewAbsolutPower = "<< NewAbsolutPower<<" >>
                // \n";

                NS_LOG_DEBUG(this << " packet collision");

                // Check if we correctly received the old packet up to now.
                CheckInterference();

                if (m_SatelliteRxMode == FIFO)
                {
                    // mousaab std::cout<<"-----------------------> FIFO\n";

                    // calculer la nouvelle valeur de correlation du bruit
                    uint32_t localPrn = m_PrnOfMaxPeak;
                    uint32_t PrnXor;
                    uint32_t SumOf32BitsPrnXor;
                    PrnXor = ~(ReceivedPRN) ^ localPrn;
                    PrnXor = PrnXor - ((PrnXor >> 1) & 0x55555555);
                    PrnXor = (PrnXor & 0x33333333) + ((PrnXor >> 2) & 0x33333333);
                    SumOf32BitsPrnXor =
                        (((PrnXor + (PrnXor >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;

                    // calculer la correlation du bruit qu'on doit etre multiplier par 1.0e-6 pour
                    // le degrader (car on deja degrader noisePowerDensity )
                    double Correlation = SumOf32BitsPrnXor / (32.0 * 1.0e6);

                    // recuperer une copie du nouveau signal qui va devenir un bruit
                    Ptr<SpectrumValue> NewSignal = lrWpanRxParams->psd->Copy();

                    // calculer le 1er peak
                    // IDS //mousaab std::cout<<"			    < NewSignal avant multiplication  =
                    // "<< *(NewSignal)<<"      >> \n";
                    (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 -
                                 2] *= Correlation;
                    (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 -
                                 1] *= Correlation;
                    (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400] *=
                        Correlation;
                    (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 +
                                 1] *= Correlation;
                    (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 +
                                 2] *= Correlation;
                    // IDS //mousaab std::cout<<"			    < NewSignal apres multiplication  =
                    // "<< *(NewSignal)<<"      >> \n";

                    // ajouter le nouveau signal correle au bruit global
                    *m_interferenceAndNoise += *NewSignal;
                    // IDS //mousaab std::cout<<"			    < interference final  = "<<
                    // *(m_interferenceAndNoise)<<"      >> \n";

                    // supprimer le packet
                    m_phyRxDropTrace(p);
                }
                else
                {
                    // si on a recu un signal qui a le meme prn du signal trackeravec une puissance
                    // superieur
                    if ((ReceivedPRN == m_PrnOfMaxPeak) && (NewAbsolutPower > m_AbsolutPower))
                    {
                        // mousaab std::cout<<"------------>Same PRN    but    new Power is
                        // greater\n";

                        // IDS //mousaab std::cout<<"
                        // spoooooooo0000000000000000000000000000oooooooooof \n"; IDS //mousaab
                        // std::cout<<" spoooooooo0000000000000000000000000000oooooooooof \n"; IDS
                        // //mousaab std::cout<<" spoooooooo0000000000000000000000000000oooooooooof
                        // \n";

                        // s'il faut reinitialiser le bruit et le calculer a nouveau  (relancer la
                        // detection d'energie )
                        if (m_SatelliteRxMode == MaxPeakRAZ)
                        {
                            // mousaab std::cout<<"------------>RAZ (power and noise)\n";

                            if (!m_edSatelliteRequest.IsExpired())
                            {
                                m_edSatelliteRequest.Cancel();
                            }

                            if (!m_EndRxSatelliteRequest.IsExpired())
                            {
                                m_EndRxSatelliteRequest.Cancel();
                            }

                            // liberer le support pour lacer la reception du 1er signal a nouveau
                            ChangeTrxState(IEEE_802_15_4_PHY_RX_ON);
                            m_signal->ClearSignals();
                            m_MaxGpsSignalPeak = 0;
                            m_AbsolutPower = 0;

                            // relancer la reception a nouveau
                            StartSatelliteRx(spectrumRxParams);
                        }

                        // garder l'ancienne interference/ on change seulement la puissance du
                        // signale recu (modifier le peak) l'ancienne signal peak devient une
                        // interference qu'on doit rajouter au bruit globale
                        if (m_SatelliteRxMode == MaxPeakWithoutRAZ)
                        {
                            // mousaab std::cout<<"------------>RAZ (Only the Absolute power)\n";

                            // la correlation doit etre multiplier par 1.0e-6 pour le degrader (car
                            // on deja degrader noisePowerDensity )
                            double Correlation = m_Correlation * (1.0e-6);

                            // convertir l'ancien signal to noise , en le multipliant par la
                            // corrlaion
                            Ptr<SpectrumValue> interferenceAndNoise = m_signal->GetSignalPsd();

                            (*interferenceAndNoise)[2405 +
                                                    5 * (m_phyPIBAttributes.phyCurrentChannel -
                                                         11) -
                                                    2400 - 2] *= Correlation;
                            (*interferenceAndNoise)[2405 +
                                                    5 * (m_phyPIBAttributes.phyCurrentChannel -
                                                         11) -
                                                    2400 - 1] *= Correlation;
                            (*interferenceAndNoise)
                                [2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400] *=
                                Correlation;
                            (*interferenceAndNoise)[2405 +
                                                    5 * (m_phyPIBAttributes.phyCurrentChannel -
                                                         11) -
                                                    2400 + 1] *= Correlation;
                            (*interferenceAndNoise)[2405 +
                                                    5 * (m_phyPIBAttributes.phyCurrentChannel -
                                                         11) -
                                                    2400 + 2] *= Correlation;

                            // ajouter l'ancien signal a l'ancienne interference
                            *m_interferenceAndNoise += (*interferenceAndNoise);

                            // affecter le nouveau signal (comme signal principale)
                            m_signal->ClearSignals();
                            m_signal->AddSignal(lrWpanRxParams->psd);

                            // calculer les parametres de securite
                            // IDS double noise = 10 * log10
                            // (LrWpanSpectrumValueHelper::TotalAvgPower (m_interferenceAndNoise,
                            // m_phyPIBAttributes.phyCurrentChannel));
                            double CurrentPower =
                                10 * log10(LrWpanSpectrumValueHelper::TotalAvgPower(
                                         m_signal->GetSignalPsd(),
                                         m_phyPIBAttributes.phyCurrentChannel)) -
                                2;
                            // IDS double sinr = LrWpanSpectrumValueHelper::TotalAvgPower
                            // (m_signal->GetSignalPsd(), m_phyPIBAttributes.phyCurrentChannel) /
                            // LrWpanSpectrumValueHelper::TotalAvgPower (m_interferenceAndNoise,
                            // m_phyPIBAttributes.phyCurrentChannel); IDS double CN0 = CurrentPower-
                            // noise;

                            // savegarder la nouvelle puissance en dBw
                            m_MaxGpsSignalPeak = CurrentPower;
                            m_AbsolutPower = CurrentPower;

                            // savegarder les nouveaux parametres du packet tracker
                            m_currentRxPacket = std::make_pair(lrWpanRxParams, false);
                            m_phyRxBeginTrace(p);
                            m_rxLastUpdate = Simulator::Now();

                            // eprogrammer la fin de la reception du signal
                            m_EndRxSatelliteRequest = Simulator::Schedule(Seconds(0.6),
                                                                          &LrWpanPhy::EndRx,
                                                                          this,
                                                                          spectrumRxParams);

                            // IDS //mousaab std::cout<<"			    < NewSignal spoofing  = "<<
                            // *(m_signal->GetSignalPsd())<<"      >> \n"; IDS //mousaab
                            // std::cout<<"			    < interference final  = "<<
                            // *(m_interferenceAndNoise)<<"      >> \n"; IDS //mousaab
                            // std::cout<<"\n			    < noise= "<< noise<<"      >> \n"; IDS
                            // //mousaab std::cout<<"			    < CurrentPower= "<<
                            // CurrentPower<<"      >> \n"; IDS //mousaab std::cout<<"\n
                            // < sinr (CurrentPower/interference)= "<< sinr<<"      >> \n"; IDS
                            // //mousaab std::cout<<"			    < 10 log (sinr )= "<< 10 *
                            // log10(sinr)<<"      >> \n"; IDS //mousaab std::cout<<"\n < CN0
                            // (CurrentPower- noise)= "<< CN0<<"      >> \n";
                        }
                    }
                    else
                    {
                        // mousaab std::cout<<"------------>new power < ancienne power  ou different
                        // PRN traquer)\n"; mousaab std::cout<<"ADD new signal to noise\n"; mousaab
                        // std::cout<<"Deleting the packet...\n";

                        // FIFO ou on n'a recu un signal dont le PRN est different au PRN tracker

                        // calculer la nouvelle valeur de correlation du bruit
                        uint32_t localPrn = m_PrnOfMaxPeak;
                        uint32_t PrnXor;
                        uint32_t SumOf32BitsPrnXor;
                        PrnXor = ~(ReceivedPRN) ^ localPrn;
                        PrnXor = PrnXor - ((PrnXor >> 1) & 0x55555555);
                        PrnXor = (PrnXor & 0x33333333) + ((PrnXor >> 2) & 0x33333333);
                        SumOf32BitsPrnXor =
                            (((PrnXor + (PrnXor >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;

                        // calculer la correlation du bruit qu'on doit etre multiplier par 1.0e-6
                        // pour le degrader (car on deja degrader noisePowerDensity )
                        double Correlation = SumOf32BitsPrnXor / (32.0 * 1.0e6);

                        // recuperer une copie du nouveau signal qui va devenir un bruit
                        Ptr<SpectrumValue> NewSignal = lrWpanRxParams->psd->Copy();

                        // calculer le 1er peak
                        // IDS //mousaab std::cout<<"			    < NewSignal avant multiplication
                        // = "<< *(NewSignal)<<"      >> \n";
                        (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 -
                                     2] *= Correlation;
                        (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 -
                                     1] *= Correlation;
                        (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) -
                                     2400] *= Correlation;
                        (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 +
                                     1] *= Correlation;
                        (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 +
                                     2] *= Correlation;
                        // IDS //mousaab std::cout<<"			    < NewSignal apres multiplication
                        // = "<< *(NewSignal)<<"      >> \n";

                        // ajouter le nouveau signal correle au bruit global
                        *m_interferenceAndNoise += *NewSignal;
                        // IDS //mousaab std::cout<<"			    < interference final  = "<<
                        // *(m_interferenceAndNoise)<<"      >> \n";

                        // calculer les parametres de securite
                        // IDS double noise = 10 * log10 (LrWpanSpectrumValueHelper::TotalAvgPower
                        // (m_interferenceAndNoise, m_phyPIBAttributes.phyCurrentChannel)); IDS
                        // double CurrentPower =10 * log10 (LrWpanSpectrumValueHelper::TotalAvgPower
                        // (m_signal->GetSignalPsd (), m_phyPIBAttributes.phyCurrentChannel))-2; IDS
                        // double sinr = LrWpanSpectrumValueHelper::TotalAvgPower
                        // (m_signal->GetSignalPsd(), m_phyPIBAttributes.phyCurrentChannel) /
                        // LrWpanSpectrumValueHelper::TotalAvgPower (m_interferenceAndNoise,
                        // m_phyPIBAttributes.phyCurrentChannel); IDS double CN0 = CurrentPower-
                        // noise;

                        // supprimer le packet
                        m_phyRxDropTrace(p);

                        // IDS //mousaab std::cout<<"		ReceivedPRN = "<< ReceivedPRN<<"      >>
                        // \n"; IDS //mousaab std::cout<<"		ancien  m_PrnOfMaxPeak = "<<
                        // m_PrnOfMaxPeak<<"      >> \n"; IDS //mousaab std::cout<<"		new
                        // Correlation  "<<SumOf32BitsPrnXor/32.0 <<"      >> \n"; IDS //mousaab
                        // std::cout<<"			    < l'ancienne interference  = "<<
                        // *(m_interferenceAndNoise)<<"      >> \n"; IDS //mousaab std::cout<<"
                        // <  l'ancien signal global restera tjr global   = "<<
                        // *(m_signal->GetSignalPsd())<<"      >> \n"; IDS //mousaab std::cout<<"\n
                        // < noise= "<< noise<<"      >> \n"; IDS //mousaab std::cout<<" <
                        // CurrentPower= "<< CurrentPower<<"      >> \n"; IDS //mousaab
                        // std::cout<<"\n			    < sinr (CurrentPower/interference)= "<<
                        // sinr<<"      >> \n"; IDS //mousaab std::cout<<"			    < 10 log
                        // (sinr )= "<< 10 * log10(sinr)<<"      >> \n"; IDS //mousaab
                        // std::cout<<"\n			    < CN0 (CurrentPower- noise)= "<< CN0<<" >>
                        // \n";
                    }
                }
            }
            else
            {
                ////IDS //mousaab std::cout<<"44444 collision + interference (car le recepteur
                /// entraine d'envoyer un msg) \n";
                // Simply drop the packet.
                NS_LOG_DEBUG(this << " transceiver not in RX state");
                m_phyRxDropTrace(p);
                // Add the signal power to the interference, anyway.
                m_signal->AddSignal(lrWpanRxParams->psd);
            }
        }
    }
    else
    {
        // IDS  //mousaab std::cout<<"------------>NON ce n'est mon canal\n";

        if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
        {
            // IDS  //mousaab std::cout<<"------------>canal  busy\n";

            // calculer la nouvelle valeur de correlation du bruit
            uint32_t localPrn = m_PrnOfMaxPeak;
            uint32_t PrnXor;
            uint32_t SumOf32BitsPrnXor;
            PrnXor = ~(ReceivedPRN) ^ localPrn;
            PrnXor = PrnXor - ((PrnXor >> 1) & 0x55555555);
            PrnXor = (PrnXor & 0x33333333) + ((PrnXor >> 2) & 0x33333333);
            SumOf32BitsPrnXor = (((PrnXor + (PrnXor >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;

            // calculer la correlation du bruit qu'on doit etre multiplier par 1.0e-6 pour le
            // degrader (car on deja degrader noisePowerDensity )
            double Correlation = SumOf32BitsPrnXor / (32.0 * 1.0e6);

            // recuperer une copie du nouveau signal qui va devenir un bruit
            Ptr<SpectrumValue> NewSignal = lrWpanRxParams->psd->Copy();

            // calculer le 1er peak
            // IDS //mousaab std::cout<<"			    < NewSignal avant multiplication  = "<<
            // *(NewSignal)<<"      >> \n";
            (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 - 2] *=
                Correlation;
            (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 - 1] *=
                Correlation;
            (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400] *=
                Correlation;
            (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 + 1] *=
                Correlation;
            (*NewSignal)[2405 + 5 * (m_phyPIBAttributes.phyCurrentChannel - 11) - 2400 + 2] *=
                Correlation;
            // IDS //mousaab std::cout<<"			    < NewSignal apres multiplication  = "<<
            // *(NewSignal)<<"      >> \n";

            // ajouter le nouveau signal correle au bruit global
            *m_interferenceAndNoise += *NewSignal;
            // IDS //mousaab std::cout<<"			    < interference final  = "<<
            // *(m_interferenceAndNoise)<<"      >> \n";

            // supprimer le packet
            m_phyRxDropTrace(p);
        }
        else
        {
            // IDS //mousaab std::cout<<"------------>canal  not busy ==> drop ther signal\n";

            ////IDS //mousaab std::cout<<"44444 collision + interference (car le recepteur entraine
            /// d'envoyer un msg) \n";
            // Simply drop the packet.
            NS_LOG_DEBUG(this << " transceiver not in RX state");
            m_phyRxDropTrace(p);
            // Add the signal power to the interference, anyway.
            // m_signal->AddSignal (lrWpanRxParams->psd);
        }
    }

    if (!m_ccaRequest.IsExpired())
    {
        double power =
            LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                     m_phyPIBAttributes.phyCurrentChannel);
        if (m_ccaPeakPower < power)
        {
            m_ccaPeakPower = power;
        }
    }

    // Simulator::Schedule (spectrumRxParams->duration + Seconds (0.16), &LrWpanPhy::EndRx, this,
    // spectrumRxParams); Simulator::Schedule (spectrumRxParams->duration, &LrWpanPhy::EndRx, this,
    // spectrumRxParams);
}

void
LrWpanPhy::StartRx(Ptr<SpectrumSignalParameters> spectrumRxParams)
{
    //////IDS //mousaab std::cout<<"==================================== Debut LrWpanPhy::StartRx
    /// general ====================================\n";

    if ((m_GPSstate == 0) && (m_isSatellite))
    {
        //////IDS //mousaab std::cout<<"==================================== Debut
        /// LrWpanPhy::StartRx (m_GPSstate==0  ====================================\n";
    }
    else
    {
        if (m_isSatellite)
        {
            StartSatelliteRx(spectrumRxParams);
        }
        else
        {
            ////IDS //mousaab std::cout<<"\n\n		*********************  Debut LrWpanPhy:: UAV
            ///*************\n";
            //////IDS //mousaab std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

            ////IDS //mousaab std::cout<<"	m_signal->GetSignalPsd () :" << *(m_signal->GetSignalPsd
            ///())<<"\n";
            //////IDS //mousaab std::cout<<"	spectrumRxParams->psd :" <<
            ///*(spectrumRxParams->psd)<<"\n";

            NS_LOG_FUNCTION(this << spectrumRxParams);
            LrWpanSpectrumValueHelper psdHelper;

            //////IDS //mousaab std::cout<<"		m_trxState :" << m_trxState<<"\n";

            if (!m_edRequest.IsExpired())
            {
                //////IDS //mousaab std::cout<<" Energy detection REQUEST is not expired\n";

                // Update the average receive power during ED.
                Time now = Simulator::Now();
                //////IDS //mousaab std::cout<<"now :" << now<<"\n";
                //////IDS //mousaab std::cout<<"m_edPower.lastUpdate :" <<
                /// m_edPower.lastUpdate<<"\n";
                //////IDS //mousaab std::cout<<"(now - m_edPower.lastUpdate).GetTimeStep () :" <<
                ///(now - m_edPower.lastUpdate).GetTimeStep ()<<"\n";
                //////IDS //mousaab std::cout<<" m_edPower.measurementLength.GetTimeStep () :" <<
                /// m_edPower.measurementLength.GetTimeStep ()<<"\n";
                //////IDS //mousaab std::cout<<"			    < m_signal->GetSignalPsd () = "<<
                ///*(m_signal->GetSignalPsd ())<<"      >> \n";

                //////IDS //mousaab std::cout<<"			    <(now -
                /// m_edPower.lastUpdate).GetTimeStep () = "<<  (now -
                /// m_edPower.lastUpdate).GetTimeStep () <<"      >> \n";
                //////IDS //mousaab std::cout<<"			    <before   m_edPower.averagePower =
                ///"<<  m_edPower.averagePower<<"      >> \n";

                m_edPower.averagePower +=
                    LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                             m_phyPIBAttributes.phyCurrentChannel) *
                    (now - m_edPower.lastUpdate).GetTimeStep() /
                    m_edPower.measurementLength.GetTimeStep();
                m_edPower.lastUpdate = now;
                //////IDS //mousaab std::cout<<"			    < after    m_edPower.averagePower =
                ///"<<  m_edPower.averagePower<<"      >> \n";
                //////IDS //mousaab std::cout<<"			    <m_edPower.lastUpdate = "<<
                /// m_edPower.lastUpdate<<"      >> \n";
                //////IDS //mousaab std::cout<<"			    < spectrumRxParams->psd = "<<
                ///*(spectrumRxParams->psd)<<"      >> \n";

                //////IDS //mousaab std::cout<<"m_edPower.averagePower :" <<
                /// m_edPower.averagePower<<"\n";
            }

            Ptr<LrWpanSpectrumSignalParameters> lrWpanRxParams =
                DynamicCast<LrWpanSpectrumSignalParameters>(spectrumRxParams);

            if (lrWpanRxParams == nullptr)
            {
                //////IDS //mousaab std::cout<<"1111111\n";

                CheckInterference();
                m_signal->AddSignal(spectrumRxParams->psd);

                // Update peak power if CCA is in progress.
                if (!m_ccaRequest.IsExpired())
                {
                    double power = LrWpanSpectrumValueHelper::TotalAvgPower(
                        m_signal->GetSignalPsd(),
                        m_phyPIBAttributes.phyCurrentChannel);
                    //////IDS //mousaab std::cout<<"			    < power= "<< power<<"      >>
                    ///\n";

                    if (m_ccaPeakPower < power)
                    {
                        m_ccaPeakPower = power;
                    }
                }

                Simulator::Schedule(spectrumRxParams->duration,
                                    &LrWpanPhy::EndRx,
                                    this,
                                    spectrumRxParams);
                return;
            }

            Ptr<Packet> p = (lrWpanRxParams->packetBurst->GetPackets()).front();
            NS_ASSERT(p != nullptr);
            //////IDS //mousaab std::cout<<"			    < lrWpanRxParams->psd = "<<
            ///*(lrWpanRxParams->psd)<<"      >> \n";

            // Prevent PHY from receiving another packet while switching the transceiver state.
            if (m_trxState == IEEE_802_15_4_PHY_RX_ON && !m_setTRXState.IsRunning())
            {
                //////IDS //mousaab std::cout<<"222222\n";

                // The specification doesn't seem to refer to BUSY_RX, but vendor
                // data sheets suggest that this is a substate of the RX_ON state
                // that is entered after preamble detection when the digital receiver
                // is enabled.  Here, for now, we use BUSY_RX to mark the period between
                // () and EndRx() states.

                // We are going to BUSY_RX state when receiving the first bit of an SHR,
                // as opposed to real receivers, which should go to this state only after
                // successfully receiving the SHR.

                // If synchronizing to the packet is possible, change to BUSY_RX state,
                // otherwise drop the packet and stay in RX state. The actual synchronization
                // is not modeled.

                // Add any incoming packet to the current interference before checking the
                // SINR.
                // NS_LOG_DEBUG (this << " receiving packet with power: " << 10 *
                // log10(LrWpanSpectrumValueHelper::TotalAvgPower (lrWpanRxParams->psd,
                // m_phyPIBAttributes.phyCurrentChannel)) + 30 << "dBm");

                //////IDS //mousaab std::cout<<"			"<<this << "receiving packet with power:
                ///" << 10 * log10(LrWpanSpectrumValueHelper::TotalAvgPower (lrWpanRxParams->psd,
                /// m_phyPIBAttributes.phyCurrentChannel)) + 30 << "dBm\n";

                m_signal->AddSignal(lrWpanRxParams->psd);
                //////IDS //mousaab std::cout<<"0002\n";

                Ptr<SpectrumValue> interferenceAndNoise = m_signal->GetSignalPsd();
                //////IDS //mousaab std::cout<<"00022\n";
                //////IDS //mousaab std::cout<<"			    < interferenceAndNoise1 = "<<
                ///*(interferenceAndNoise)<<"      >> \n";

                *interferenceAndNoise -= *lrWpanRxParams->psd;
                //////IDS //mousaab std::cout<<"000222\n";
                //////IDS //mousaab std::cout<<"			    < interferenceAndNoise2 = "<<
                ///*(interferenceAndNoise)<<"      >> \n";

                *interferenceAndNoise += *m_noise;
                //////IDS //mousaab std::cout<<"0002222\n";
                //////IDS //mousaab std::cout<<"			    < interferenceAndNoise3 = "<<
                ///*(interferenceAndNoise)<<"      >> \n";

                m_interferenceAndNoise = interferenceAndNoise;
                double sinr =
                    LrWpanSpectrumValueHelper::TotalAvgPower(lrWpanRxParams->psd,
                                                             m_phyPIBAttributes.phyCurrentChannel) /
                    LrWpanSpectrumValueHelper::TotalAvgPower(interferenceAndNoise,
                                                             m_phyPIBAttributes.phyCurrentChannel);
                //////IDS //mousaab std::cout<<"00022222\n";

                //////IDS //mousaab std::cout<<"			    < sinr= "<< sinr<<"      >> \n";

                // Ptr<LrWpanSpectrumSignalParameters> params =
                // DynamicCast<LrWpanSpectrumSignalParameters> (par);

                if (m_currentRxPacket.first == lrWpanRxParams)
                {
                    //////IDS //mousaab std::cout<<"m_currentRxPacket.first  ==  lrWpanRxParams \n";
                }
                else
                {
                    //////IDS //mousaab std::cout<<"m_currentRxPacket.first  !=   lrWpanRxParams\n";
                }

                // Std. 802.15.4-2006, appendix E, Figure E.2
                // At SNR < -5 the BER is less than 10e-1.
                // It's useless to even *try* to decode the packet.

                // //////IDS //mousaab std::cout<<"			    < sinr= "<< sinr<<"      >> \n";
                //////IDS //mousaab std::cout<<"			    < 10 * log10 (sinr)= "<< (10 * log10
                ///(sinr))  <<"      >> \n";

                if (m_isSatellite)
                {
                    ////IDS //mousaab std::cout<<"		^^^^^^^^^ deveice belong to  GNSS System
                    ///(satellite / UAV GPS receiver)   \n";

                    ChangeTrxState(IEEE_802_15_4_PHY_BUSY_RX);
                    m_currentRxPacket = std::make_pair(lrWpanRxParams, false);
                    m_phyRxBeginTrace(p);
                    m_rxLastUpdate = Simulator::Now();
                    Simulator::ScheduleNow(&LrWpanPhy::PlmeEdRequest, this);
                }
                else
                {
                    ////IDS //mousaab std::cout<<"		^^^^^^^^^^^^ simple device for UAV
                    /// communication   \n";

                    if (10 * log10(sinr) > -5)
                    { // mousaab hna la difference entre un signal yal7ag wala mayal7agch puissance
                      // <10db et level = 0
                        // l'uav va recevoir la donnee

                        //////IDS //mousaab std::cout<<"			    10 * log10 (sinr) > -5 \n";

                        //////IDS //mousaab std::cout<<"77777\n";
                        ChangeTrxState(IEEE_802_15_4_PHY_BUSY_RX);
                        // mis a jour du parametre actuel (var global) qui sera utiliser par endrx
                        // pour verifier si on est entraine de traiter le meme packet
                        m_currentRxPacket = std::make_pair(lrWpanRxParams, false);
                        m_phyRxBeginTrace(p);
                        m_rxLastUpdate = Simulator::Now();
                        Simulator::ScheduleNow(&LrWpanPhy::PlmeEdRequest, this);
                    }
                    else
                    {
                        m_phyRxDropTrace(p);
                    }
                }
            }
            else if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
            {
                //////IDS //mousaab std::cout<<"33333 collision , il est entraine de recevoir un
                /// autre paquet \n";

                // Drop the new packet.
                NS_LOG_DEBUG(this << " packet collision");
                m_phyRxDropTrace(p);

                // Check if we correctly received the old packet up to now.
                CheckInterference();

                // Add the incoming packet to the current interference after we have
                // checked for successfull reception of the current packet for the time
                // before the additional interference.
                m_signal->AddSignal(lrWpanRxParams->psd);
            }
            else
            {
                //////IDS //mousaab std::cout<<"44444 collision + interference (car le recepteur
                /// entraine d'envoyer un msg) \n";

                // Simply drop the packet.
                NS_LOG_DEBUG(this << " transceiver not in RX state");
                m_phyRxDropTrace(p);

                // Add the signal power to the interference, anyway.
                m_signal->AddSignal(lrWpanRxParams->psd);
            }

            // Update peak power if CCA is in progress.
            if (!m_ccaRequest.IsExpired())
            {
                //////IDS //mousaab std::cout<<"55555\n";

                double power =
                    LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                             m_phyPIBAttributes.phyCurrentChannel);
                //////IDS //mousaab std::cout<<"			    < power= "<< power<<"      >> \n";

                if (m_ccaPeakPower < power)
                {
                    m_ccaPeakPower = power;
                }
            }
            //////IDS //mousaab std::cout<<"66666\n";

            // Always call EndRx to update the interference.
            // \todo: Do we need to keep track of these events to unschedule them when disposing off
            // the PHY?

            Simulator::Schedule(spectrumRxParams->duration,
                                &LrWpanPhy::EndRx,
                                this,
                                spectrumRxParams);

            /**************************** mousaab 1***********************************/
            // Simulator::ScheduleNow ( &LrWpanPhy::PlmeEdRequest, this);
            /***********************************************************/
            //////IDS //mousaab std::cout<<"		*********************  Fin LrWpanPhy::StartRx
            ///*************\n";

            ////IDS //mousaab
            /// std::cout<<"=============================================================================================\n";
        }
    }
}

void
LrWpanPhy::CheckInterference(void)
{
    //////IDS //mousaab std::cout<<"\n\n		*********************  debut
    /// LrWpanPhy::CheckInterference  *************\n";

    // Calculate whether packet was lost.
    LrWpanSpectrumValueHelper psdHelper;
    Ptr<LrWpanSpectrumSignalParameters> currentRxParams = m_currentRxPacket.first;

    // We are currently receiving a packet.
    if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
    {
        // NS_ASSERT (currentRxParams && !m_currentRxPacket.second);

        Ptr<Packet> currentPacket = currentRxParams->packetBurst->GetPackets().front();
        if (m_errorModel != nullptr)
        {
            // How many bits did we receive since the last calculation?
            double t = (Simulator::Now() - m_rxLastUpdate).ToDouble(Time::MS);
            uint32_t chunkSize = ceil(t * (GetDataOrSymbolRate(true) / 1000));
            Ptr<SpectrumValue> interferenceAndNoise = m_signal->GetSignalPsd();
            *interferenceAndNoise -= *currentRxParams->psd;
            *interferenceAndNoise += *m_noise;
            double sinr =
                LrWpanSpectrumValueHelper::TotalAvgPower(currentRxParams->psd,
                                                         m_phyPIBAttributes.phyCurrentChannel) /
                LrWpanSpectrumValueHelper::TotalAvgPower(interferenceAndNoise,
                                                         m_phyPIBAttributes.phyCurrentChannel);
            //////IDS //mousaab std::cout<<"			    < sinr= "<< sinr<<"      >> \n";

            double per = 1.0 - m_errorModel->GetChunkSuccessRate(sinr, chunkSize);
            //////IDS //mousaab std::cout<<"			    < per= "<< per<<"      >> \n";

            // The LQI is the total packet success rate scaled to 0-255.
            // If not already set, initialize to 255.
            LrWpanLqiTag tag(std::numeric_limits<uint8_t>::max());
            currentPacket->PeekPacketTag(tag);
            uint8_t lqi = tag.Get();
            tag.Set(lqi - (per * lqi));
            currentPacket->ReplacePacketTag(tag);

            if (m_random->GetValue() < per)
            {
                // The packet was destroyed, drop the packet after reception.

                // mousaab
                // ici j'ai supprimer l'instruction (mettre en commentaire) suivante:
                // m_currentRxPacket.second = true;
            }
        }
        else
        {
            NS_LOG_WARN("Missing ErrorModel");
        }
    }
    m_rxLastUpdate = Simulator::Now();
    //////IDS //mousaab std::cout<<"\n\n		*********************  fin
    /// LrWpanPhy::CheckInterference  *************\n";
}

void
LrWpanPhy::EndRx(Ptr<SpectrumSignalParameters> par)
{
    // IDS //mousaab std::cout<<"**********************debut
    // LrWpanPhy::EndRx*********************************\n";

    NS_LOG_FUNCTION(this);

    Ptr<LrWpanSpectrumSignalParameters> params = DynamicCast<LrWpanSpectrumSignalParameters>(par);

    if (!m_edRequest.IsExpired())
    {
        //////IDS //mousaab std::cout<<"		<3<3<3<3<3<3	 Update the average receive power
        /// during ED   <3<3<3<3<3<3 \n";

        // Update the average receive power during ED.
        Time now = Simulator::Now();

        //////IDS //mousaab std::cout<<"now :" << now<<"\n";
        //////IDS //mousaab std::cout<<"m_edPower.lastUpdate :" << m_edPower.lastUpdate<<"\n";
        //////IDS //mousaab std::cout<<"(now - m_edPower.lastUpdate).GetTimeStep () :" << (now -
        /// m_edPower.lastUpdate).GetTimeStep ()<<"\n";
        //////IDS //mousaab std::cout<<" m_edPower.measurementLength.GetTimeStep () :" <<
        /// m_edPower.measurementLength.GetTimeStep ()<<"\n";
        //////IDS //mousaab std::cout<<"m_edPower.averagePower :" << m_edPower.averagePower<<"\n";

        m_edPower.averagePower +=
            LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                     m_phyPIBAttributes.phyCurrentChannel) *
            (now - m_edPower.lastUpdate).GetTimeStep() / m_edPower.measurementLength.GetTimeStep();
        m_edPower.lastUpdate = now;

        //////IDS //mousaab std::cout<<"m_edPower.averagePower :" << m_edPower.averagePower<<"\n";
    }
    //////IDS //mousaab std::cout<<"	11111 \n";

    Ptr<LrWpanSpectrumSignalParameters> currentRxParams = m_currentRxPacket.first;

    if (currentRxParams == params)
    {
        // IDS //mousaab std::cout<<"		currentRxParams == params \n";

        CheckInterference();
    }
    //////IDS //mousaab std::cout<<"	Update the interference ==> m_signal->RemoveSignal
    ///(par->psd) \n";

    // Update the interference.
    // IDS //mousaab std::cout<<"	avant m_signal->RemoveSignal () :" << *(m_signal->GetSignalPsd
    // ())<<"\n";

    m_signal->RemoveSignal(par->psd);
    // IDS //mousaab std::cout<<"	apres m_signal->RemoveSignal () :" << *(m_signal->GetSignalPsd
    // ())<<"\n";

    if (params == nullptr)
    {
        ////IDS std::cout<< "Node: " << m_device->GetAddress() << " Removing interferent: " <<
        ///*(par->psd)<<" \n";

        NS_LOG_LOGIC("Node: " << m_device->GetAddress()
                              << " Removing interferent: " << *(par->psd));
        return;
    }
    //////IDS //mousaab std::cout<<"	3333 \n";

    // If this is the end of the currently received packet, check if reception was successful.
    if (currentRxParams == params)
    {
        // IDS //mousaab std::cout<<"	 If this is the end of the currently received packet, check
        // if reception was successful  \n";

        Ptr<Packet> currentPacket = currentRxParams->packetBurst->GetPackets().front();
        NS_ASSERT(currentPacket != nullptr);

        // If there is no error model attached to the PHY, we always report the maximum LQI value.
        LrWpanLqiTag tag(std::numeric_limits<uint8_t>::max());
        currentPacket->PeekPacketTag(tag);
        m_phyRxEndTrace(currentPacket, tag.Get());

        // IDS //mousaab std::cout<<"			tag.Get () :" << (int)tag.Get ()<<" dB \n";

        if (!m_currentRxPacket.second)
        {
            // IDS //mousaab std::cout<<"		<3<3<3<3<3<3	 rah yatab3eth 2   <3<3<3<3<3<3 \n";

            // The packet was successfully received, push it up the stack.
            if (!m_pdDataIndicationCallback.IsNull())
            {
                // mousaab (fin) le vrai appel d'indication
                // IDS //mousaab std::cout<<"		The packet was successfully received \n";

                m_pdDataIndicationCallback(currentPacket->GetSize(), currentPacket, tag.Get());
            }
            else
            {
                // IDS //mousaab std::cout<<"		The packet was successfully received / but
                // callback = null \n";
            }
        }
        else
        {
            // The packet was destroyed, drop it.
            // IDS //mousaab std::cout<<"		 the frame is either invalid or destroyed due to
            // interference ==>  packet was destroyed, drop it.\n";

            m_phyRxDropTrace(currentPacket);
        }
        Ptr<LrWpanSpectrumSignalParameters> none = 0;
        m_currentRxPacket = std::make_pair(none, true);

        // We may be waiting to apply a pending state change.
        if (m_trxStatePending != IEEE_802_15_4_PHY_IDLE)
        {
            // IDS //mousaab std::cout<<"		<3<3<3<3<3<3	mahoch rah yatab3eth 5 <3<3<3<3<3<3
            // \n";

            // Only change the state immediately, if the transceiver is not already
            // switching the state.
            if (!m_setTRXState.IsRunning())
            {
                // IDS //mousaab std::cout<<"		<3<3<3<3<3<3	mahoch rah yatab3eth 6
                // <3<3<3<3<3<3 \n";

                NS_LOG_LOGIC("Apply pending state change to " << m_trxStatePending);
                ChangeTrxState(m_trxStatePending);
                m_trxStatePending = IEEE_802_15_4_PHY_IDLE;
                if (!m_plmeSetTRXStateConfirmCallback.IsNull())
                {
                    //////IDS //mousaab std::cout<<"		<3<3<3<3<3<3	mahoch rah yatab3eth 7
                    ///< 3<3<3<3<3<3 \n";

                    m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_SUCCESS);
                }
            }
        }
        else
        {
            // IDS //mousaab std::cout<<"		IEEE_802_15_4_PHY_IDLE \n";

            ChangeTrxState(IEEE_802_15_4_PHY_RX_ON);
        }
    }
    else
    {
        // IDS //mousaab std::cout<<"	currentRxParams != params \n";
    }
    // IDS //mousaab std::cout<<"**********************fin
    // LrWpanPhy::EndRx*********************************\n";
}

void
LrWpanPhy::PdDataRequest(const uint32_t psduLength, Ptr<Packet> p)
{
    //////IDS //mousaab std::cout<<"XXXXXXXXXXXXXXXXXXX debut  LrWpanPhy::PdDataRequest
    /// XXXXXXXXXXXX\n";

    NS_LOG_FUNCTION(this << psduLength << p);

    if (psduLength > aMaxPhyPacketSize)
    {
        //////IDS //mousaab std::cout<<"       psduLength > aMaxPhyPacketSize\n";

        if (!m_pdDataConfirmCallback.IsNull())
        {
            m_pdDataConfirmCallback(IEEE_802_15_4_PHY_UNSPECIFIED);
        }
        NS_LOG_DEBUG("Drop packet because psduLength too long: " << psduLength);
        return;
    }
    else
    {
        //////IDS //mousaab std::cout<<"       psduLength goooood\n";
    }

    // Prevent PHY from sending a packet while switching the transceiver state.
    if (!m_setTRXState.IsRunning())
    {
        //////IDS //mousaab std::cout<<"       m_setTRXState is not running\n";

        if (m_trxState == IEEE_802_15_4_PHY_TX_ON)
        {
            //////IDS //mousaab std::cout<<"            m_trxState == IEEE_802_15_4_PHY_TX_ON\n";
            // send down
            NS_ASSERT(m_channel);

            // Remove a possible LQI tag from a previous transmission of the packet.
            //////IDS //mousaab std::cout<<"            Remove a possible LQI tag from a previous
            /// transmission of the packet\n";

            LrWpanLqiTag lqiTag;
            p->RemovePacketTag(lqiTag);

            m_phyTxBeginTrace(p);
            m_currentTxPacket.first = p;
            m_currentTxPacket.second = false;

            //////IDS //mousaab std::cout<<"         txParams config\n";

            Ptr<LrWpanSpectrumSignalParameters> txParams = Create<LrWpanSpectrumSignalParameters>();
            txParams->duration = CalculateTxTime(p);
            txParams->txPhy = GetObject<SpectrumPhy>();
            txParams->psd = m_txPsd;
            txParams->txAntenna = m_antenna;
            //////IDS //mousaab std::cout<<"          ---  duration = "<<txParams->duration<<"\n";
            //////IDS //mousaab std::cout<<"          ---  txPhy = "<<txParams->txPhy<<"\n";
            //////IDS //mousaab std::cout<<"          ---  psd = "<<*(txParams->psd)<<"\n";
            //////IDS //mousaab std::cout<<"          ---  txAntenna = "<<txParams->txAntenna<<"\n";

            //////IDS //mousaab std::cout<<"            operation de BURST  \n";

            Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
            pb->AddPacket(p);
            txParams->packetBurst = pb;

            //////IDS //mousaab std::cout<<"           depart de transmission pendant une duree de
            ///"<<txParams->duration<<"  \n";
            m_channel->StartTx(txParams);

            m_pdDataRequest = Simulator::Schedule(txParams->duration, &LrWpanPhy::EndTx, this);

            //////IDS //mousaab std::cout<<"            ChangeTrxState (IEEE_802_15_4_PHY_BUSY_TX)
            ///\n";

            ChangeTrxState(IEEE_802_15_4_PHY_BUSY_TX);
            //////IDS //mousaab std::cout<<"XXXXXXXXXXXXXXXXXXX fin1  LrWpanPhy::PdDataRequest
            /// XXXXXXXXXXXX\n";

            return;
        }
        else if ((m_trxState == IEEE_802_15_4_PHY_RX_ON) ||
                 (m_trxState == IEEE_802_15_4_PHY_TRX_OFF) ||
                 (m_trxState == IEEE_802_15_4_PHY_BUSY_TX))
        {
            if (!m_pdDataConfirmCallback.IsNull())
            {
                m_pdDataConfirmCallback(m_trxState);
            }
            // Drop packet, hit PhyTxDrop trace
            m_phyTxDropTrace(p);
            //////IDS //mousaab std::cout<<"XXXXXXXXXXXXXXXXXXX fin2  LrWpanPhy::PdDataRequest
            /// XXXXXXXXXXXX\n";

            return;
        }
        else
        {
            NS_FATAL_ERROR("This should be unreachable, or else state "
                           << m_trxState << " should be added as a case");
        }
    }
    else
    {
        // TODO: This error code is not covered by the standard.
        // What is the correct behavior in this case?
        if (!m_pdDataConfirmCallback.IsNull())
        {
            m_pdDataConfirmCallback(IEEE_802_15_4_PHY_UNSPECIFIED);
        }
        // Drop packet, hit PhyTxDrop trace
        m_phyTxDropTrace(p);
        //////IDS //mousaab std::cout<<"XXXXXXXXXXXXXXXXXXX fin3  LrWpanPhy::PdDataRequest
        /// XXXXXXXXXXXX\n";

        return;
    }
}

void
LrWpanPhy::PlmeCcaRequest(void)
{
    NS_LOG_FUNCTION(this);

    if (m_trxState == IEEE_802_15_4_PHY_RX_ON || m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
    {
        m_ccaPeakPower = 0.0;
        Time ccaTime = Seconds(8.0 / GetDataOrSymbolRate(false));
        m_ccaRequest = Simulator::Schedule(ccaTime, &LrWpanPhy::EndCca, this);
    }
    else
    {
        if (!m_plmeCcaConfirmCallback.IsNull())
        {
            if (m_trxState == IEEE_802_15_4_PHY_TRX_OFF)
            {
                m_plmeCcaConfirmCallback(IEEE_802_15_4_PHY_TRX_OFF);
            }
            else
            {
                m_plmeCcaConfirmCallback(IEEE_802_15_4_PHY_BUSY);
            }
        }
    }
}

void
LrWpanPhy::CcaCancel()
{
    NS_LOG_FUNCTION(this);
    m_ccaRequest.Cancel();
}

void
LrWpanPhy::PlmeSatelliteEdRequest(Time duration, Ptr<SpectrumSignalParameters> spectrumRxParams)
{
    // IDS //mousaab std::cout<<"**********************debut
    // LrWpanPhy::PlmeSatelliteEdRequest*********************************\n";

    NS_LOG_FUNCTION(this);

    //////IDS //mousaab std::cout<<"	m_trxState  :" <<  m_trxState <<"\n";

    if (m_trxState == IEEE_802_15_4_PHY_RX_ON || m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
    {
        //////IDS //mousaab std::cout<<"1111111111111\n";
        // Average over the powers of all signals received until EndEd()
        m_edPower.averagePower = 0;
        m_InterferenceAvg = 0;
        m_edPower.lastUpdate = Simulator::Now();

        m_edPower.measurementLength = Seconds(0.01); // satellite rate =50hz ( bspk sssd )

        // m_edPower.measurementLength=duration;

        // IDS //mousaab std::cout<<"	m_edPower.measurementLength  :" <<
        // m_edPower.measurementLength <<"\n"; IDS //mousaab std::cout<<"	time of reception  :" <<
        // Seconds (0.6) <<"\n";

        m_edSatelliteRequest =
            Simulator::Schedule(m_edPower.measurementLength, &LrWpanPhy::EndEd, this);
        m_EndRxSatelliteRequest =
            Simulator::Schedule(Seconds(0.6), &LrWpanPhy::EndRx, this, spectrumRxParams);
    }
    else
    {
        //////IDS //mousaab std::cout<<"222222222\n";

        LrWpanPhyEnumeration result = m_trxState;
        if (m_trxState == IEEE_802_15_4_PHY_BUSY_TX)
        {
            //////IDS //mousaab std::cout<<"333333333\n";

            result = IEEE_802_15_4_PHY_TX_ON;
        }

        if (!m_plmeEdConfirmCallback.IsNull())
        {
            //////IDS //mousaab std::cout<<"4444\n";

            m_plmeEdConfirmCallback(result, 0);
            m_plmeEdConfirmCallback_V2(result, 0, 0, 0, 0);
        }
    }
    // IDS //mousaab std::cout<<"**********************fin
    // LrWpanPhy::PlmeSatelliteEdRequest*********************************\n";
}

void
LrWpanPhy::PlmeEdRequest(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::PlmeEdRequest*********************************\n";

    NS_LOG_FUNCTION(this);

    //////IDS //mousaab std::cout<<"	m_trxState  :" <<  m_trxState <<"\n";

    if (m_trxState == IEEE_802_15_4_PHY_RX_ON || m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
    {
        //////IDS //mousaab std::cout<<"1111111111111\n";
        // Average over the powers of all signals received until EndEd()
        m_edPower.averagePower = 0;
        m_InterferenceAvg = 0;
        m_edPower.lastUpdate = Simulator::Now();

        // measurementLength : c'est le temps d'un seul octet
        if (m_isSatellite)
        {
            // m_edPower.measurementLength = Seconds (8.0 / GetDataOrSymbolRate (false));

            m_edPower.measurementLength = Seconds(8.0 / 50.0); // satellite rate =50hz ( bspk sssd )
        }
        else
        {
            m_edPower.measurementLength = Seconds(8.0 / GetDataOrSymbolRate(false));
        }
        //////IDS //mousaab std::cout<<"m_edPower.measurementLength  :" <<
        /// m_edPower.measurementLength <<"\n";
        //////IDS //mousaab std::cout<<"now :" << Simulator::Now ()<<"\n";
        //////IDS //mousaab std::cout<<"m_edPower.lastUpdate :" << m_edPower.lastUpdate<<"\n";
        //////IDS //mousaab std::cout<<" m_edPower.measurementLength.GetTimeStep () :" <<
        /// m_edPower.measurementLength.GetTimeStep ()<<"\n";
        //////IDS //mousaab std::cout<<"m_edPower.averagePower :" << m_edPower.averagePower<<"\n";

        //////IDS //mousaab std::cout<<" accumuler la puissance durant :" <<
        /// m_edPower.measurementLength<< " seconde \n";

        m_edRequest = Simulator::Schedule(m_edPower.measurementLength, &LrWpanPhy::EndEd, this);
    }
    else
    {
        //////IDS //mousaab std::cout<<"222222222\n";

        LrWpanPhyEnumeration result = m_trxState;
        if (m_trxState == IEEE_802_15_4_PHY_BUSY_TX)
        {
            //////IDS //mousaab std::cout<<"333333333\n";

            result = IEEE_802_15_4_PHY_TX_ON;
        }

        if (!m_plmeEdConfirmCallback.IsNull())
        {
            //////IDS //mousaab std::cout<<"4444\n";

            m_plmeEdConfirmCallback(result, 0);
            m_plmeEdConfirmCallback_V2(result, 0, 0, 0, 0);
        }
    }
    ////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::PlmeEdRequest*********************************\n";
}

/*
void
LrWpanPhy::PlmeGetAttributeRequest(LrWpanPibAttributeIdentifier id)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::PlmeGetAttributeRequest*********************************\n";

    NS_LOG_FUNCTION(this << id);
    LrWpanPhyEnumeration status;

    switch (id)
    {
    case phyCurrentChannel:
    case phyChannelsSupported:
    case phyTransmitPower:
    case phyCCAMode:
    case phyCurrentPage:
    case phyMaxFrameDuration:
    case phySHRDuration:
    case phySymbolsPerOctet: {
        status = IEEE_802_15_4_PHY_SUCCESS;
        break;
    }
    
    case phyCurrentChannel:
        attributes->phyCurrentChannel = m_phyPIBAttributes.phyCurrentChannel;
        break;
    case phyCurrentPage:
        attributes->phyCurrentPage = m_phyPIBAttributes.phyCurrentPage;
        break;
    case phySHRDuration:
        attributes->phySHRDuration = GetPhySHRDuration();
        break;
    case phySymbolsPerOctet:
        attributes->phySymbolsPerOctet = GetPhySymbolsPerOctet();
        break;

    default: {
        status = IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE;
        break;
    }

    }
    if (!m_plmeGetAttributeConfirmCallback.IsNull())
    {
        LrWpanPhyPibAttributes retValue;
        memcpy(&retValue, &m_phyPIBAttributes, sizeof(LrWpanPhyPibAttributes));
        m_plmeGetAttributeConfirmCallback(status, id, &retValue);
    }

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::PlmeGetAttributeRequest*********************************\n";
}
*/

void
LrWpanPhy::PlmeGetAttributeRequest(LrWpanPibAttributeIdentifier id)
{
    NS_LOG_FUNCTION(this << id);
    LrWpanPhyEnumeration status = IEEE_802_15_4_PHY_SUCCESS;
    Ptr<LrWpanPhyPibAttributes> attributes = Create<LrWpanPhyPibAttributes>();

    switch (id)
    {
    case phyCurrentChannel:
        attributes->phyCurrentChannel = m_phyPIBAttributes.phyCurrentChannel;
        break;
    case phyCurrentPage:
        attributes->phyCurrentPage = m_phyPIBAttributes.phyCurrentPage;
        break;
    case phySHRDuration:
        attributes->phySHRDuration = GetPhySHRDuration();
        break;
    case phySymbolsPerOctet:
        attributes->phySymbolsPerOctet = GetPhySymbolsPerOctet();
        break;
    default:
        status = IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE;
        break;
    }

    if (!m_plmeGetAttributeConfirmCallback.IsNull())
    {
        m_plmeGetAttributeConfirmCallback(status, id, attributes);
    }
}

// Section 6.2.2.7.3
void
LrWpanPhy::PlmeSetTRXStateRequest(LrWpanPhyEnumeration state)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::PlmeSetTRXStateRequest*********************************\n";
    NS_LOG_FUNCTION(this << state);

    // Check valid states (Table 14)
    NS_ABORT_IF((state != IEEE_802_15_4_PHY_RX_ON) && (state != IEEE_802_15_4_PHY_TRX_OFF) &&
                (state != IEEE_802_15_4_PHY_FORCE_TRX_OFF) && (state != IEEE_802_15_4_PHY_TX_ON));

    NS_LOG_LOGIC("Trying to set m_trxState from " << m_trxState << " to " << state);

    //////IDS //mousaab std::cout<<"Trying to set m_trxState from " << m_trxState << " to " <<
    /// state<<"\n";

    // this method always overrides previous state setting attempts
    if (!m_setTRXState.IsExpired())
    {
        //////IDS //mousaab std::cout<<"1111111\n";

        if (m_trxStatePending == state)
        {
            //////IDS //mousaab std::cout<<"2222222\n";

            // Simply wait for the ongoing state switch.
            return;
        }
        else
        {
            //////IDS //mousaab std::cout<<"3333333\n";

            NS_LOG_DEBUG("Cancel m_setTRXState");
            // Keep the transceiver state as the old state before the switching attempt.
            m_setTRXState.Cancel();
        }
    }
    if (m_trxStatePending != IEEE_802_15_4_PHY_IDLE)
    {
        //////IDS //mousaab std::cout<<"44444\n";

        m_trxStatePending = IEEE_802_15_4_PHY_IDLE;
    }

    if (state == m_trxState)
    {
        //////IDS //mousaab std::cout<<"5555\n";

        if (!m_plmeSetTRXStateConfirmCallback.IsNull())
        {
            //////IDS //mousaab std::cout<<"6666666\n";

            m_plmeSetTRXStateConfirmCallback(state);
        }
        return;
    }

    if (((state == IEEE_802_15_4_PHY_RX_ON) || (state == IEEE_802_15_4_PHY_TRX_OFF)) &&
        (m_trxState == IEEE_802_15_4_PHY_BUSY_TX))
    {
        //////IDS //mousaab std::cout<<"7777777\n";

        NS_LOG_DEBUG("Phy is busy; setting state pending to " << state);
        m_trxStatePending = state;
        return; // Send PlmeSetTRXStateConfirm later
    }

    // specification talks about being in RX_ON and having received
    // a valid SFD.  Here, we are not modelling at that level of
    // granularity, so we just test for BUSY_RX state (any part of
    // a packet being actively received)
    if (state == IEEE_802_15_4_PHY_TRX_OFF)
    {
        //////IDS //mousaab std::cout<<"888888\n";

        CancelEd(state);

        if ((m_trxState == IEEE_802_15_4_PHY_BUSY_RX) && (m_currentRxPacket.first) &&
            (!m_currentRxPacket.second))
        {
            //////IDS //mousaab std::cout<<"999999\n";

            NS_LOG_DEBUG("Receiver has valid SFD; defer state change");
            m_trxStatePending = state;
            return; // Send PlmeSetTRXStateConfirm later
        }
        else if (m_trxState == IEEE_802_15_4_PHY_RX_ON || m_trxState == IEEE_802_15_4_PHY_TX_ON)
        {
            //////IDS //mousaab std::cout<<"10101010\n";

            ChangeTrxState(IEEE_802_15_4_PHY_TRX_OFF);
            if (!m_plmeSetTRXStateConfirmCallback.IsNull())
            {
                m_plmeSetTRXStateConfirmCallback(state);
            }
            return;
        }
    }

    if (state == IEEE_802_15_4_PHY_TX_ON)
    {
        //////IDS //mousaab std::cout<<"12121212\n";

        CancelEd(state);
        NS_LOG_DEBUG("turn on PHY_TX_ON");
        if ((m_trxState == IEEE_802_15_4_PHY_BUSY_RX) || (m_trxState == IEEE_802_15_4_PHY_RX_ON))
        {
            if (m_currentRxPacket.first)
            {
                // terminate reception if needed
                // incomplete reception -- force packet discard
                NS_LOG_DEBUG("force TX_ON, terminate reception");
                m_currentRxPacket.second = true;
            }

            // If CCA is in progress, cancel CCA and return BUSY.
            if (!m_ccaRequest.IsExpired())
            {
                m_ccaRequest.Cancel();
                if (!m_plmeCcaConfirmCallback.IsNull())
                {
                    m_plmeCcaConfirmCallback(IEEE_802_15_4_PHY_BUSY);
                }
            }

            m_trxStatePending = IEEE_802_15_4_PHY_TX_ON;

            // Delay for turnaround time
            // TODO: Does it also take aTurnaroundTime to switch the transceiver state,
            //       even when the receiver is not busy? (6.9.2)
            Time setTime = Seconds((double)aTurnaroundTime / GetDataOrSymbolRate(false));
            m_setTRXState = Simulator::Schedule(setTime, &LrWpanPhy::EndSetTRXState, this);
            return;
        }
        else if (m_trxState == IEEE_802_15_4_PHY_BUSY_TX || m_trxState == IEEE_802_15_4_PHY_TX_ON)
        {
            // We do NOT change the transceiver state here. We only report that
            // the transceiver is already in TX_ON state.
            if (!m_plmeSetTRXStateConfirmCallback.IsNull())
            {
                m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_TX_ON);
            }
            return;
        }
        else if (m_trxState == IEEE_802_15_4_PHY_TRX_OFF)
        {
            ChangeTrxState(IEEE_802_15_4_PHY_TX_ON);
            if (!m_plmeSetTRXStateConfirmCallback.IsNull())
            {
                m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_TX_ON);
            }
            return;
        }
    }

    if (state == IEEE_802_15_4_PHY_FORCE_TRX_OFF)
    {
        ////IDS //mousaab std::cout<<"131313131\n";

        if (m_trxState == IEEE_802_15_4_PHY_TRX_OFF)
        {
            NS_LOG_DEBUG("force TRX_OFF, was already off");
        }
        else
        {
            NS_LOG_DEBUG("force TRX_OFF, SUCCESS");
            if (m_currentRxPacket.first)
            { // terminate reception if needed
                // incomplete reception -- force packet discard
                NS_LOG_DEBUG("force TRX_OFF, terminate reception");
                m_currentRxPacket.second = true;
            }
            if (m_trxState == IEEE_802_15_4_PHY_BUSY_TX)
            {
                NS_LOG_DEBUG("force TRX_OFF, terminate transmission");
                m_currentTxPacket.second = true;
            }
            ChangeTrxState(IEEE_802_15_4_PHY_TRX_OFF);
            // Clear any other state
            m_trxStatePending = IEEE_802_15_4_PHY_IDLE;
        }
        if (!m_plmeSetTRXStateConfirmCallback.IsNull())
        {
            m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_SUCCESS);
        }
        return;
    }

    if (state == IEEE_802_15_4_PHY_RX_ON)
    {
        //////IDS //mousaab std::cout<<"14141414\n";

        if (m_trxState == IEEE_802_15_4_PHY_TX_ON || m_trxState == IEEE_802_15_4_PHY_TRX_OFF)
        {
            // Turnaround delay
            // TODO: Does it really take aTurnaroundTime to switch the transceiver state,
            //       even when the transmitter is not busy? (6.9.1)
            m_trxStatePending = IEEE_802_15_4_PHY_RX_ON;

            Time setTime = Seconds((double)aTurnaroundTime / GetDataOrSymbolRate(false));
            m_setTRXState = Simulator::Schedule(setTime, &LrWpanPhy::EndSetTRXState, this);
            return;
        }
        else if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
        {
            if (!m_plmeSetTRXStateConfirmCallback.IsNull())
            {
                m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_RX_ON);
            }
            return;
        }
    }

    NS_FATAL_ERROR("Unexpected transition from state " << m_trxState << " to state " << state);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::PlmeSetTRXStateRequest*********************************\n";

}

bool
LrWpanPhy::ChannelSupported(uint8_t channel)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::ChannelSupported*********************************\n";

    NS_LOG_FUNCTION(this << channel);
    bool retValue = false;

    for (uint32_t i = 0; i < 32; i++)
    {
        if ((m_phyPIBAttributes.phyChannelsSupported[i] & (1 << channel)) != 0)
        {
            retValue = true;
            break;
        }
    }

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::ChannelSupported*********************************\n";

    return retValue;
}

bool
LrWpanPhy::PageSupported(uint8_t page)
{
    NS_LOG_FUNCTION(this << +page);
    bool retValue = false;

    // TODO: Only O-QPSK 2.4GHz is supported in the LrWpanSpectrumModel
    //       we must limit the page until support for other modulation is added to the spectrum
    //       model.
    //
    NS_ABORT_MSG_UNLESS(page == 0, " Only Page 0 (2.4Ghz O-QPSK supported).");

    // IEEE 802.15.4-2006, Table 23 phyChannelsSupported   Bits 27-31  (5 MSB)
    uint8_t supportedPage = (m_phyPIBAttributes.phyChannelsSupported[page] >> 27) & (0x1F);

    if (page == supportedPage)
    {
        retValue = true;
    }

    return retValue;
}

void
LrWpanPhy::PlmeSetAttributeRequest(LrWpanPibAttributeIdentifier id,
                                   Ptr<LrWpanPhyPibAttributes> attribute)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::PlmeSetAttributeRequest*********************************\n";

    NS_LOG_FUNCTION(this << id << attribute);
    NS_ASSERT(attribute);
    LrWpanPhyEnumeration status = IEEE_802_15_4_PHY_SUCCESS;

    switch (id)
    {
    case phyCurrentChannel: {
        if (!ChannelSupported(attribute->phyCurrentChannel))
        {
            status = IEEE_802_15_4_PHY_INVALID_PARAMETER;
        }
        if (m_phyPIBAttributes.phyCurrentChannel != attribute->phyCurrentChannel)
        {
            // Cancel a pending transceiver state change.
            // Switch off the transceiver.
            // TODO: Is switching off the transceiver the right choice?
            m_trxState = IEEE_802_15_4_PHY_TRX_OFF;
            if (m_trxStatePending != IEEE_802_15_4_PHY_IDLE)
            {
                m_trxStatePending = IEEE_802_15_4_PHY_IDLE;
                m_setTRXState.Cancel();
                if (!m_plmeSetTRXStateConfirmCallback.IsNull())
                {
                    m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_TRX_OFF);
                }
            }

            // Any packet in transmission or reception will be corrupted.
            if (m_currentRxPacket.first)
            {
                m_currentRxPacket.second = true;
            }
            if (PhyIsBusy())
            {
                m_currentTxPacket.second = true;
                m_pdDataRequest.Cancel();
                m_currentTxPacket.first = 0;
                if (!m_pdDataConfirmCallback.IsNull())
                {
                    m_pdDataConfirmCallback(IEEE_802_15_4_PHY_TRX_OFF);
                }
            }
            m_phyPIBAttributes.phyCurrentChannel = attribute->phyCurrentChannel;
            LrWpanSpectrumValueHelper psdHelper;
            m_txPsd = psdHelper.CreateTxPowerSpectralDensity(m_phyPIBAttributes.phyTransmitPower,
                                                             m_phyPIBAttributes.phyCurrentChannel);

            //////IDS //mousaab std::cout<<"	2		    <m_phyPIBAttributes.phyTransmitPower =
            ///"<< (uint32_t)m_phyPIBAttributes.phyTransmitPower<<"      >> \n";
            //////IDS //mousaab std::cout<<"	2		    <m_phyPIBAttributes.phyCurrentChannel =
            ///"<< (uint32_t)m_phyPIBAttributes.phyCurrentChannel<<"      >> \n";
        }
        break;
    }
    case phyChannelsSupported: { // only the first element is considered in the array
        if ((attribute->phyChannelsSupported[0] & 0xf8000000) != 0)
        { // 5 MSBs reserved
            status = IEEE_802_15_4_PHY_INVALID_PARAMETER;
        }
        else
        {
            m_phyPIBAttributes.phyChannelsSupported[0] = attribute->phyChannelsSupported[0];
        }
        break;
    }
    case phyTransmitPower: {
        if (attribute->phyTransmitPower > 0xbf)
        {
            status = IEEE_802_15_4_PHY_INVALID_PARAMETER;
        }
        else
        {
            m_phyPIBAttributes.phyTransmitPower = attribute->phyTransmitPower;
            LrWpanSpectrumValueHelper psdHelper;
            m_txPsd = psdHelper.CreateTxPowerSpectralDensity(m_phyPIBAttributes.phyTransmitPower,
                                                             m_phyPIBAttributes.phyCurrentChannel);

            //////IDS //mousaab std::cout<<"	3		    <m_phyPIBAttributes.phyTransmitPower =
            ///"<< (uint32_t)m_phyPIBAttributes.phyTransmitPower<<"      >> \n";
            //////IDS //mousaab std::cout<<"	3		    <m_phyPIBAttributes.phyCurrentChannel =
            ///"<< (uint32_t)m_phyPIBAttributes.phyCurrentChannel<<"      >> \n";
        }
        break;
    }
    case phyCCAMode: {
        if ((attribute->phyCCAMode < 1) || (attribute->phyCCAMode > 3))
        {
            status = IEEE_802_15_4_PHY_INVALID_PARAMETER;
        }
        else
        {
            m_phyPIBAttributes.phyCCAMode = attribute->phyCCAMode;
        }
        break;
    }
    default: {
        status = IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE;
        break;
    }
    }

    if (!m_plmeSetAttributeConfirmCallback.IsNull())
    {
        m_plmeSetAttributeConfirmCallback(status, id);
    }

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::PlmeSetAttributeRequest*********************************\n";
}

void
LrWpanPhy::SetPdDataIndicationCallback(PdDataIndicationCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPdDataIndicationCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_pdDataIndicationCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPdDataIndicationCallback*********************************\n";
}

void
LrWpanPhy::SetPdDataConfirmCallback(PdDataConfirmCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPdDataConfirmCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_pdDataConfirmCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPdDataConfirmCallback*********************************\n";
}

void
LrWpanPhy::SetPlmeCcaConfirmCallback(PlmeCcaConfirmCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPlmeCcaConfirmCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_plmeCcaConfirmCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPlmeCcaConfirmCallback*********************************\n";
}

void
LrWpanPhy::SetPlmeEdConfirmCallback(PlmeEdConfirmCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPlmeEdConfirmCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_plmeEdConfirmCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPlmeEdConfirmCallback*********************************\n";
}

// mousaab
void
LrWpanPhy::SetPlmeEdConfirmCallback_V2(PlmeEdConfirmCallback_V2 c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPlmeEdConfirmCallback_V2*********************************\n";

    NS_LOG_FUNCTION(this);
    m_plmeEdConfirmCallback_V2 = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPlmeEdConfirmCallback_V2*********************************\n";
}

void
LrWpanPhy::SetPlmeGetAttributeConfirmCallback(PlmeGetAttributeConfirmCallback c)
{
    NS_LOG_FUNCTION(this);
    m_plmeGetAttributeConfirmCallback = c;
}

void
LrWpanPhy::SetPlmeSetTRXStateConfirmCallback(PlmeSetTRXStateConfirmCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPlmeSetTRXStateConfirmCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_plmeSetTRXStateConfirmCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPlmeSetTRXStateConfirmCallback*********************************\n";
}

void
LrWpanPhy::SetPlmeSetAttributeConfirmCallback(PlmeSetAttributeConfirmCallback c)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetPlmeSetAttributeConfirmCallback*********************************\n";

    NS_LOG_FUNCTION(this);
    m_plmeSetAttributeConfirmCallback = c;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetPlmeSetAttributeConfirmCallback*********************************\n";
}

void
LrWpanPhy::ChangeTrxState(LrWpanPhyEnumeration newState)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::ChangeTrxState*********************************\n";

    NS_LOG_LOGIC(this << " state: " << m_trxState << " -> " << newState);
    m_trxStateLogger(Simulator::Now(), m_trxState, newState);
    m_trxState = newState;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::ChangeTrxState*********************************\n";
}

bool
LrWpanPhy::PhyIsBusy(void) const
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::PhyIsBusy*********************************\n";

    NS_LOG_FUNCTION(this << m_trxState);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::PhyIsBusy*********************************\n";

    return ((m_trxState == IEEE_802_15_4_PHY_BUSY_TX) ||
            (m_trxState == IEEE_802_15_4_PHY_BUSY_RX) || (m_trxState == IEEE_802_15_4_PHY_BUSY));
}

void
LrWpanPhy::CancelEd(LrWpanPhyEnumeration state)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::CancelEd*********************************\n";

    NS_LOG_FUNCTION(this);
    NS_ASSERT(state == IEEE_802_15_4_PHY_TRX_OFF || state == IEEE_802_15_4_PHY_TX_ON);

    if (!m_edRequest.IsExpired())
    {
        m_edRequest.Cancel();
        if (!m_plmeEdConfirmCallback.IsNull())
        {
            m_plmeEdConfirmCallback(state, 0);
            m_plmeEdConfirmCallback_V2(state, 0, 0, 0, 0);
        }
    }
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::CancelEd*********************************\n";
}

void
LrWpanPhy::EndEd(void)
{
    NS_LOG_FUNCTION(this);
    // mousaab 3
    // IDS //mousaab std::cout<<"**********************debut
    // LrWpanPhy::EndEd*********************************\n";

    // IDS //mousaab std::cout<<"now :" << Simulator::Now ()<<"\n";
    //////IDS //mousaab std::cout<<"m_edPower.lastUpdate :" << m_edPower.lastUpdate<<"\n";
    //////IDS //mousaab std::cout<<"(now - m_edPower.lastUpdate).GetTimeStep () :" <<
    ///(Simulator::Now () - m_edPower.lastUpdate).GetTimeStep ()<<"\n";
    //////IDS //mousaab std::cout<<" m_edPower.measurementLength.GetTimeStep () :" <<
    /// m_edPower.measurementLength.GetTimeStep ()<<"\n";
    //////IDS //mousaab std::cout<<"	m_signal->GetSignalPsd () :" << *(m_signal->GetSignalPsd
    ///())<<"\n";

    //////IDS //mousaab std::cout<<"  avant       m_edPower.averagePower :" <<
    /// m_edPower.averagePower<<"\n";

    //////IDS //mousaab std::cout<<"	m_interferenceAndNoise :" <<
    ///*(m_interferenceAndNoise)<<"\n";

    // ajouter le dernier signal

    // IDS //mousaab std::cout<<"   avant           m_InterferenceAvgr :" <<
    // m_InterferenceAvg<<"\n";

    m_edPower.averagePower +=
        LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                 m_phyPIBAttributes.phyCurrentChannel) *
        (Simulator::Now() - m_edPower.lastUpdate).GetTimeStep() /
        m_edPower.measurementLength.GetTimeStep();

    m_InterferenceAvg +=
        LrWpanSpectrumValueHelper::TotalAvgPower(m_interferenceAndNoise,
                                                 m_phyPIBAttributes.phyCurrentChannel) *
        (Simulator::Now() - m_edPower.lastUpdate).GetTimeStep() /
        m_edPower.measurementLength.GetTimeStep();

    // IDS //mousaab std::cout<<"   apres           m_InterferenceAvgr :" <<
    // m_InterferenceAvg<<"\n";

    // IDS double v = 10.0 * log10 (m_InterferenceAvg);
    // IDS //mousaab std::cout<<"              10 log (m_InterferenceAvg) :" << v<<"\n";

    // IDS //mousaab std::cout<<"			    < m_edPower.averagePower = "<<
    // m_edPower.averagePower<<"      >> \n";
    //////IDS //mousaab std::cout<<"			    < m_rxSensitivity = "<<  m_rxSensitivity<<" >>
    ///\n";

    int energyLevel;

    // mousaab : j'ai change la sensitivity = 1 / pour que je puisse calculer l'energie recu par un
    // satellite m_rxSensitivity=1;

    // Per IEEE802.15.4-2006 sec 6.9.7
    double ratio = m_edPower.averagePower / m_rxSensitivity;

    ratio = 10.0 * log10(ratio);
    if (ratio <= 10.0)
    { // less than 10 dB
        energyLevel = 0;
        // energyLevel = static_cast<uint8_t> (((ratio - 10.0) / 30.0) * 255.0);
        //////IDS //mousaab std::cout<<"			    <ratio = "<< ratio<<"      >> \n";
    }
    else if (ratio >= 40.0)
    { // less than 40 dB
        energyLevel = 255;
        // energyLevel = static_cast<uint8_t> (((ratio - 10.0) / 30.0) * 255.0);
        //////IDS //mousaab std::cout<<"			    <ratio = "<< ratio<<"      >> \n";
    }
    else
    {
        // in-between with linear increase per sec 6.9.7
        energyLevel = static_cast<uint8_t>(((ratio - 10.0) / 30.0) * 255.0);
        //////IDS //mousaab std::cout<<"			    <ratio = "<< ratio<<"      >> \n";
    }

    if (!m_plmeEdConfirmCallback.IsNull())
    {
        //////IDS //mousaab std::cout<<"			    <energyLevel = "<< energyLevel<<"      >>
        ///\n";

        m_plmeEdConfirmCallback(IEEE_802_15_4_PHY_SUCCESS, energyLevel);
    }
    else
    {
        //////IDS //mousaab std::cout<<"			m_plmeEdConfirmCallback est null    >> \n";
    }

    if (!m_plmeEdConfirmCallback_V2.IsNull())
    {
        ////IDS //mousaab std::cout<<"			    <energyLevel = "<< energyLevel<<"      >> \n";

        m_plmeEdConfirmCallback_V2(IEEE_802_15_4_PHY_SUCCESS,
                                   energyLevel,
                                   m_edPower.averagePower,
                                   m_InterferenceAvg,
                                   m_PrnChannel);
    }
    else
    {
        //////IDS //mousaab std::cout<<"			m_plmeEdConfirmCallback est null    >> \n";
    }
    // IDS //mousaab std::cout<<"**********************fin
    // LrWpanPhy::EndEd*********************************\n";
}

void
LrWpanPhy::EndCca(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::EndCca*********************************\n";

    NS_LOG_FUNCTION(this);
    LrWpanPhyEnumeration sensedChannelState = IEEE_802_15_4_PHY_UNSPECIFIED;

    // Update peak power.
    double power = LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                            m_phyPIBAttributes.phyCurrentChannel);
    if (m_ccaPeakPower < power)
    {
        m_ccaPeakPower = power;
    }

    if (PhyIsBusy())
    {
        sensedChannelState = IEEE_802_15_4_PHY_BUSY;
    }
    else if (m_phyPIBAttributes.phyCCAMode == 1)
    { // sec 6.9.9 ED detection
        // -- ED threshold at most 10 dB above receiver sensitivity.
        if (10 * log10(m_ccaPeakPower / m_rxSensitivity) >= 10.0)
        {
            sensedChannelState = IEEE_802_15_4_PHY_BUSY;
        }
        else
        {
            sensedChannelState = IEEE_802_15_4_PHY_IDLE;
        }
    }
    else if (m_phyPIBAttributes.phyCCAMode == 2)
    {
        // sec 6.9.9 carrier sense only
        if (m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
        {
            // We currently do not model PPDU reception in detail. Instead we model
            // packet reception starting with the first bit of the preamble.
            // Therefore, this code will never be reached, as PhyIsBusy() would
            // already lead to a channel busy condition.
            // TODO: Change this, if we also model preamble and SFD detection.
            sensedChannelState = IEEE_802_15_4_PHY_BUSY;
        }
        else
        {
            sensedChannelState = IEEE_802_15_4_PHY_IDLE;
        }
    }
    else if (m_phyPIBAttributes.phyCCAMode == 3)
    { // sect 6.9.9 both
        if ((10 * log10(m_ccaPeakPower / m_rxSensitivity) >= 10.0) &&
            m_trxState == IEEE_802_15_4_PHY_BUSY_RX)
        {
            // Again, this code will never be reached, if we are already receiving
            // a packet, as PhyIsBusy() would already lead to a channel busy condition.
            // TODO: Change this, if we also model preamble and SFD detection.
            sensedChannelState = IEEE_802_15_4_PHY_BUSY;
        }
        else
        {
            sensedChannelState = IEEE_802_15_4_PHY_IDLE;
        }
    }
    else
    {
        NS_ASSERT_MSG(false, "Invalid CCA mode");
    }

    NS_LOG_LOGIC(this << "channel sensed state: " << sensedChannelState);

    if (!m_plmeCcaConfirmCallback.IsNull())
    {
        m_plmeCcaConfirmCallback(sensedChannelState);
    }
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::EndCca*********************************\n";
}

void
LrWpanPhy::EndSetTRXState(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::EndSetTRXState*********************************\n";

    NS_LOG_FUNCTION(this);

    NS_ABORT_IF((m_trxStatePending != IEEE_802_15_4_PHY_RX_ON) &&
                (m_trxStatePending != IEEE_802_15_4_PHY_TX_ON));
    ChangeTrxState(m_trxStatePending);
    m_trxStatePending = IEEE_802_15_4_PHY_IDLE;

    if (!m_plmeSetTRXStateConfirmCallback.IsNull())
    {
        m_plmeSetTRXStateConfirmCallback(m_trxState);
    }
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::EndSetTRXState*********************************\n";
}

void
LrWpanPhy::EndTx(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::EndTx*********************************\n";
    //////IDS //mousaab std::cout<<"time :" << Simulator::Now ().GetSeconds ()<<"\n";

    NS_LOG_FUNCTION(this);

    NS_ABORT_IF((m_trxState != IEEE_802_15_4_PHY_BUSY_TX) &&
                (m_trxState != IEEE_802_15_4_PHY_TRX_OFF));

    if (m_currentTxPacket.second == false)
    {
        //////IDS //mousaab std::cout<<"		Packet successfully transmitted\n";

        NS_LOG_DEBUG("Packet successfully transmitted");
        m_phyTxEndTrace(m_currentTxPacket.first);

        if (!m_pdDataConfirmCallback.IsNull())
        {
            m_pdDataConfirmCallback(IEEE_802_15_4_PHY_SUCCESS);
        }
    }
    else
    {
        NS_LOG_DEBUG("Packet transmission aborted");
        m_phyTxDropTrace(m_currentTxPacket.first);
        if (!m_pdDataConfirmCallback.IsNull())
        {
            // See if this is ever entered in another state
            NS_ASSERT(m_trxState == IEEE_802_15_4_PHY_TRX_OFF);
            m_pdDataConfirmCallback(m_trxState);
        }
    }
    m_currentTxPacket.first = 0;
    m_currentTxPacket.second = false;

    // We may be waiting to apply a pending state change.
    if (m_trxStatePending != IEEE_802_15_4_PHY_IDLE)
    {
        // Only change the state immediately, if the transceiver is not already
        // switching the state.
        if (!m_setTRXState.IsRunning())
        {
            NS_LOG_LOGIC("Apply pending state change to " << m_trxStatePending);
            ChangeTrxState(m_trxStatePending);
            m_trxStatePending = IEEE_802_15_4_PHY_IDLE;
            if (!m_plmeSetTRXStateConfirmCallback.IsNull())
            {
                m_plmeSetTRXStateConfirmCallback(IEEE_802_15_4_PHY_SUCCESS);
            }
        }
    }
    else
    {
        if (m_trxState != IEEE_802_15_4_PHY_TRX_OFF)
        {
            ChangeTrxState(IEEE_802_15_4_PHY_TX_ON);
        }
    }
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::EndTx*********************************\n";
}

Time
LrWpanPhy::CalculateTxTime(Ptr<const Packet> packet)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::CalculateTxTime*********************************\n";

    NS_LOG_FUNCTION(this << packet);
    //////IDS //mousaab std::cout<<"========= debut == LrWpanPhy::CalculateTxTime
    ///======================\n";

    bool isData = true;
    Time txTime = GetPpduHeaderTxTime();
    //////IDS //mousaab std::cout<<"	packet size :" << packet->GetSize ()<<"\n";

    //////IDS //mousaab std::cout<<"	txTime1 for header :" << txTime<<"\n";

    txTime += Seconds(packet->GetSize() * 8.0 / GetDataOrSymbolRate(isData));
    //////IDS //mousaab std::cout<<"	txTime2 for data+ header :" << txTime<<"\n";

    //////IDS //mousaab std::cout<<"========= fin == LrWpanPhy::CalculateTxTime
    ///======================\n";

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::CalculateTxTime*********************************\n";

    return txTime;
}

uint8_t
LrWpanPhy::GetCurrentPage() const
{
    return m_phyPIBAttributes.phyCurrentPage;
}

uint8_t
LrWpanPhy::GetCurrentChannelNum() const
{
    return m_phyPIBAttributes.phyCurrentChannel;
}

double
LrWpanPhy::GetDataOrSymbolRate(bool isData)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetDataOrSymbolRate*********************************\n";

    NS_LOG_FUNCTION(this << isData);

    double rate = 0.0;

    NS_ASSERT(m_phyOption < IEEE_802_15_4_INVALID_PHY_OPTION);

    if (isData)
    {
        //////IDS //mousaab std::cout<<"	          its a data  \n";

        //////IDS //mousaab std::cout<<"	          dataSymbolRates[0].bitRate= " <<
        /// dataSymbolRates[0].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[1].bitRate= " <<
        /// dataSymbolRates[1].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[2].bitRate= " <<
        /// dataSymbolRates[2].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[3].bitRate= " <<
        /// dataSymbolRates[3].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[4].bitRate= " <<
        /// dataSymbolRates[4].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[5].bitRate= " <<
        /// dataSymbolRates[5].bitRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[6].bitRate= " <<
        /// dataSymbolRates[6].bitRate<<"\n";

        //////IDS //mousaab std::cout<<"	          m_phyOption= " << m_phyOption<<"\n";

        rate = dataSymbolRates[m_phyOption].bitRate;
        //////IDS //mousaab std::cout<<"	          rate= " << rate<<"\n";
    }
    else
    {
        //////IDS //mousaab std::cout<<"	          its not  a data  \n";

        //////IDS //mousaab std::cout<<"	          dataSymbolRates[0].bitRate= " <<
        /// dataSymbolRates[0].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[1].bitRate= " <<
        /// dataSymbolRates[1].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[2].bitRate= " <<
        /// dataSymbolRates[2].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[3].bitRate= " <<
        /// dataSymbolRates[3].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[4].bitRate= " <<
        /// dataSymbolRates[4].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[5].bitRate= " <<
        /// dataSymbolRates[5].symbolRate<<"\n";
        //////IDS //mousaab std::cout<<"	          dataSymbolRates[6].bitRate= " <<
        /// dataSymbolRates[6].symbolRate<<"\n";

        //////IDS //mousaab std::cout<<"	          m_phyOption= " << m_phyOption<<"\n";

        rate = dataSymbolRates[m_phyOption].symbolRate;
        //////IDS //mousaab std::cout<<"	          rate= " << rate<<"\n";
    }

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetDataOrSymbolRate*********************************\n";

    return (rate * 1000.0);
}

Time
LrWpanPhy::GetPpduHeaderTxTime(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetPpduHeaderTxTime*********************************\n";

    NS_LOG_FUNCTION(this);

    bool isData = false;
    double totalPpduHdrSymbols;

    NS_ASSERT(m_phyOption < IEEE_802_15_4_INVALID_PHY_OPTION);

    totalPpduHdrSymbols = ppduHeaderSymbolNumbers[m_phyOption].shrPreamble +
                          ppduHeaderSymbolNumbers[m_phyOption].shrSfd +
                          ppduHeaderSymbolNumbers[m_phyOption].phr;

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetPpduHeaderTxTime*********************************\n";

    return Seconds(totalPpduHdrSymbols / GetDataOrSymbolRate(isData));
}

// IEEE802.15.4-2006 Table 2 in section 6.1.2
void
LrWpanPhy::SetMyPhyOption(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetMyPhyOption*********************************\n";

    NS_LOG_FUNCTION(this);

    m_phyOption = IEEE_802_15_4_INVALID_PHY_OPTION;

    if (m_phyPIBAttributes.phyCurrentPage == 0)
    {
        if (m_phyPIBAttributes.phyCurrentChannel == 0)
        { // 868 MHz BPSK
            m_phyOption = IEEE_802_15_4_868MHZ_BPSK;
        }
        else if (m_phyPIBAttributes.phyCurrentChannel <= 10)
        { // 915 MHz BPSK
            m_phyOption = IEEE_802_15_4_915MHZ_BPSK;
        }
        else if (m_phyPIBAttributes.phyCurrentChannel <= 26)
        { // 2.4 GHz MHz O-QPSK
            m_phyOption = IEEE_802_15_4_2_4GHZ_OQPSK;
        }
    }
    else if (m_phyPIBAttributes.phyCurrentPage == 1)
    {
        if (m_phyPIBAttributes.phyCurrentChannel == 0)
        { // 868 MHz ASK
            m_phyOption = IEEE_802_15_4_868MHZ_ASK;
        }
        else if (m_phyPIBAttributes.phyCurrentChannel <= 10)
        { // 915 MHz ASK
            m_phyOption = IEEE_802_15_4_915MHZ_ASK;
        }
    }
    else if (m_phyPIBAttributes.phyCurrentPage == 2)
    {
        if (m_phyPIBAttributes.phyCurrentChannel == 0)
        { // 868 MHz O-QPSK
            m_phyOption = IEEE_802_15_4_868MHZ_OQPSK;
        }
        else if (m_phyPIBAttributes.phyCurrentChannel <= 10)
        { // 915 MHz O-QPSK
            m_phyOption = IEEE_802_15_4_915MHZ_OQPSK;
        }
    }
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetMyPhyOption*********************************\n";
}

void
LrWpanPhy::EnableGPS(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::EnableGPS*********************************\n";

    m_GPSstate = 1;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::EnableGPS*********************************\n";

    NS_LOG_FUNCTION(this);
}

void
LrWpanPhy::DisableGPS(void)
{
    m_GPSstate = 0;

    NS_LOG_FUNCTION(this);
}

int
LrWpanPhy::GetGPSstate(void)
{
    return m_GPSstate;
}

LrWpanPhyOption
LrWpanPhy::GetMyPhyOption(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetMyPhyOption*********************************\n";
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetMyPhyOption*********************************\n";

    NS_LOG_FUNCTION(this);
    return m_phyOption;
}

void
LrWpanPhy::SetRxSensitivity(double dbmSensitivity)
{
    NS_LOG_FUNCTION(this << dbmSensitivity << "dBm");

    // See IEEE 802.15.4-2011 Sections 10.3.4, 11.3.4, 13.3.4, 13.3.4, 14.3.4, 15.3.4
    if (m_phyOption == IEEE_802_15_4_915MHZ_BPSK || m_phyOption == IEEE_802_15_4_950MHZ_BPSK)
    {
        if (dbmSensitivity > -92)
        {
            NS_ABORT_MSG("The minimum Rx sensitivity for this band should be at least -92 dBm");
        }
    }
    else
    {
        if (dbmSensitivity > -85)
        {
            NS_ABORT_MSG("The minimum Rx sensitivity for this band should be at least -85 dBm");
        }
    }

    // Calculate the noise factor required to reduce the Rx sensitivity.
    // The maximum possible sensitivity in the current modulation is used as a reference
    // to calculate the noise factor (F). The noise factor is a dimensionless ratio.
    // Currently only one PHY modulation is supported:
    // O-QPSK 250kpps which has a Max Rx sensitivity: -106.58 dBm (Noise factor = 1).
    // After Rx sensitivity is set, this becomes the new point where PER < 1 % for a
    // PSDU of 20 bytes as described by the standard.

    // TODO: recalculate maxRxSensitivity (Noise factor = 1) when additional modulations are
    // supported.
    double maxRxSensitivityW = DbmToW(-106.58);

    LrWpanSpectrumValueHelper psdHelper;
    m_txPsd = psdHelper.CreateTxPowerSpectralDensity(
        GetNominalTxPowerFromPib(m_phyPIBAttributes.phyTransmitPower),
        m_phyPIBAttributes.phyCurrentChannel);
    // Update thermal noise + noise factor added.
    long double noiseFactor = DbmToW(dbmSensitivity) / maxRxSensitivityW;
    psdHelper.SetNoiseFactor(noiseFactor);
    m_noise = psdHelper.CreateNoisePowerSpectralDensity(m_phyPIBAttributes.phyCurrentChannel);

    m_signal = Create<LrWpanInterferenceHelper>(m_noise->GetSpectrumModel());
    // Change receiver sensitivity from dBm to Watts
    m_rxSensitivity = DbmToW(dbmSensitivity);
}

double
LrWpanPhy::GetRxSensitivity()
{
    NS_LOG_FUNCTION(this);
    // Change receiver sensitivity from Watt to dBm
    return WToDbm(m_rxSensitivity);
}

void
LrWpanPhy::SetTxPowerSpectralDensity(Ptr<SpectrumValue> txPsd)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetTxPowerSpectralDensity*********************************\n";

    NS_LOG_FUNCTION(this << txPsd);
    NS_ASSERT(txPsd);
    m_txPsd = txPsd;
    //////IDS //mousaab std::cout<<"	      \t computed tx_psd: " << *txPsd << "\t stored tx_psd:
    ///" << *m_txPsd<< "      >> \n";

    NS_LOG_INFO("\t computed tx_psd: " << *txPsd << "\t stored tx_psd: " << *m_txPsd);
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetTxPowerSpectralDensity*********************************\n";
}

void
LrWpanPhy::SetNoisePowerSpectralDensity(Ptr<const SpectrumValue> noisePsd)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetNoisePowerSpectralDensity*********************************\n";

    NS_LOG_FUNCTION(this << noisePsd);
    NS_LOG_INFO("\t computed noise_psd: " << *noisePsd);
    NS_ASSERT(noisePsd);
    m_noise = noisePsd;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetNoisePowerSpectralDensity*********************************\n";
}

Ptr<const SpectrumValue>
LrWpanPhy::GetNoisePowerSpectralDensity(void)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetNoisePowerSpectralDensity*********************************\n";

    NS_LOG_FUNCTION(this);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetNoisePowerSpectralDensity*********************************\n";

    return m_noise;
}

void
LrWpanPhy::SetErrorModel(Ptr<LrWpanErrorModel> e)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::SetErrorModel*********************************\n";

    NS_LOG_FUNCTION(this << e);
    NS_ASSERT(e);
    m_errorModel = e;
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::SetErrorModel*********************************\n";
}

Ptr<LrWpanErrorModel>
LrWpanPhy::GetErrorModel(void) const
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetErrorModel*********************************\n";

    NS_LOG_FUNCTION(this);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetErrorModel*********************************\n";

    return m_errorModel;
}

uint64_t
LrWpanPhy::GetPhySHRDuration(void) const
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetPhySHRDuration*********************************\n";

    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_phyOption < IEEE_802_15_4_INVALID_PHY_OPTION);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetPhySHRDuration*********************************\n";

    return ppduHeaderSymbolNumbers[m_phyOption].shrPreamble +
           ppduHeaderSymbolNumbers[m_phyOption].shrSfd;
}

double
LrWpanPhy::GetPhySymbolsPerOctet(void) const
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::GetPhySymbolsPerOctet*********************************\n";

    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_phyOption < IEEE_802_15_4_INVALID_PHY_OPTION);

    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::GetPhySymbolsPerOctet*********************************\n";

    return dataSymbolRates[m_phyOption].symbolRate / (dataSymbolRates[m_phyOption].bitRate / 8);
}

double
LrWpanPhy::GetCurrentSignalPsd()
{
    double powerWatts =
        LrWpanSpectrumValueHelper::TotalAvgPower(m_signal->GetSignalPsd(),
                                                 m_phyPIBAttributes.phyCurrentChannel);
    return WToDbm(powerWatts);
}


int8_t
LrWpanPhy::GetNominalTxPowerFromPib(uint8_t phyTransmitPower)
{
    NS_LOG_FUNCTION(this << +phyTransmitPower);

    // The nominal Tx power is stored in the PIB as a 6-bit
    // twos-complement, signed number.

    // The 5 LSBs can be copied - as their representation
    // is the same for unsigned and signed integers.
    int8_t nominalTxPower = phyTransmitPower & 0x1F;

    // Now check the 6th LSB (the "sign" bit).
    // It's a twos-complement format, so the "sign"
    // bit represents -2^5 = -32.
    if (phyTransmitPower & 0x20)
    {
        nominalTxPower -= 32;
    }
    return nominalTxPower;
}

double
LrWpanPhy::WToDbm(double watt)
{
    return (10 * log10(1000 * watt));
}

double
LrWpanPhy::DbmToW(double dbm)
{
    return (pow(10.0, dbm / 10.0) / 1000.0);
}

int64_t
LrWpanPhy::AssignStreams(int64_t stream)
{
    //////IDS //mousaab std::cout<<"**********************debut
    /// LrWpanPhy::AssignStreams*********************************\n";

    NS_LOG_FUNCTION(this);
    m_random->SetStream(stream);
    //////IDS //mousaab std::cout<<"**********************fin
    /// LrWpanPhy::AssignStreams*********************************\n";

    return 1;
}

void
LrWpanPhy::SetPostReceptionErrorModel(const Ptr<ErrorModel> em)
{
    NS_LOG_FUNCTION(this << em);
    m_postReceptionErrorModel = em;
}

} // namespace ns3
