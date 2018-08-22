/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/ns2-mobility-helper.h"

#include "ns3/animation-interface.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("11pInterference");

/*
 * In WAVE module, there is no net device class named like "Wifi80211pNetDevice",
 * instead, we need to use Wifi80211pHelper to create an object of
 * WifiNetDevice class.
 *
 * usage:
 *  NodeContainer nodes;
 *  NetDeviceContainer devices;
 *  nodes.Create (2);
 *  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
 *  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
 *  wifiPhy.SetChannel (wifiChannel.Create ());
 *  NqosWaveMacHelper wifi80211pMac = NqosWave80211pMacHelper::Default();
 *  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
 *  devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
 *
 * The reason of not providing a 802.11p class is that most of modeling
 * 802.11p standard has been done in wifi module, so we only need a high
 * MAC class that enables OCB mode.
 */

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      char buf[256];
      sprintf(buf,"Received one packet at %u (%u)!",
        socket->GetNode()->GetId(),socket->GetNode()->GetSystemId());
      //NS_LOG_UNCOND (buf);
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval ) {
  if (pktCount > 0) {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    } else  socket->Close ();
}

class WifiPhyStats : public Object
{
private:
  uint32_t m_phyPkts[3]; ///< phy transmit packets
  uint32_t m_phyBytes[3]; ///< phy transmit bytes
  AnimationInterface anim;;
public:
  static TypeId GetTypeId (void);
  WifiPhyStats() : anim("80211p-interference.xml") { Reset(); }
  virtual ~WifiPhyStats () {}
  uint32_t GetBytes (unsigned i);
  uint32_t GetPkts (unsigned i);
  void Reset();
  void TxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower);
  void RxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, WifiPreamble preamble);
  void RxErrorTrace (std::string context, Ptr<const Packet> packet, double snr);
  void Tx (std::string context, Ptr<const Packet> packet);
  void Rx (std::string context, Ptr<const Packet> packet);
  void TxDrop (std::string context, Ptr<const Packet> packet);
  void RxDrop (std::string context, Ptr<const Packet> packet);
};

NS_OBJECT_ENSURE_REGISTERED (WifiPhyStats);

TypeId WifiPhyStats::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiPhyStats")
    .SetParent<Object> ()
    .AddConstructor<WifiPhyStats> ();
  return tid;
}

void WifiPhyStats::TxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower) {
  uint32_t pktSize = packet->GetSize ();
  Ptr<Node> d=anim.GetNodeFromContext(context);
  //Ipv4Header h;
  WifiMacHeader h;
  packet->PeekHeader(h);
  unsigned a[4];
  uint8_t sm[6];
  Mac48Address mac;
  mac=h.GetAddr1(); mac.CopyTo(sm); a[0]=(unsigned)sm[5]-1;
  mac=h.GetAddr2(); mac.CopyTo(sm); a[1]=(unsigned)sm[5]-1;
  mac=h.GetAddr3(); mac.CopyTo(sm); a[2]=(unsigned)sm[5]-1;
  mac=h.GetAddr4(); mac.CopyTo(sm); a[3]=(unsigned)sm[5]-1;
  //Ipv4Address src=h.GetSource();
  //if (d->GetId()!=0) return;
  NS_LOG_FUNCTION (this << context << "PHYRXOk " << d->GetId() << "<-"
                        << a[0] << "," << a[1] << "," << a[2] << "," << a[3] << ","
                        << " mode=" << mode << " size=" << pktSize << " txPower=" << (unsigned)txPower);
}

void WifiPhyStats::RxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, WifiPreamble preamble)
{
  uint32_t pktSize = packet->GetSize ();
  Ptr<Node> d=anim.GetNodeFromContext(context);
  //Ipv4Header h;
  WifiMacHeader h;
  packet->PeekHeader(h);
  unsigned a[4];
  uint8_t sm[6];
  Mac48Address mac;
  mac=h.GetAddr1(); mac.CopyTo(sm); a[0]=(unsigned)sm[5]-1;
  mac=h.GetAddr2(); mac.CopyTo(sm); a[1]=(unsigned)sm[5]-1;
  mac=h.GetAddr3(); mac.CopyTo(sm); a[2]=(unsigned)sm[5]-1;
  mac=h.GetAddr4(); mac.CopyTo(sm); a[3]=(unsigned)sm[5]-1;
  //Ipv4Address src=h.GetSource();
  //if (d->GetId()!=2) return;
  if (a[1]==1) return;
  NS_LOG_UNCOND (this << context << "PHYRXOk " << d->GetId() << "<-"
                        << a[0] << "," << a[1] << "," << a[2] << "," << a[3] << ","
                        << " mode=" << mode << " size=" << pktSize << " snr=" << snr);
  unsigned bin=(pktSize==14     ? 0 :
               (pktSize==128+64 ? 1  : 2));
  ++m_phyPkts[bin];
  m_phyBytes[bin] += pktSize;
}
void WifiPhyStats::RxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
  //uint32_t pktSize = packet->GetSize ();
  NS_LOG_UNCOND (this << context << "PHYRXError mode=" << " snr=" << snr);
}

void WifiPhyStats::Tx (std::string context, Ptr<const Packet> packet) { NS_LOG_UNCOND (this << context); }
void WifiPhyStats::Rx (std::string context, Ptr<const Packet> packet) { NS_LOG_UNCOND (this << context); }
void WifiPhyStats::TxDrop (std::string context, Ptr<const Packet> packet) { NS_LOG_UNCOND (this << context); }
void WifiPhyStats::RxDrop (std::string context, Ptr<const Packet> packet) { NS_LOG_UNCOND (this << context); }


uint32_t WifiPhyStats::GetBytes (unsigned i) { return m_phyBytes[i]; }
uint32_t WifiPhyStats::GetPkts (unsigned i) { return m_phyPkts[i]; }
void WifiPhyStats::Reset (void) {
  m_phyPkts[0]=0; m_phyBytes[0]=0;
  m_phyPkts[1]=0; m_phyBytes[1]=0;
  m_phyPkts[2]=0; m_phyBytes[2]=0;
}
static void ResetStats (Ptr<WifiPhyStats> s)
{
    s->Reset();
}

Ptr<Node> n0;
static void MonitorTraffic (Ptr<WifiPhyStats> s)
{

  Ptr<MobilityModel> m = n0->GetObject<MobilityModel> ();
  cout << "Node0" << "@" << m->GetPosition();

    for (unsigned i=0; i<3; i++)
    cout << s->GetPkts(i) << "," << s->GetBytes(i) << ", ";
    cout << endl;
    Simulator::Schedule (Seconds(1), &MonitorTraffic,s);
}

static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)  __attribute__ ((unused));
static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}


int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 128; // bytes
  uint32_t numPackets = 1000;
  double interval = .1; // 100 milliseconds
  bool verbose = true;
  ns3::PacketMetadata::Enable ();
  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn o n all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  NodeContainer c0,c1;
  c0.Create (2);
  c1.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy0 =  YansWifiPhyHelper::Default ();
  YansWifiPhyHelper wifiPhy1 =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();


  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  //wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel", "Frequency", DoubleValue (5.925e9), "HeightAboveZ", DoubleValue (1.5));
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

  Ptr<YansWifiChannel> channel = wifiChannel.Create ();

  wifiPhy0.SetChannel (channel);
  wifiPhy0.Set ("TxPowerStart",DoubleValue (20));
  wifiPhy0.Set ("TxPowerEnd", DoubleValue (20));

  wifiPhy1.SetChannel (channel);
  wifiPhy1.Set ("TxPowerStart",DoubleValue (-20));
  wifiPhy1.Set ("TxPowerEnd", DoubleValue (-20));

  // ns-3 supports generate a pcap trace
  wifiPhy0.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  wifiPhy1.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      //LogComponentEnable ("PropagationLossModel", LOG_LEVEL_ALL);
      //LogComponentEnable ("WifiPhy", LOG_LEVEL_ALL);
      //LogComponentEnable ("MacLow", LOG_LEVEL_ALL);
      //LogComponentEnable ("YansWifiPhy", LOG_LEVEL_ALL);
      //LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
      //LogComponentEnable ("RegularWifiMac", LOG_LEVEL_ALL);
      //LogComponentEnable ("OcbWifiMac", LOG_LEVEL_ALL);
      //LogComponentEnable ("DcfManager", LOG_LEVEL_ALL);
      //wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }


  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices0 = wifi80211p.Install (wifiPhy0, wifi80211pMac, c0);
  NetDeviceContainer devices1 = wifi80211p.Install (wifiPhy1, wifi80211pMac, c1);

  // Tracing
  wifiPhy0.EnablePcap ("11pSafety", devices0);
  wifiPhy1.EnablePcap ("11pInterference", devices1);

#if 1
  MobilityHelper mobility;
  #if 0
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                           "Mode", StringValue ("Time"),
                           "Time", StringValue ("2s"),
                           "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                           "Bounds", StringValue ("0|200|0|200"));
  #elif 0
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                           "Mode", StringValue ("Time"),
                           "Time", StringValue ("2s"),
                           "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                           "Bounds", RectangleValue (Rectangle (0.0, 20.0, 0.0, 20.0)));
  #else
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  #endif
#if 0
mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                               "X", StringValue ("7.0"),
                               "Y", StringValue ("7.0"),
                               "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=7]"));
#elif 0
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                "MinX", DoubleValue (0.0),
                                "MinY", DoubleValue (0.0),
                                "DeltaX", DoubleValue (5.0),
                                "DeltaY", DoubleValue (5.0),
                                "GridWidth", UintegerValue (2),
                                "LayoutType", StringValue ("RowFirst"));

#else
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (100.0, 50.0, 0.0));
  positionAlloc->Add (Vector (125.0, 50.0, 0.0));
  positionAlloc->Add (Vector (100.0, 52.0, 0.0));
  positionAlloc->Add (Vector (100.0, 55.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
#endif
  mobility.Install (c0);
  mobility.Install (c1);
#else
  Ns2MobilityHelper ns2 = Ns2MobilityHelper ("src/wave/examples/mob8lane,600,120sec.tcl");
  ns2.Install();
#endif
  InternetStackHelper internet;
  internet.Install (c0);
  internet.Install (c1);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ic0 = ipv4.Assign (devices0);
  Ipv4InterfaceContainer ic1 = ipv4.Assign (devices1);

  if (true) {
    for (int i=0; i<2; i++) {
      Ptr<Node> n=c0.Get(i);
      Ptr<MobilityModel> m = n->GetObject<MobilityModel> ();
      cout << "Device" << i << ": " << ic0.GetAddress(i)
           << "(" << devices0.Get(i)->GetAddress() << ")"
           << "@" << m->GetPosition()
           << endl;
    }
    for (int i=0; i<2; i++) {
      Ptr<Node> n=c1.Get(i);
      Ptr<MobilityModel> m = n->GetObject<MobilityModel> ();
      cout << "Device" << i << ": " << ic1.GetAddress(i)
           << "(" << devices0.Get(i)->GetAddress() << ")"
           << "@" << m->GetPosition()
           << endl;
    }
  }
  n0=c0.Get(0);

  Ptr<WifiPhyStats> m_wifiPhyStats; ///< wifi phy statistics
  m_wifiPhyStats = CreateObject<WifiPhyStats> ();

  //std::ofstream os;
  //os.open (logFile.c_str ());
  //Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeBoundCallback (&CourseChange, &cerr));

  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&WifiPhyStats::TxTrace, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback (&WifiPhyStats::RxOkTrace, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback (&WifiPhyStats::RxErrorTrace, m_wifiPhyStats));

  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback (&WifiPhyStats::Tx, m_wifiPhyStats));
  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback (&WifiPhyStats::Rx, m_wifiPhyStats));

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback (&WifiPhyStats::TxDrop, m_wifiPhyStats));
  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback (&WifiPhyStats::RxDrop, m_wifiPhyStats));

  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&WifiPhyStats::Tx, m_wifiPhyStats));
  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&WifiPhyStats::Rx, m_wifiPhyStats));

  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback (&WifiPhyStats::TxDrop, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback (&WifiPhyStats::RxDrop, m_wifiPhyStats));


  TypeId utid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink0 = Socket::CreateSocket (c0.Get (0), utid);
  { InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink0->Bind (local);
    recvSink0->SetRecvCallback (MakeCallback (&ReceivePacket));
  }

  Ptr<Socket> src1 = Socket::CreateSocket (c0.Get (1), utid);
  { InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);//10.1.1.1
    src1->SetAllowBroadcast (true);
    src1->Connect (remote);
  }

  TypeId ttid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  Ptr<Socket> src2 = Socket::CreateSocket (c1.Get (0), utid);
  { InetSocketAddress remote = InetSocketAddress (Ipv4Address ("10.1.1.4"), 80);
    src2->Connect (remote);
  }

  Ptr<Socket> recvSink1 = Socket::CreateSocket (c1.Get (1), utid);
  { InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink1->Bind (local);
    //recvSink1->Listen();
    recvSink1->SetRecvCallback (MakeCallback (&ReceivePacket));
  }

/*
# Scheduling
*/
  if (true) Simulator::ScheduleWithContext (src1->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  src1, packetSize, numPackets, interPacketInterval);

  if (true) Simulator::ScheduleWithContext (src2->GetNode ()->GetId (),
                                  Seconds (10.0), &GenerateTraffic,
                                  src2, 1432, 10000000, MicroSeconds(32*4300));

  if (true) {
      Simulator::Schedule (Seconds (10.0), &ResetStats, m_wifiPhyStats);
      Simulator::Schedule (Seconds (0.0), &MonitorTraffic, m_wifiPhyStats);
  }
  //Simulator::Schedule (Seconds (20.0), &Simulator::Stop);
  Simulator::Stop(Seconds (20.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
