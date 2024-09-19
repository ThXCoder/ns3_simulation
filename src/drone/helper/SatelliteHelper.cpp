/*
 * normal-drone-helper.cpp
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#include "SatelliteHelper.h"
#include "ns3/node.h"
#include "ns3/names.h"
#include "ns3/log.h"

#include "iostream"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("SatelliteHelper");

  SatelliteHelper::SatelliteHelper()
  {
    // std::cout<<"debut SatelliteHelper==============================================\n";

    m_factory.SetTypeId("ns3::SatelliteApp");

    //	  std::cout<<"fin SatelliteHelper==============================================\n";
  }

  void
  SatelliteHelper::SetAttribute(std::string name, const AttributeValue &value)
  {
    // std::cout<<"debut SetAttribute==============================================\n";

    m_factory.Set(name, value);
    // std::cout<<"fin SetAttribute==============================================\n";
  }

  ApplicationContainer
  SatelliteHelper::Install(NodeContainer c) const
  {
    // std::cout<<"debut Install with nodecontainet==============================================\n";

    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
      apps.Add(InstallPriv(*i));
    }

    // std::cout<<"fin Install with nodecontainer==============================================\n";

    return apps;
  }

  ApplicationContainer
  SatelliteHelper::Install(Ptr<Node> node) const
  {
    // std::cout<<"debut + fin Install with ptr<node>==============================================\n";

    return ApplicationContainer(InstallPriv(node));
  }

  ApplicationContainer
  SatelliteHelper::Install(std::string nodeName) const
  {
    //  std::cout<<"debut Instal with string node name==============================================\n";

    Ptr<Node> node = Names::Find<Node>(nodeName);

    // std::cout<<"fin Instal with string node name==============================================\n";

    return ApplicationContainer(InstallPriv(node));
  }

  Ptr<Application>
  SatelliteHelper::InstallPriv(Ptr<Node> node) const
  {
    //  std::cout<<"debut InstallPriv==============================================\n";

    NS_LOG_INFO("installing node");
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);
    // std::cout<<"fin InstallPriv==============================================\n";

    return app;
  }
}
