/*
 * MaliciousDroneHelper.cpp
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#include "MaliciousDroneHelper.h"
#include "ns3/node.h"
#include "ns3/names.h"
#include "ns3/log.h"

#include "iostream"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("MaliciousDroneHelper");

  MaliciousDroneHelper::MaliciousDroneHelper()
  {
    // std::cout<<"debut MaliciousDroneHelper==============================================\n";

    m_factory.SetTypeId("ns3::Malicious");

    //	  std::cout<<"fin MaliciousDroneHelper==============================================\n";
  }

  void
  MaliciousDroneHelper::SetAttribute(std::string name, const AttributeValue &value)
  {
    // std::cout<<"debut SetAttribute==============================================\n";

    m_factory.Set(name, value);
    // std::cout<<"fin SetAttribute==============================================\n";
  }

  ApplicationContainer
  MaliciousDroneHelper::Install(NodeContainer c) const
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
  MaliciousDroneHelper::Install(Ptr<Node> node) const
  {
    // std::cout<<"debut + fin Install with ptr<node>==============================================\n";

    return ApplicationContainer(InstallPriv(node));
  }

  ApplicationContainer
  MaliciousDroneHelper::Install(std::string nodeName) const
  {
    //  std::cout<<"debut Instal with string node name==============================================\n";

    Ptr<Node> node = Names::Find<Node>(nodeName);

    // std::cout<<"fin Instal with string node name==============================================\n";

    return ApplicationContainer(InstallPriv(node));
  }

  Ptr<Application>
  MaliciousDroneHelper::InstallPriv(Ptr<Node> node) const
  {
    //  std::cout<<"debut InstallPriv==============================================\n";

    NS_LOG_INFO("installing node");
    Ptr<Application> app = m_factory.Create<Application>();
    node->AddApplication(app);
    // std::cout<<"fin InstallPriv==============================================\n";

    return app;
  }
}
