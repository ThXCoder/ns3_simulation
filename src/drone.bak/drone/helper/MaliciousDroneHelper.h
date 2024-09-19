/*
 * MaliciousDroneHelper.h
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#ifndef SRC_DRONE_HELPER_MALICIOUSDRONEHELPER_H_
#define SRC_DRONE_HELPER_MALICIOUSDRONEHELPER_H_

#include "ns3/object-factory.h"

#include "ns3/Malicious.h"
#include "ns3/application-container.h"
#include "ns3/attribute.h"
#include "ns3/node-container.h"
#include <ns3/lr-wpan-module.h>

namespace ns3
{

  class MaliciousDroneHelper
  {
  public:
    MaliciousDroneHelper();

    void SetAttribute(std::string name, const AttributeValue &value);

    ApplicationContainer Install(NodeContainer c) const;

    ApplicationContainer Install(Ptr<Node> node) const;

    ApplicationContainer Install(std::string nodeName) const;

    std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesTable;

  private:
    Ptr<Application> InstallPriv(Ptr<Node> node) const;

    ObjectFactory m_factory;
  };
}

#endif /* SRC_DRONE_HELPER_MALICIOUSDRONEHELPER_H_ */
