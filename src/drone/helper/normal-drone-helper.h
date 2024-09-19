/*
 * normal-drone-helper.h
 *
 *  Created on: May 24, 2018
 *      Author: bada
 */

#ifndef SRC_DRONE_HELPER_NORMAL_DRONE_HELPER_H_
#define SRC_DRONE_HELPER_NORMAL_DRONE_HELPER_H_

#include "ns3/object-factory.h"

#include "ns3/normal-drone-app.h"
#include "ns3/application-container.h"
#include "ns3/attribute.h"
#include "ns3/node-container.h"
#include <ns3/lr-wpan-module.h>

namespace ns3
{

  class NormalDroneHelper
  {
  public:
    NormalDroneHelper();

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

#endif /* SRC_DRONE_HELPER_NORMAL_DRONE_HELPER_H_ */
