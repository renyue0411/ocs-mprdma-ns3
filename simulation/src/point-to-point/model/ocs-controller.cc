#include "ocs-controller.h"
#include "ocs-node.h"

#include "ns3/simulator.h"
#include "ns3/log.h"

#include <algorithm>
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OcsController");

RrOcsPolicy::RrOcsPolicy(
    const std::vector<uint32_t>& ocsIds,
    const std::vector<uint32_t>& logicalPorts
)
    : m_ocsIds(ocsIds),
      m_logicalPorts(logicalPorts),
      m_slot(0)
{
    std::sort(m_logicalPorts.begin(), m_logicalPorts.end());
    BuildRoundRobinPatterns();

    std::cout << "[OCS POLICY] mode=rr"
              << " ocs_count=" << m_ocsIds.size()
              << " port_count=" << m_logicalPorts.size()
              << " pattern_count=" << m_patterns.size()
              << std::endl;
}

std::string
RrOcsPolicy::GetName() const
{
    return "rr";
}

void
RrOcsPolicy::BuildRoundRobinPatterns()
{
    NS_ASSERT_MSG(!m_ocsIds.empty(), "RR OCS policy requires at least one OCS node");
    NS_ASSERT_MSG(m_logicalPorts.size() >= 2, "RR OCS policy requires at least two logical ports");
    NS_ASSERT_MSG(m_logicalPorts.size() % 2 == 0, "RR OCS policy requires an even number of logical ports");

    std::vector<uint32_t> ports = m_logicalPorts;
    uint32_t n = ports.size();

    // Standard round-robin one-factorization.
    // Each slot is a perfect matching over all logical OCS ports.
    for (uint32_t round = 0; round < n - 1; ++round) {
        std::vector<std::pair<uint32_t, uint32_t> > pairs;

        for (uint32_t i = 0; i < n / 2; ++i) {
            pairs.push_back(std::make_pair(ports[i], ports[n - 1 - i]));
        }

        m_patterns.push_back(pairs);

        // Keep the first element fixed and rotate the remaining elements right.
        uint32_t last = ports[n - 1];
        for (uint32_t i = n - 1; i > 1; --i) {
            ports[i] = ports[i - 1];
        }
        ports[1] = last;
    }
}

OcsLogicalMap
RrOcsPolicy::GetNextMap()
{
    NS_ASSERT_MSG(!m_patterns.empty(), "RR OCS policy has no pattern");

    uint32_t patternId = m_slot % m_patterns.size();
    const std::vector<std::pair<uint32_t, uint32_t> >& pairs = m_patterns[patternId];

    OcsLogicalMap map;

    for (uint32_t i = 0; i < m_ocsIds.size(); ++i) {
        uint32_t ocsId = m_ocsIds[i];

        for (uint32_t j = 0; j < pairs.size(); ++j) {
            map.circuits.push_back(
                OcsCircuit(ocsId, pairs[j].first, pairs[j].second)
            );
        }
    }

    std::cout << "[OCS RR] t=" << Simulator::Now().GetTimeStep()
              << " slot=" << m_slot
              << " pattern=" << patternId
              << std::endl;

    m_slot++;
    return map;
}

OcsController::OcsController(
    NodeContainer nodes,
    const std::map<uint32_t, std::map<uint32_t, uint32_t> >& logicalPortToIf,
    OcsPolicy* policy,
    Time controlInterval,
    Time reconfigDelay
)
    : m_nodes(nodes),
      m_logicalPortToIf(logicalPortToIf),
      m_policy(policy),
      m_controlInterval(controlInterval),
      m_reconfigDelay(reconfigDelay),
      m_busyUntil(Seconds(0)),
      m_tickCount(0)
{
    NS_ASSERT_MSG(m_policy != 0, "OcsController requires a non-null policy");
    NS_ASSERT_MSG(m_controlInterval > Seconds(0), "OCS control interval must be positive");

    std::cout << "[OCS CTRL INIT] mode=" << m_policy->GetName()
              << " interval=" << m_controlInterval.GetTimeStep()
              << " reconfig_delay=" << m_reconfigDelay.GetTimeStep()
              << std::endl;
}

void
OcsController::Start(Time startTime)
{
    std::cout << "[OCS CTRL START] start=" << startTime.GetTimeStep()
              << std::endl;

    Simulator::Schedule(startTime, &OcsController::Tick, this);
}

void
OcsController::ScheduleNextTick()
{
    Simulator::Schedule(m_controlInterval, &OcsController::Tick, this);
}

void
OcsController::Tick()
{
    std::cout << "[OCS CTRL TICK] t=" << Simulator::Now().GetTimeStep()
              << " tick=" << m_tickCount
              << std::endl;

    if (Simulator::Now() < m_busyUntil) {
        std::cout << "[OCS CTRL SKIP] t=" << Simulator::Now().GetTimeStep()
                  << " busy_until=" << m_busyUntil.GetTimeStep()
                  << std::endl;
        m_tickCount++;
        ScheduleNextTick();
        return;
    }

    OcsLogicalMap nextMap = m_policy->GetNextMap();

    if (IsValidLogicalMap(nextMap)) {
        ApplyMap(nextMap);
        m_busyUntil = Simulator::Now() + m_reconfigDelay;
    } else {
        std::cout << "[OCS CTRL INVALID MAP] t=" << Simulator::Now().GetTimeStep()
                  << std::endl;
    }

    m_tickCount++;
    ScheduleNextTick();
}

bool
OcsController::IsValidLogicalMap(const OcsLogicalMap& logicalMap) const
{
    std::map<uint32_t, std::set<uint32_t> > usedPorts;

    for (uint32_t i = 0; i < logicalMap.circuits.size(); ++i) {
        const OcsCircuit& c = logicalMap.circuits[i];

        if (c.portA == c.portB) {
            std::cout << "[OCS CTRL INVALID] self circuit: OCS " << c.ocsId
                      << " port " << c.portA << std::endl;
            return false;
        }

        if (m_logicalPortToIf.find(c.ocsId) == m_logicalPortToIf.end()) {
            std::cout << "[OCS CTRL INVALID] unknown OCS " << c.ocsId << std::endl;
            return false;
        }

        if (m_logicalPortToIf.find(c.ocsId)->second.find(c.portA) ==
            m_logicalPortToIf.find(c.ocsId)->second.end()) {
            std::cout << "[OCS CTRL INVALID] OCS " << c.ocsId
                      << " logical port " << c.portA << " not found" << std::endl;
            return false;
        }

        if (m_logicalPortToIf.find(c.ocsId)->second.find(c.portB) ==
            m_logicalPortToIf.find(c.ocsId)->second.end()) {
            std::cout << "[OCS CTRL INVALID] OCS " << c.ocsId
                      << " logical port " << c.portB << " not found" << std::endl;
            return false;
        }

        if (usedPorts[c.ocsId].find(c.portA) != usedPorts[c.ocsId].end() ||
            usedPorts[c.ocsId].find(c.portB) != usedPorts[c.ocsId].end()) {
            std::cout << "[OCS CTRL INVALID] duplicated logical port on OCS "
                      << c.ocsId << std::endl;
            return false;
        }

        usedPorts[c.ocsId].insert(c.portA);
        usedPorts[c.ocsId].insert(c.portB);
    }

    return true;
}

void
OcsController::ApplyMap(const OcsLogicalMap& logicalMap)
{
    std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > > ifMaps;

    for (uint32_t i = 0; i < logicalMap.circuits.size(); ++i) {
        const OcsCircuit& c = logicalMap.circuits[i];

        uint32_t ifA = m_logicalPortToIf[c.ocsId].find(c.portA)->second;
        uint32_t ifB = m_logicalPortToIf[c.ocsId].find(c.portB)->second;

        // One logical circuit is bidirectional.
        ifMaps[c.ocsId].push_back(std::make_pair(ifA, ifB));
        ifMaps[c.ocsId].push_back(std::make_pair(ifB, ifA));

        std::cout << "[OCS CTRL MAP] t=" << Simulator::Now().GetTimeStep()
                  << " OCS " << c.ocsId
                  << " logical " << c.portA << "(if " << ifA << ")"
                  << " <-> "
                  << "logical " << c.portB << "(if " << ifB << ")"
                  << std::endl;
    }

    for (std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > >::iterator it = ifMaps.begin();
         it != ifMaps.end(); ++it) {
        uint32_t ocsId = it->first;

        NS_ASSERT_MSG(ocsId < m_nodes.GetN(), "OCS controller points to invalid node id");

        Ptr<OcsNode> ocs = DynamicCast<OcsNode>(m_nodes.Get(ocsId));
        NS_ASSERT_MSG(ocs != 0, "OCS controller points to a non-OCS node");

        ocs->RequestReconfiguration(it->second, m_reconfigDelay);
    }
}

} // namespace ns3
