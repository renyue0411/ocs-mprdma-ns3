#ifndef OCS_CONTROLLER_H
#define OCS_CONTROLLER_H

#include <stdint.h>
#include <map>
#include <set>
#include <vector>
#include <utility>
#include <string>

#include "ns3/node-container.h"
#include "ns3/nstime.h"

namespace ns3 {

struct OcsCircuit
{
    uint32_t ocsId;
    uint32_t portA;
    uint32_t portB;

    OcsCircuit() : ocsId(0), portA(0), portB(0) {}
    OcsCircuit(uint32_t id, uint32_t a, uint32_t b)
        : ocsId(id), portA(a), portB(b) {}
};

struct OcsLogicalMap
{
    std::vector<OcsCircuit> circuits;
};

class OcsPolicy
{
public:
    virtual ~OcsPolicy() {}
    virtual OcsLogicalMap GetNextMap() = 0;
    virtual std::string GetName() const = 0;
};

class RrOcsPolicy : public OcsPolicy
{
public:
    RrOcsPolicy(
        const std::vector<uint32_t>& ocsIds,
        const std::vector<uint32_t>& logicalPorts
    );

    virtual OcsLogicalMap GetNextMap();
    virtual std::string GetName() const;

private:
    void BuildRoundRobinPatterns();

private:
    std::vector<uint32_t> m_ocsIds;
    std::vector<uint32_t> m_logicalPorts;
    std::vector<std::vector<std::pair<uint32_t, uint32_t> > > m_patterns;
    uint32_t m_slot;
};

class OcsController
{
public:
    OcsController(
        NodeContainer nodes,
        const std::map<uint32_t, std::map<uint32_t, uint32_t> >& logicalPortToIf,
        OcsPolicy* policy,
        Time controlInterval,
        Time reconfigDelay
    );

    void Start(Time startTime);
    void Tick();

private:
    void ScheduleNextTick();
    void ApplyMap(const OcsLogicalMap& logicalMap);
    bool IsValidLogicalMap(const OcsLogicalMap& logicalMap) const;

private:
    NodeContainer m_nodes;
    std::map<uint32_t, std::map<uint32_t, uint32_t> > m_logicalPortToIf;

    OcsPolicy* m_policy;
    Time m_controlInterval;
    Time m_reconfigDelay;
    Time m_busyUntil;
    uint32_t m_tickCount;
};

} // namespace ns3

#endif // OCS_CONTROLLER_H
