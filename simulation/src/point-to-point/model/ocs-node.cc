#include "ocs-node.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OcsNode");
NS_OBJECT_ENSURE_REGISTERED(OcsNode);

TypeId
OcsNode::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::OcsNode")
        .SetParent<Node>()
        .AddConstructor<OcsNode>();
    return tid;
}

OcsNode::OcsNode()
{
    // 0 = host, 1 = normal switch, 2 = OCS
    m_node_type = 2;
    m_reconfiguring = false;
}

void
OcsNode::InstallMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping)
{
    m_activeMap.clear();

    for (size_t i = 0; i < mapping.size(); ++i) {
        uint32_t a = mapping[i].first;
        uint32_t b = mapping[i].second;

        // 默认双向 circuit
        m_activeMap[a] = b;
        m_activeMap[b] = a;

        std::cout << "[OCS MAP] t=" << Simulator::Now().GetTimeStep()
                  << " node=" << GetId()
                  << " " << a << "<->" << b << std::endl;
    }
}

void
OcsNode::SetInitialMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping)
{
    m_reconfiguring = false;
    InstallMapping(mapping);
}

void
OcsNode::RequestReconfiguration(
    const std::vector<std::pair<uint32_t, uint32_t> >& newMapping,
    Time downtime
)
{
    m_reconfiguring = true;
    m_pendingMap.clear();

    for (size_t i = 0; i < newMapping.size(); ++i) {
        uint32_t a = newMapping[i].first;
        uint32_t b = newMapping[i].second;

        m_pendingMap[a] = b;
        m_pendingMap[b] = a;
    }

    std::cout << "[OCS RECONFIG START] t=" << Simulator::Now().GetTimeStep()
              << " node=" << GetId()
              << " downtime=" << downtime.GetTimeStep()
              << std::endl;

    Simulator::Schedule(downtime, &OcsNode::FinishReconfiguration, this);
}

void
OcsNode::FinishReconfiguration()
{
    m_activeMap = m_pendingMap;
    m_pendingMap.clear();
    m_reconfiguring = false;

    std::cout << "[OCS RECONFIG DONE] t=" << Simulator::Now().GetTimeStep()
              << " node=" << GetId()
              << std::endl;
}

bool
OcsNode::IsReconfiguring() const
{
    return m_reconfiguring;
}

uint32_t
OcsNode::GetQueueIndex(CustomHeader &ch) const
{
    // 和 MpSwitchNode 的基本逻辑保持接近：
    // PFC / CNP / ACK / NACK 走高优先级 queue 0。
    if (ch.l3Prot == 0xFF || ch.l3Prot == 0xFE ||
        ch.l3Prot == 0xFD || ch.l3Prot == 0xFC) {
        return 0;
    }

    // TCP 放 queue 1；RDMA/UDP 按 PG 放队列。
    if (ch.l3Prot == 0x06) {
        return 1;
    }

    return ch.udp.pg;
}

bool
OcsNode::SwitchReceiveFromDevice(Ptr<NetDevice> device, Ptr<Packet> packet, CustomHeader &ch)
{
    uint32_t inPort = device->GetIfIndex();

    if (m_reconfiguring) {
        std::cout << "[OCS DROP RECONFIG] t=" << Simulator::Now().GetTimeStep()
                  << " node=" << GetId()
                  << " inPort=" << inPort
                  << std::endl;
        return true;
    }

    std::unordered_map<uint32_t, uint32_t>::iterator it = m_activeMap.find(inPort);
    if (it == m_activeMap.end()) {
        std::cout << "[OCS DROP NO_MAP] t=" << Simulator::Now().GetTimeStep()
                  << " node=" << GetId()
                  << " inPort=" << inPort
                  << std::endl;
        return true;
    }

    uint32_t outPort = it->second;

    if (outPort >= GetNDevices()) {
        std::cout << "[OCS DROP BAD_PORT] t=" << Simulator::Now().GetTimeStep()
                  << " node=" << GetId()
                  << " inPort=" << inPort
                  << " outPort=" << outPort
                  << std::endl;
        return true;
    }

    Ptr<MpQbbNetDevice> outDev = DynamicCast<MpQbbNetDevice>(GetDevice(outPort));
    if (outDev == 0) {
        std::cout << "[OCS DROP BAD_DEV] t=" << Simulator::Now().GetTimeStep()
                  << " node=" << GetId()
                  << " outPort=" << outPort
                  << std::endl;
        return true;
    }

    uint32_t qIndex = GetQueueIndex(ch);

    outDev->SwitchSend(qIndex, packet, ch);
    return true;
}

void
OcsNode::SwitchNotifyDequeue(uint32_t ifIndex, uint32_t qIndex, Ptr<Packet> p)
{
    // 第一版不做 OCS 内部 MMU / PFC / ECN 统计。
    // 后面如果要模拟 OCS 入口缓存或 ToR admission，可以在这里扩展。
}

} // namespace ns3