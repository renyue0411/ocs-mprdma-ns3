#ifndef OCS_NODE_H
#define OCS_NODE_H

#include <unordered_map>
#include <vector>
#include <utility>

#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "ns3/custom-header.h"
#include "mp-qbb-net-device.h"

namespace ns3 {

class OcsNode : public Node {
public:
    static TypeId GetTypeId(void);

    OcsNode();

    // 直接设置当前生效的 mapping，不产生 downtime。
    void SetInitialMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping);

    // 后面用于动态切换。第一版可以先不用。
    void RequestReconfiguration(
        const std::vector<std::pair<uint32_t, uint32_t> >& newMapping,
        Time downtime
    );

    bool IsReconfiguring() const;

    // OCS data plane: packet 从某个端口进来后，按 mapping 找到输出端口。
    virtual bool SwitchReceiveFromDevice(Ptr<NetDevice> device, Ptr<Packet> packet, CustomHeader &ch);

    // OCS 第一版不做 MMU/PFC/ECN，先空实现。
    virtual void SwitchNotifyDequeue(uint32_t ifIndex, uint32_t qIndex, Ptr<Packet> p);

private:
    void FinishReconfiguration();
    void InstallMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping);
    uint32_t GetQueueIndex(CustomHeader &ch) const;

private:
    bool m_reconfiguring;

    // active map: inPort -> outPort
    std::unordered_map<uint32_t, uint32_t> m_activeMap;

    // pending map: reconfiguration 完成后启用
    std::unordered_map<uint32_t, uint32_t> m_pendingMap;
};

} // namespace ns3

#endif