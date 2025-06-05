/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "uav-base-scheduler_nr.h"
#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UavBaseScheduler");

NS_OBJECT_ENSURE_REGISTERED(UavBaseScheduler);

TypeId
UavBaseScheduler::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::UavBaseScheduler")
            .SetParent<NrSlUeMacSchedulerFixedMcs>()
            .AddConstructor<UavBaseScheduler>()
            .SetGroupName("nr");
    return tid;
}

UavBaseScheduler::UavBaseScheduler()
{
    NS_LOG_FUNCTION(this);
}

UavBaseScheduler::~UavBaseScheduler()
{
    NS_LOG_FUNCTION(this);
}




uint32_t UavBaseScheduler::CalculateTbSize(Ptr<const NrAmc> nrAmc,
                            uint8_t dstMcs,
                            uint16_t symbolsPerSlot,
                            uint16_t availableSubChannels,
                            uint16_t subChannelSize) const
{

    NS_LOG_FUNCTION(this << nrAmc << dstMcs << symbolsPerSlot << availableSubChannels
                         << subChannelSize);
    NS_ASSERT_MSG(availableSubChannels > 0, "Must have at least one available subchannel");
    NS_ASSERT_MSG(subChannelSize > 0, "Must have non-zero subChannelSize");
    NS_ASSERT_MSG(symbolsPerSlot <= 14, "Invalid number of symbols per slot");
    uint8_t slRank{1}; // XXX get the sidelink rank from somewhere
    return nrAmc->CalculateTbSize(dstMcs,
                                  slRank,
                                  subChannelSize * availableSubChannels * symbolsPerSlot);
}


void
UavBaseScheduler::DoSchedNrSlRlcBufferReq(
    const struct NrSlMacSapProvider::NrSlReportBufferStatusParameters& params)
{
    NS_LOG_FUNCTION(this << params.dstL2Id << +params.lcid);

    NS_LOG_INFO("Current MCS = " << m_mcs);
    // printf("Current MCS = %d\n", m_mcs);
    GetSecond DstInfoOf;
    auto itDst = m_dstMap.find(params.dstL2Id);
    NS_ABORT_MSG_IF(itDst == m_dstMap.end(), "Destination " << params.dstL2Id << " info not found");
    
    for (const auto& lcg : DstInfoOf(*itDst)->GetNrSlLCG())
    {
        if (lcg.second->Contains(params.lcid))
        {
            NS_LOG_INFO("Updating buffer status for LC in LCG: "
                        << +lcg.first << " LC: " << +params.lcid << " dstL2Id: " << params.dstL2Id
                        << " queue size: " << params.txQueueSize);
            lcg.second->UpdateInfo(params);
            return;
        }
    }
    // Fail miserably because we didn't find any LC
    NS_FATAL_ERROR("The LC does not exist. Can't update");
}

} // namespace ns3
