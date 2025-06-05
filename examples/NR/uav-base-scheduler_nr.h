/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef UAV_BASE_SCHEDULER_H
#define UAV_BASE_SCHEDULER_H

#include <ns3/nr-sl-ue-mac-scheduler-fixed-mcs.h>
#include <ns3/nstime.h>
#include <ns3/log.h>


namespace ns3
{

class UavBaseScheduler : public NrSlUeMacSchedulerFixedMcs
{
public:
    static TypeId GetTypeId(void);
    UavBaseScheduler();

    virtual ~UavBaseScheduler();


    /**
     * \brief Override the RLC Buffer Status scheduling function
     * \param params The buffer status parameters from the RLC layer
     */
    void DoSchedNrSlRlcBufferReq(
        const NrSlMacSapProvider::NrSlReportBufferStatusParameters& params) override;


    /**
     * \brief Calculate the transport block size for input parameters
     *
     * For a given modulation and coding scheme, number of subchannels,
     * subchannel size, and symbols per slot, calculate the resulting transport
     * block size in bytes.
     * \param nrAmc pointer to modulation and coding model
     * \param dstMcs MCS value to use
     * \param symbolsPerSlot number of symbols to assume in a slot
     * \param availableSubChannels number of subchannels
     * \param subChannelSize subchannel size in physical resource blocks
     * \return transport block size in bytes
     */
    uint32_t CalculateTbSize(Ptr<const NrAmc> nrAmc,
                             uint8_t dstMcs,
                             uint16_t symbolsPerSlot,
                             uint16_t availableSubChannels,
                             uint16_t subChannelSize) const;

private:
    // Member variables
    // Ptr<NetDevice> m_device; //!< The NetDevice associated with this scheduler


};


} // namespace ns3

#endif // UAV_BASE_SCHEDULER_H
