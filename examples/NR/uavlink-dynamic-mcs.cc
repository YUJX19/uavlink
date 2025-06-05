#include "uavlink-dynamic-mcs.h"

/**
 * \brief Establish shared memory link with the given ID and set operational lock.
 *
 * \param[in] id  Shared memory ID, must be the same in both Python and ns-3.
 */
namespace ns3
{
NS_LOG_COMPONENT_DEFINE("uavlink-dynamic-mcs");

NS_OBJECT_ENSURE_REGISTERED(UAVLinkSINR);

UAVLinkSINR::UAVLinkSINR()
{
    auto interface = UavLinkMsgInterface::Get();
    interface->SetIsMemoryCreator(false);
    interface->SetUseVector(false);
    interface->SetHandleFinish(true);
}

UAVLinkSINR::~UAVLinkSINR()
{
}

TypeId
UAVLinkSINR::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UAVLinkSINR").SetParent<Object>().SetGroupName("UavLink").AddConstructor<UAVLinkSINR>();
    return tid;
}


/**
 * \brief Set the SINR values for all Resource Block Groups (RBGs).
 *
 * \param[in] sinr  An array of SINR values to be set.
 */
void
UAVLinkSINR::SetSinr(const std::array<double, MAX_RBG_NUM>& sinr)
{
    // Ensure the input array size matches the expected size.
    if (sinr.size() != MAX_RBG_NUM)
    {
        NS_LOG_ERROR("SetSinr: Input vector size is not equal to 40.");
        return;
    }
    UavLinkMsgInterfaceImpl<RbgSinrFeature, RbgSinrPrediction>* msgInterface =
        UavLinkMsgInterface::Get()->GetInterface<RbgSinrFeature, RbgSinrPrediction>();
    msgInterface->CppSendBegin();

    // Store SINR values into shared memory
    for (size_t i = 0; i < sinr.size(); ++i)
    {
        // std::cout << "RB_" << i << ": " << sinr[i] << ",";
        msgInterface->GetCpp2PyStruct()->sinrPerRbg[i] = sinr[i];
    }
    // std::cout << "]" << std::endl;
    msgInterface->CppSendEnd();
}

/**
 * \brief Retrieve the predicted SINR values from shared memory.
 *
 * \returns An array containing the predicted SINR values.
 */
std::array<double, MAX_RBG_NUM>
UAVLinkSINR::GetSinr()
{
    std::array<double, MAX_RBG_NUM> sinr = {0.0}; // Initialize the array with default values

    UavLinkMsgInterfaceImpl<RbgSinrFeature, RbgSinrPrediction>* msgInterface =
        UavLinkMsgInterface::Get()->GetInterface<RbgSinrFeature, RbgSinrPrediction>();
    msgInterface->CppRecvBegin();

    // Retrieve predicted SINR values from shared memory
    for (size_t i = 0; i < sinr.size(); ++i)
    {
        sinr[i] = msgInterface->GetPy2CppStruct()->predictedSinrPerRbg[i];
    }

    msgInterface->CppRecvEnd();
    return sinr; // Return the predicted SINR values
}

/**
 * \brief Set the SINR and corresponding MCS values.
 *
 * \param[in] sinr  An array of SINR values.
 * \param[in] macs  An array of MCS values corresponding to the SINR values.
 */
void
UAVLinkSINR::SetMcs(const std::array<double, MAX_RBG_NUM>& sinr, const std::array<double, MAX_RBG_NUM>& macs)
{
    // Ensure input arrays have the expected size
    if (sinr.size() != MAX_RBG_NUM || macs.size() != MAX_RBG_NUM)
    {
        NS_LOG_ERROR("SetMacs: Input vector sizes are not equal to 40.");
        return;
    }
    UavLinkMsgInterfaceImpl<RbgSinrFeature, RbgSinrPrediction>* msgInterface =
        UavLinkMsgInterface::Get()->GetInterface<RbgSinrFeature, RbgSinrPrediction>();
    msgInterface->CppSendBegin();


    // Store SINR and MCS values into shared memory
    for (size_t i = 0; i < MAX_RBG_NUM; ++i)
    {
        msgInterface->GetCpp2PyStruct()->sinrPerRbg[i] = sinr[i];
        msgInterface->GetCpp2PyStruct()->selectedMcs[i] = macs[i]; 
    }
    std::cout << std::endl;
    msgInterface->CppSendEnd();
}

/**
 * \brief Set the SINR and corresponding MCS values for 6-DoF UAVs.
 *
 * \param[in] sinr_6dof  An array of SINR values for 6-DoF UAVs.
 * \param[in] macs       An array of MCS values corresponding to the SINR values.
 */
void
UAVLinkSINR::SetMcs_6dof(const std::array<double, MAX_RBG_NUM_6DOF>& sinr_6dof, const std::array<double, MAX_RBG_NUM>& macs){
    // Ensure input arrays have the expected size
    if (sinr_6dof.size() != MAX_RBG_NUM_6DOF || macs.size() != MAX_RBG_NUM)
    {
        NS_LOG_ERROR("SetMacs: Input vector sizes are not equal to 40.");
        return;
    }

    UavLinkMsgInterfaceImpl<RbgSinrFeature, RbgSinrPrediction>* msgInterface =
        UavLinkMsgInterface::Get()->GetInterface<RbgSinrFeature, RbgSinrPrediction>();
    msgInterface->CppSendBegin();

    // Store MCS values into shared memory
    for (size_t i = 0; i < MAX_RBG_NUM; ++i)
    {
        // msgInterface->GetCpp2PyStruct()->wbsinr[i] = sinr_6dof[i];
        msgInterface->GetCpp2PyStruct()->selectedMcs[i] = macs[i];
    }

    // Store SINR values for 6-DoF UAVs into shared memory
    for (size_t i = 0; i < MAX_RBG_NUM_6DOF; ++i)
    {
        msgInterface->GetCpp2PyStruct()->sinrPerRbg_6dof[i] = sinr_6dof[i];
    }
    std::cout << std::endl;
    msgInterface->CppSendEnd();
}

/**
 * \brief Retrieve the predicted MCS values from shared memory.
 *
 * \returns An array containing the predicted MCS values.
 */
std::array<double, MAX_RBG_NUM>
UAVLinkSINR::GetMcs()
{
    std::array<double, MAX_RBG_NUM> macs = {0.0}; // Initialize the array with default values

    UavLinkMsgInterfaceImpl<RbgSinrFeature, RbgSinrPrediction>* msgInterface =
        UavLinkMsgInterface::Get()->GetInterface<RbgSinrFeature, RbgSinrPrediction>();
    msgInterface->CppRecvBegin();

    // Retrieve predicted MCS values from shared memory
    for (size_t i = 0; i < MAX_RBG_NUM; ++i)
    {
        macs[i] = msgInterface->GetPy2CppStruct()->predictedMcs[i]*10.0;
    }

    msgInterface->CppRecvEnd();
    return macs; // Return the predicted MCS values
}


} // namespace ns3