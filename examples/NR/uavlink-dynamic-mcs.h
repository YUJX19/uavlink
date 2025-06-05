/*
 * Copyright (c) 2025 Southwest University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Jingxiang Yu <yujx@mails.swust.edu.cn> (alternate: yujx.res@gmail.com)
 *
 */


#pragma once
#include "ns3/uavlink-module.h"
#include "ns3/core-module.h"

namespace ns3
{
#define MAX_RBG_NUM 10  ///< Maximum Resource Block Groups (RBG) for UAVLink
#define MAX_RBG_NUM_6DOF 16  ///< Maximum RBGs for UAVs with 6-DoF mobility

/**
 * \brief Resource Block Group (RBG) SINR Features shared between ns-3 and Python.
 *
 * This struct encapsulates key SINR measurements required for LSTM-based prediction
 * and adaptive MCS selection in UAV-to-UAV (U2U) sidechain communications.
 * The AI model dynamically adjusts MCS levels under fixed BLER constraints to enhance 
 * communication efficiency and reliability in high-mobility scenarios.
 */
struct RbgSinrFeature
{
    std::array<double, MAX_RBG_NUM> sinrPerRbg; ///< SINR per RBG for standard UAV mobility
    std::array<double, MAX_RBG_NUM_6DOF> sinrPerRbg_6dof; ///< SINR per RBG for UAVs in 6-DoF mobility
    std::array<double, MAX_RBG_NUM> selectedMcs; ///< Current MCS index per RBG
};

/**
 * \brief Predicted RBG-level SINR and Adaptive MCS values using AI-based LSTM model.
 *
 * This struct stores the SINR predictions and dynamically optimized MCS levels
 * computed by an external Python-based AI model via shared memory.
 */
struct RbgSinrPrediction
{
    std::array<double, MAX_RBG_NUM> predictedSinrPerRbg; ///< Predicted SINR per RBG
    std::array<double, MAX_RBG_NUM_6DOF> predictedSinrPerRbg_6dof; ///< Predicted SINR per RBG in 6-DoF
    std::array<double, MAX_RBG_NUM> predictedMcs; ///< AI-optimized MCS index per RBG
};

/**
 * \brief AI-based SINR prediction and adaptive MCS selection for UAVLink.
 *
 * This class enables real-time SINR prediction and dynamic MCS adaptation 
 * using AI-based LSTM models in UAV-to-UAV (U2U) sidechain communications.
 * It exchanges SINR data with Python via shared memory, allowing AI-based optimization
 * of link performance under fixed BLER constraints in high-mobility and interference-prone environments.
 */
class UAVLinkSINR : public Object
{
  public:
    UAVLinkSINR();
    ~UAVLinkSINR() override;
    static TypeId GetTypeId();

    void SetSinr(const std::array<double, MAX_RBG_NUM>& sinr);
    std::array<double, MAX_RBG_NUM> GetSinr();

    void SetMcs(const std::array<double, MAX_RBG_NUM>& sinr, const std::array<double, MAX_RBG_NUM>& mcs);
    void SetMcs_6dof(const std::array<double, MAX_RBG_NUM_6DOF>& sinr_6dof, const std::array<double, MAX_RBG_NUM>& mcs);
    std::array<double, MAX_RBG_NUM> GetMcs();
};

} // namespace ns3