/*
 * Copyright (c) 2020 University of Padova, Dep. of Information Engineering, SIGNET lab.
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
 * Modified by: Jingxiang Yu <yujx@mails.swust.edu.cn> (alternate: yujx.res@gmail.com)
 */


#include "three-gpp-antenna-model.h"

#include "antenna-model.h"

#include <ns3/double.h>
#include <ns3/log.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThreeGppAntennaModel");

NS_OBJECT_ENSURE_REGISTERED(ThreeGppAntennaModel);

TypeId
ThreeGppAntennaModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThreeGppAntennaModel")
                            .SetParent<AntennaModel>()
                            .SetGroupName("Antenna")
                            .AddConstructor<ThreeGppAntennaModel>()
                            .AddAttribute("VerticalBeamwidth",
                                          "The vertical beamwidth in degrees",
                                          DoubleValue(65.0),
                                          MakeDoubleAccessor(&ThreeGppAntennaModel::m_verticalBeamwidthDegrees),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("HorizontalBeamwidth",
                                          "The horizontal beamwidth in degrees",
                                          DoubleValue(65.0),
                                          MakeDoubleAccessor(&ThreeGppAntennaModel::m_horizontalBeamwidthDegrees),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("MaxAttenuation",
                                          "The maximum attenuation in dB",
                                          DoubleValue(30.0),
                                          MakeDoubleAccessor(&ThreeGppAntennaModel::m_aMax),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("SlaV",
                                          "The side-lobe attenuation in dB",
                                          DoubleValue(30.0),
                                          MakeDoubleAccessor(&ThreeGppAntennaModel::m_slaV),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("AntennaElementGain",
                                          "The maximum antenna element gain in dBi",
                                          DoubleValue(8.0),
                                          MakeDoubleAccessor(&ThreeGppAntennaModel::m_geMax),
                                          MakeDoubleChecker<double>());
    return tid;
}

ThreeGppAntennaModel::ThreeGppAntennaModel()
    : m_verticalBeamwidthDegrees{65},
      m_horizontalBeamwidthDegrees{65},
      m_aMax{30},
      m_slaV{30},
      m_geMax{8.0}
{
}

ThreeGppAntennaModel::~ThreeGppAntennaModel()
{
}

double
ThreeGppAntennaModel::GetVerticalBeamwidth() const
{
    return m_verticalBeamwidthDegrees;
}

double
ThreeGppAntennaModel::GetHorizontalBeamwidth() const
{
    return m_horizontalBeamwidthDegrees;
}

double
ThreeGppAntennaModel::GetSlaV() const
{
    return m_slaV;
}

double
ThreeGppAntennaModel::GetMaxAttenuation() const
{
    return m_aMax;
}

double
ThreeGppAntennaModel::GetAntennaElementGain() const
{
    return m_geMax;
}

double
ThreeGppAntennaModel::GetGainDb(Angles a)
{
    NS_LOG_FUNCTION(this << a);

    double phiDeg = RadiansToDegrees(a.GetAzimuth());
    double thetaDeg = RadiansToDegrees(a.GetInclination());

    NS_ASSERT_MSG(-180.0 <= phiDeg && phiDeg <= 180.0, "Out of boundaries: phiDeg=" << phiDeg);
    NS_ASSERT_MSG(0.0 <= thetaDeg && thetaDeg <= 180.0, "Out of boundaries: thetaDeg=" << thetaDeg);

    // compute the radiation power pattern using equations in table 7.3-1 in
    // 3GPP TR 38.901
    double vertGain = -std::min(m_slaV,
                                12 * pow((thetaDeg - 90) / m_verticalBeamwidthDegrees,
                                         2)); // vertical cut of the radiation power pattern (dB)
    double horizGain = -std::min(m_aMax,
                                 12 * pow(phiDeg / m_horizontalBeamwidthDegrees,
                                          2)); // horizontal cut of the radiation power pattern (dB)

    double gainDb =
        m_geMax - std::min(m_aMax, -(vertGain + horizGain)); // 3D radiation power pattern (dB)

    NS_LOG_DEBUG("gain=" << gainDb << " dB");
    return gainDb;
}

} // namespace ns3
