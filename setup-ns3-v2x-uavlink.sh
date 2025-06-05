#!/bin/bash

cd ..
# Clone and checkout ns-3-dev
git clone https://gitlab.com/cttc-lena/ns-3-dev.git
cd ns-3-dev
git checkout tags/ns-3-dev-v2x-v1.1 -b ns-3-dev-v2x-uavlink-1.0

# Clone and checkout nr module
cd contrib
git clone https://gitlab.com/cttc-lena/nr.git
cd nr
git checkout tags/v2x-1.1 -b v2x-uavlink-1.0
cd ../../../


cp uavlink/nr-sidelink/nr-phy-mac-common.h ns-3-dev/contrib/nr/model/nr-phy-mac-common.h
cp uavlink/nr-sidelink/nr-sl-ue-mac-scheduler-fixed-mcs.h ns-3-dev/contrib/nr/model/nr-sl-ue-mac-scheduler-fixed-mcs.h
cp uavlink/nr-sidelink/three-gpp-antenna-model.cc ns-3-dev/src/antenna/model/three-gpp-antenna-model.cc
cp uavlink/nr-sidelink/nr-sl-ue-mac.cc ns-3-dev/contrib/nr/model/nr-sl-ue-mac.cc
cp uavlink/nr-sidelink/nr-sl-ue-mac-harq.h ns-3-dev/contrib/nr/model/nr-sl-ue-mac-harq.h
cp uavlink/nr-sidelink/nr-sl-ue-mac-harq.cc ns-3-dev/contrib/nr/model/nr-sl-ue-mac-harq.cc
cp uavlink/nr-sidelink/nr-spectrum-phy.cc ns-3-dev/contrib/nr/model/nr-spectrum-phy.cc


ln -s /uavlink/ /ns-3-dev/contrib/uavlink

# Build the project
cd ns-3-dev
./ns3 configure --enable-tests --enable-examples
./ns3 build
