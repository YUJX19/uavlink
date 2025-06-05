#include "uavlink-dynamic-mcs.h"
#include <ns3/uavlink-module.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(uavlink_py, m)
{
    namespace py = pybind11;

    py::class_<ns3::RbgSinrFeature>(m, "PyEnvStruct")
          .def(py::init<>())
          .def_property("sinrPerRbg",
               [](const ns3::RbgSinrFeature& obj) {
               return obj.sinrPerRbg;
               },
               [](ns3::RbgSinrFeature& obj, py::array_t<double> array) {
                    if (array.size() != obj.sinrPerRbg.size()) {
                    throw std::runtime_error("Invalid size for sinrPerRbg.");
                    }
                    std::copy(array.data(), array.data() + obj.sinrPerRbg.size(), obj.sinrPerRbg.begin());
               })
          .def_property("sinrPerRbg_6dof",
               [](const ns3::RbgSinrFeature& obj) {
                    return obj.sinrPerRbg_6dof;
               },
               [](ns3::RbgSinrFeature& obj, py::array_t<double> array) {
                    if (array.size() != obj.sinrPerRbg_6dof.size()) {
                    throw std::runtime_error("Invalid size for sinrPerRbg_6dof.");
                    }
                    std::copy(array.data(), array.data() + obj.sinrPerRbg_6dof.size(), obj.sinrPerRbg_6dof.begin());
               })
          .def_property("selectedMcs",
               [](const ns3::RbgSinrFeature& obj) {
                    return obj.selectedMcs;
               },
               [](ns3::RbgSinrFeature& obj, py::array_t<double> array) {
                    if (array.size() != obj.selectedMcs.size()) {
                         throw std::runtime_error("Invalid size for selectedMcs.");
                    }
                    std::copy(array.data(), array.data() + obj.selectedMcs.size(), obj.selectedMcs.begin());
               });

    py::class_<ns3::RbgSinrPrediction>(m, "PyActStruct")
          .def(py::init<>())
          .def_property("predictedSinrPerRbg",
               [](const ns3::RbgSinrPrediction& obj) {
                    return py::array_t<double>(obj.predictedSinrPerRbg.size(), obj.predictedSinrPerRbg.data());
               },
               [](ns3::RbgSinrPrediction& obj, py::array_t<double> array) {
                    if (array.size() != obj.predictedSinrPerRbg.size()) {
                    throw std::runtime_error("Invalid size for predictedSinrPerRbg.");
                    }
                    std::copy(array.data(), array.data() + obj.predictedSinrPerRbg.size(), obj.predictedSinrPerRbg.begin());
               })
          .def_property("predictedSinrPerRbg_6dof",
               [](const ns3::RbgSinrPrediction& obj) {
                    return py::array_t<double>(obj.predictedSinrPerRbg_6dof.size(), obj.predictedSinrPerRbg_6dof.data());
               },
               [](ns3::RbgSinrPrediction& obj, py::array_t<double> array) {
                    if (array.size() != obj.predictedSinrPerRbg_6dof.size()) {
                    throw std::runtime_error("Invalid size for predictedSinrPerRbg_6dof.");
                    }
                    std::copy(array.data(), array.data() + obj.predictedSinrPerRbg_6dof.size(), obj.predictedSinrPerRbg_6dof.begin());
               })
          .def_property("predictedMcs",
               [](const ns3::RbgSinrPrediction& obj) {
                    return py::array_t<double>(obj.predictedMcs.size(), obj.predictedMcs.data());
               },
               [](ns3::RbgSinrPrediction& obj, py::array_t<double> array) {
                    if (array.size() != obj.predictedMcs.size()) {
                         throw std::runtime_error("Invalid size for predictedMcs.");
                    }
                    std::copy(array.data(), array.data() + obj.predictedMcs.size(), obj.predictedMcs.begin());
               });

    py::class_<ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>>(
        m, "UavLinkMsgInterfaceImpl")
        .def(py::init<bool, bool, bool, uint32_t, const char*, const char*, const char*, const char*>())
        .def("PyRecvBegin",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::PyRecvBegin)
        .def("PyRecvEnd",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::PyRecvEnd)
        .def("PySendBegin",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::PySendBegin)
        .def("PySendEnd",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::PySendEnd)
        .def("PyGetFinished",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::PyGetFinished)
        .def("GetCpp2PyStruct",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::GetCpp2PyStruct,
             py::return_value_policy::reference)
        .def("GetPy2CppStruct",
             &ns3::UavLinkMsgInterfaceImpl<ns3::RbgSinrFeature, ns3::RbgSinrPrediction>::GetPy2CppStruct,
             py::return_value_policy::reference);
}