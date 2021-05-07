#ifndef ELMAH_HPP_
#define ELMAH_HPP_

#include <string>
#include "elmah_defs.hpp"
#include "elmah_cell_factory.hpp"


class IElmahCore 
{
public:
    virtual ~IElmahCore() = default;

    virtual void GenerateMesh(const std::string& path, cp::media_type AxisType = cp::media_type::NoFibreOrientation) = 0;

    virtual void GenerateMesh(const MeshConfig& meshConfig) = 0;

    virtual void StartSimulation(SimConfig* simConfig) = 0;

    virtual void SetOutputParameters(const OutConfig& outConfig)  = 0;
};



#endif /*ELMAH_HPP_*/
