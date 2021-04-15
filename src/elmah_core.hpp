#ifndef ELMAH_HPP_
#define ELMAH_HPP_

#include <string>
#include "elmah_defs.hpp"
#include "elmah_cell_factory.hpp"


class IElmahCore 
{
public:
    virtual ~IElmahCore(){}
    virtual void GenerateMesh(const std::string& path) = 0;

    virtual void GenerateMesh(const MeshConfig& meshConfig) = 0;

    virtual void StartSimulation(SimConfig* simConfig) = 0;

    virtual void SetOutputParameters(const OutConfig& outConfig)  = 0;
};



#endif /*ELMAH_HPP_*/
