#pragma once

#include "elmah_defs.hpp"
#include "elmah_core.hpp"
#include "elmah_cell_factory.hpp"
#include "PlaneStimulusCellFactory.hpp"
#include "NonlinearElasticityTools.hpp"
#include "QuadraticMesh.hpp"

class ElmahCoreChaste : public IElmahCore
{
public:

    ElmahCoreChaste();
    ~ElmahCoreChaste();
    void GenerateMesh(std::string path);


    void GenerateMesh(const std::string& path, cp::media_type AxisType = cp::media_type::NoFibreOrientation);
    

    void GenerateMesh(const MeshConfig& meshConfig);

    template<typename T, unsigned size>
    void StartSimulation(const T& simConfig);

    void StartSimulation(SimConfig* simConfig)
    {
        SimConfig2D* simConf2D = dynamic_cast<SimConfig2D*>(simConfig);
        if(simConf2D)
        {
            StartSimulation<SimConfig2D, 2>(*simConf2D);
            return;
        }
    
        SimConfig3D* simConf3D = dynamic_cast<SimConfig3D*>(simConfig);
        if(simConf3D)
        {
            StartSimulation<SimConfig3D, 3>(*simConf3D);
            return;
        }
    }

    void SetOutputParameters(const OutConfig& outConfig);

private:

    template<unsigned size, typename T>
    void SolveProblem(const ProblemName& problem, T* cell_factory);

    template<typename T>
    void SolveElMech(T* cell_factory);

    template<typename T>
    void SolveElMech3D(T* cell_factory);

    void SetDuration(double duration);

    template<unsigned DIM>
    void SetIntracellularConductivities(c_vector<double, DIM> cond)
    {
        HeartConfig::Instance()->SetIntracellularConductivities(cond);
    }

    template<unsigned DIM>
    void SetExtracellularConductivities(c_vector<double, DIM> cond)
    {
        HeartConfig::Instance()->SetExtracellularConductivities(cond);
    }

    void SetOdePdeAndPrintingTimeSteps(double ode, double pde, double printing);

    TetrahedralMesh<2, 2> m_electrics_mesh;
    QuadraticMesh<2> m_mechanics_mesh;
    std::string      m_ortho_mesh;
    
    TetrahedralMesh<3, 3> m_electrics_mesh3D;
    QuadraticMesh<3> m_mechanics_mesh3D;
};
