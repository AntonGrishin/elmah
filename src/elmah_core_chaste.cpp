#include "elmah_core_chaste.hpp"
#include "elmah_core_factory.hpp"
#include "PetscTools.hpp"

ElmahCoreChaste::ElmahCoreChaste()
{
    HeartConfig::Instance()->SetSurfaceAreaToVolumeRatio(1400); // 1/cm
    HeartConfig::Instance()->SetCapacitance(1.0); // uF/cm^2
}

ElmahCoreChaste::~ElmahCoreChaste()
{
}


void ElmahCoreChaste::GenerateMesh(const std::string& path, cp::media_type AxisType)
{
    HeartConfig::Instance()->SetMeshFileName(path, AxisType);
}


void ElmahCoreChaste::GenerateMesh(const MeshConfig& meshConfig) {}

template<typename T, unsigned size>
void ElmahCoreChaste::StartSimulation(const T& simConfig)
{
    if(simConfig.inCellConductivities.size())
        SetIntracellularConductivities<size>(simConfig.inCellConductivities);

    if(simConfig.outCellConductivities.size() && simConfig.prName != ProblemName::MONO)
        SetExtracellularConductivities<size>(simConfig.outCellConductivities);

    if(simConfig.duration)
        SetDuration(simConfig.duration);

    if(simConfig.odeTS && simConfig.pdeTS && simConfig.printingTS)
    {
        SetOdePdeAndPrintingTimeSteps(simConfig.odeTS,simConfig.pdeTS,simConfig.printingTS);
    }


    if(simConfig.dim == Dimension::dim2D)
    {
        boost::shared_ptr<AbstractIvpOdeSolver> solver = CreateSolver(simConfig.sName);
        AbstractCellFactory2D cell_factory(simConfig.CMName, solver);
        SolveProblem<2>(simConfig.prName, &cell_factory);
    }
    else
    {
        if(simConfig.sName == SolverName::BackwardEuler)
        {
            CellMlModel cmname = static_cast<CellMlModel>(simConfig.CMName + 20);
            AbstractCellFactory3D cell_factory(cmname);
            SolveProblem<3>(simConfig.prName, &cell_factory);
        }
        else
        {
            CellMlModel cmname = static_cast<CellMlModel>(simConfig.CMName + 10);

            AbstractCellFactory3DCVode cell_factory(cmname);
            SolveProblem<3>(simConfig.prName, &cell_factory);
        }
    }


}
    
template<unsigned size, typename T>
void ElmahCoreChaste::SolveProblem(const ProblemName& problemName, T* cell_factory)
{
    if(PetscTools::AmMaster())
        Timer::Reset();

    switch(problemName)
    {
        case ProblemName::MONO:
        {
            MonodomainProblem<size> problem(cell_factory);
            problem.Initialise();
            problem.Solve();
            break;
        }
        case ProblemName::BI:
        {
            BidomainProblem<size> problem(cell_factory);
            problem.Initialise();
            problem.Solve();
            break;
        }
        // case ProblemName::ELECTRO_MECHA:
        // {
        //     CardiacElectroMechanicsProblem<size, 1> problem(cell_factory);
        //     Timer::Reset();
        //     problem.Initialise();
        //     problem.Solve();
        //     Timer::Print(__FUNCTION__);
        //     break;
        // }
        default:
            throw("Unimplemented");
    }
    if(PetscTools::AmMaster())
        Timer::Print(__FUNCTION__);
}

void ElmahCoreChaste::SetOutputParameters(const OutConfig& outConfig)
{
    if(outConfig.vtkOutput)
        HeartConfig::Instance()->SetVisualizeWithVtk(true);
    if(outConfig.outFolder.size())
        HeartConfig::Instance()->SetOutputDirectory(outConfig.outFolder);
    if(outConfig.outFileName.size())
        HeartConfig::Instance()->SetOutputFilenamePrefix(outConfig.outFileName);
}

void ElmahCoreChaste::SetDuration(double duration)
{
    HeartConfig::Instance()->SetSimulationDuration(duration); //ms
}

void ElmahCoreChaste::SetOdePdeAndPrintingTimeSteps(double ode, double pde, double printing)
{
    HeartConfig::Instance()->SetOdePdeAndPrintingTimeSteps(ode, pde, printing);
}