#include "elmah_core_chaste.hpp"
#include "elmah_core_factory.hpp"
#include "PetscTools.hpp"
#include "PlaneStimulusCellFactory.hpp"
#include "NonlinearElasticityTools.hpp"

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


    if(simConfig.prName == ProblemName::ELECTRO_MECHA)
    {
        boost::shared_ptr<AbstractIvpOdeSolver> solver = CreateSolver(simConfig.sName);
        AbstractCellFactory2D cell_factory(simConfig.CMName, solver);
        SolveElMech(&cell_factory);
    }
    else
    {
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
                printf("BE\n");
                CellMlModel cmname = static_cast<CellMlModel>(simConfig.CMName + 20);
                printf("cmname %u\n", cmname);
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

}


template<typename T>
void ElmahCoreChaste::SolveElMech(T* cell_factory)
{
    if(PetscTools::AmMaster())
        Timer::Reset();
    {
        TetrahedralMesh<2, 2> electrics_mesh;
        QuadraticMesh<2> mechanics_mesh;

        // could (should?) use finer electrics mesh, but keeping electrics simulation time down
        TrianglesMeshReader<2, 2> reader1("mesh/test/data/annuli/circular_annulus_960_elements");
        electrics_mesh.ConstructFromMeshReader(reader1);

        TrianglesMeshReader<2, 2> reader2("mesh/test/data/annuli/circular_annulus_960_elements_quad", 2 /*quadratic elements*/);
        mechanics_mesh.ConstructFromMeshReader(reader2);

        std::vector<unsigned> fixed_nodes;
        std::vector<c_vector<double, 2> > fixed_node_locations;
        for (unsigned i = 0; i < mechanics_mesh.GetNumNodes(); i++)
        {
            double x = mechanics_mesh.GetNode(i)->rGetLocation()[0];
            double y = mechanics_mesh.GetNode(i)->rGetLocation()[1];

            if (fabs(x) < 1e-6 && fabs(y + 0.5) < 1e-6) // fixed point (0.0,-0.5) at bottom of mesh
            {
                fixed_nodes.push_back(i);
                c_vector<double, 2> new_position;
                new_position(0) = x;
                new_position(1) = y;
                fixed_node_locations.push_back(new_position);
            }
            if (fabs(x) < 1e-6 && fabs(y - 0.5) < 1e-6) // constrained point (0.0,0.5) at top of mesh
            {
                fixed_nodes.push_back(i);
                c_vector<double, 2> new_position;
                new_position(0) = x;
                new_position(1) = ElectroMechanicsProblemDefinition<2>::FREE;
                fixed_node_locations.push_back(new_position);
            }
        }

        ElectroMechanicsProblemDefinition<2> problem_defn(mechanics_mesh);

        problem_defn.SetContractionModel(KERCHOFFS2003, 0.1);
        problem_defn.SetUseDefaultCardiacMaterialLaw(COMPRESSIBLE);
        //problem_defn.SetZeroDisplacementNodes(fixed_nodes);
        problem_defn.SetFixedNodes(fixed_nodes, fixed_node_locations);
        problem_defn.SetMechanicsSolveTimestep(1.0);

        FileFinder finder("heart/test/data/fibre_tests/circular_annulus_960_elements.ortho", RelativeTo::ChasteSourceRoot);
        problem_defn.SetVariableFibreSheetDirectionsFile(finder, false);

        problem_defn.SetSolveUsingSnes();

        std::vector<BoundaryElement<1, 2>*> boundary_elems;
        for (TetrahedralMesh<2, 2>::BoundaryElementIterator iter
             = mechanics_mesh.GetBoundaryElementIteratorBegin();
             iter != mechanics_mesh.GetBoundaryElementIteratorEnd();
             ++iter)
        {
            ChastePoint<2> centroid = (*iter)->CalculateCentroid();
            double r = sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);

            if (r < 0.4)
            {
                BoundaryElement<1, 2>* p_element = *iter;
                boundary_elems.push_back(p_element);
            }
        }

        problem_defn.SetApplyNormalPressureOnDeformedSurface(boundary_elems, -1.0 /*1 KPa is about 8mmHg*/);
        problem_defn.SetNumIncrementsForInitialDeformation(3);

        CardiacElectroMechanicsProblem<2, 1> problem(COMPRESSIBLE,
                                                     MONODOMAIN,
                                                     &electrics_mesh,
                                                     &mechanics_mesh,
                                                     cell_factory,
                                                     &problem_defn,
                                                     "Elmah_mech");

        problem.SetOutputDeformationGradientsAndStress(10.0 /*how often (in ms) to write - should be a multiple of mechanics timestep*/);

        if(PetscTools::AmMaster())
            Timer::PrintAndReset("ElMech Reading and preparations");
        problem.Solve();
    }
    if(PetscTools::AmMaster())
        Timer::Print("ElMech Solve");
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
        default:
            throw("Unimplemented");
    }
    if(PetscTools::AmMaster())
    {
        printf("\n\n");
        Timer::Print(__FUNCTION__);
        printf("\n\n");
    }
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