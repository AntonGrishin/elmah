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


void ElmahCoreChaste::GenerateMesh(const MeshConfig& meshConfig)
{
    if(meshConfig.dim == Dimension::dim2D)
    {
        if(!meshConfig.electric_mesh.empty())
        {
            TrianglesMeshReader<2, 2> reader(meshConfig.electric_mesh);
            m_electrics_mesh.ConstructFromMeshReader(reader);
        }

        if(!meshConfig.mecha_mesh.empty())
        {
            TrianglesMeshReader<2, 2> reader(meshConfig.mecha_mesh, 2 /*quadratic elements*/);
            m_mechanics_mesh.ConstructFromMeshReader(reader);
        }
        
    }
    else if (meshConfig.dim == Dimension::dim3D)
    {
        if(meshConfig.width && meshConfig.heigth && meshConfig.depth && meshConfig.step)
        {
            m_electrics_mesh3D.ConstructRegularSlabMesh(meshConfig.step * 1, meshConfig.width, meshConfig.heigth, meshConfig.depth);
            m_mechanics_mesh3D.ConstructRegularSlabMesh(meshConfig.step * 2, meshConfig.width, meshConfig.heigth, meshConfig.depth);
        }
        else
        {
            if(meshConfig.electric_mesh.empty())
            {
                throw("incorrect setup!");
            }

            if(!meshConfig.electric_mesh.empty())
            {
                TrianglesMeshReader<3, 3> reader(meshConfig.electric_mesh);
                m_electrics_mesh3D.ConstructFromMeshReader(reader);
            }

            if(!meshConfig.mecha_mesh.empty())
            {
                TrianglesMeshReader<3, 3> reader(meshConfig.mecha_mesh, 2 /*quadratic elements*/);
                m_mechanics_mesh3D.ConstructFromMeshReader(reader);
            }
        }
    }
    else
    {
        throw("Unimplemented");
    }
    m_ortho_mesh = meshConfig.ortho_mesh;
}

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
        if(simConfig.dim == Dimension::dim2D)
        {
            AbstractCellFactory2D cell_factory(simConfig.CMName, solver);
            SolveElMech(&cell_factory);
        }
        else if (simConfig.dim == Dimension::dim3D)
        {
            AbstractCellFactory3D cell_factory(simConfig.CMName, solver);
            SolveElMech3D(&cell_factory);
        }
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

}


template<typename T>
void ElmahCoreChaste::SolveElMech(T* cell_factory)
{
    if(PetscTools::AmMaster())
        Timer::Reset();
    {
            std::vector<unsigned> fixed_nodes;
            std::vector<c_vector<double, 2> > fixed_node_locations;
            for (unsigned i = 0; i < m_mechanics_mesh.GetNumNodes(); i++)
            {
                double x = m_mechanics_mesh.GetNode(i)->rGetLocation()[0];
                double y = m_mechanics_mesh.GetNode(i)->rGetLocation()[1];

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

        ElectroMechanicsProblemDefinition<2> problem_defn(m_mechanics_mesh);

        problem_defn.SetContractionModel(KERCHOFFS2003, 0.1);
        problem_defn.SetUseDefaultCardiacMaterialLaw(COMPRESSIBLE);
        problem_defn.SetFixedNodes(fixed_nodes, fixed_node_locations);
        problem_defn.SetMechanicsSolveTimestep(1.0);

        FileFinder finder(m_ortho_mesh, RelativeTo::ChasteSourceRoot);
        problem_defn.SetVariableFibreSheetDirectionsFile(finder, false);

        problem_defn.SetSolveUsingSnes();

        std::vector<BoundaryElement<1, 2>*> boundary_elems;
        for (TetrahedralMesh<2, 2>::BoundaryElementIterator iter
             = m_mechanics_mesh.GetBoundaryElementIteratorBegin();
             iter != m_mechanics_mesh.GetBoundaryElementIteratorEnd();
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
                                                     &m_electrics_mesh,
                                                     &m_mechanics_mesh,
                                                     cell_factory,
                                                     &problem_defn,
                                                     HeartConfig::Instance()->GetOutputDirectory());

        problem.SetOutputDeformationGradientsAndStress(10.0 /*how often (in ms) to write - should be a multiple of mechanics timestep*/);

        if(PetscTools::AmMaster())
            Timer::PrintAndReset("ElMech Reading and preparations");
        problem.Solve();
    }
    if(PetscTools::AmMaster())
        Timer::Print("ElMech Solve");
}


template<typename T>
void ElmahCoreChaste::SolveElMech3D(T* cell_factory)
{
    if(PetscTools::AmMaster())
        Timer::Reset();
    {

        PlaneStimulusCellFactory<CellLuoRudy1991FromCellML, 3u> cell_factory23(-1000*1000);


        std::vector<unsigned> fixed_nodes
          = NonlinearElasticityTools<3u>::GetNodesByComponentValue(m_mechanics_mesh3D, 2, 0.0);

        ElectroMechanicsProblemDefinition<3u> problem_defn(m_mechanics_mesh3D);
        problem_defn.SetContractionModel(KERCHOFFS2003,1.0);
        problem_defn.SetUseDefaultCardiacMaterialLaw(COMPRESSIBLE);
        problem_defn.SetZeroDisplacementNodes(fixed_nodes);
        problem_defn.SetMechanicsSolveTimestep(1.0); // ms

        std::string output_directory = HeartConfig::Instance()->GetOutputDirectory();//"elmah_elmecha3d";
        std::string fibre_file_name = m_ortho_mesh;

        {
            OutputFileHandler handler(output_directory + "Fibres");
            out_stream p_file = handler.OpenOutputFile(fibre_file_name);
            *p_file << m_mechanics_mesh3D.GetNumElements() << "\n"; // first line is number of entries
            std::vector<c_vector<double,3u> > fibre_directions;
            for(unsigned i=0; i<m_mechanics_mesh3D.GetNumElements(); i++)
            {
                double big_x = m_mechanics_mesh3D.GetElement(i)->CalculateCentroid()(0);
                double theta = M_PI/3.0 - 10*big_x*2*M_PI/3.0; // 60 degrees when X=0, -60 when X=0.1;

                c_vector<double,3u> fibre_direction;
                fibre_direction[0] = 0;
                fibre_direction[1] = cos(theta);
                fibre_direction[2] = sin(theta);
                *p_file <<  fibre_direction[0] << " " << fibre_direction[1]  << " " << fibre_direction[2]  // first three entries are fibre direction
                        << " 0 " << -fibre_direction[2] << " " << fibre_direction[1]                       // next three are sheet direction
                        << " 1 0 0\n";                                                                     // then normal to sheet direction
                fibre_directions.push_back(fibre_direction);
            }
            p_file->close();

        }

        FileFinder fibre_file_finder(output_directory + "Fibres/" + fibre_file_name, RelativeTo::ChasteTestOutput);
        problem_defn.SetVariableFibreSheetDirectionsFile(fibre_file_finder, false);

        CardiacElectroMechanicsProblem<3u> problem(COMPRESSIBLE,
                                                   MONODOMAIN,
                                                   &m_electrics_mesh3D,
                                                   &m_mechanics_mesh3D,
                                                   &cell_factory23,
                                                   &problem_defn,
                                                   output_directory);

        if(PetscTools::AmMaster())
            Timer::PrintAndReset("ElMech Reading and preparations");

        problem.Solve();

        MechanicsEventHandler::Headings();
        MechanicsEventHandler::Report();

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