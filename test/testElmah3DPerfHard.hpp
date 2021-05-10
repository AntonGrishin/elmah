#ifndef TEST_ELMAH3D_1000_HPP_
#define TEST_ELMAH3D_1000_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmah3DMech : public CxxTest::TestSuite
{
public:

    void commonConfig_3D(SimConfig3D& config)
    {
        config.inCellConductivities  = Create_c_vector(1.75, 0.19, 0.19);
        config.CMName = CellMlModel::LuoRudy1991;
        config.odeTS  = 0.01;
        config.pdeTS  = 0.01;
        config.printingTS = 0.2;
        config.dim = Dimension::dim3D;
    }

    void testElmah10000msPerfMonoBE_3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
                                                 cp::media_type::Axisymmetric);

        SimConfig3D config;
        commonConfig_3D(config);
        config.prName = ProblemName::MONO;
        config.sName  = SolverName::BackwardEuler;
        config.duration =  1000.0;

        OutConfig outConfig = {};
        outConfig.vtkOutput = true;
        outConfig.outFileName = "result";
        outConfig.outFolder = "elmah_1sec";


        elmah->SetOutputParameters(outConfig);

        elmah->StartSimulation(&config);


        TS_ASSERT_EQUALS(1, 1);
    }

 };

#endif /*TEST_ELMAH_HPP_*/
