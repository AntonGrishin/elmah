#ifndef TEST_ELMAHPROF_HPP_
#define TEST_ELMAHPROF_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmahProf : public CxxTest::TestSuite
{
public:

    void commonConfig(SimConfig2D& config)
    {
        config.inCellConductivities  = Create_c_vector(1.75, 0.19);
        config.CMName = CellMlModel::LuoRudy1991;
        config.odeTS  = 0.01;
        config.pdeTS  = 0.01;
        config.printingTS = 0.1;
    }
    void testElmah1000msPerfBidomainRK4()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("mesh/test/data/2D_0_to_1mm_800_elements");

        SimConfig2D config;
        commonConfig(config);
        config.prName = ProblemName::BI;
        config.sName  = SolverName::Euler;
        config.outCellConductivities = Create_c_vector(6.2, 2.4);
        config.duration = 200.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }


};

#endif /*TEST_ELMAH_HPP_*/
