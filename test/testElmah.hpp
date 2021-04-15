#ifndef TEST_ELMAH_HPP_
#define TEST_ELMAH_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmah : public CxxTest::TestSuite
{
public:

    void commonConfig(SimConfig2D& config)
    {
        config.duration = 100.0;

        config.inCellConductivities  = Create_c_vector(1.75, 0.19);
        config.outCellConductivities = Create_c_vector(6.2, 2.4);
        config.prName = ProblemName::BI;
        config.CMName = CellMlModel::LuoRudy1991;
        config.odeTS  = 0.01;
        config.pdeTS  = 0.01;
        config.printingTS = 0.1;
    }
    void testElmahBidomainRK4()
    {

        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        elmah->GenerateMesh("mesh/test/data/2D_0_to_1mm_800_elements");

        SimConfig2D config;
        commonConfig(config);

        config.sName  = SolverName::RK4;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmahBidomainRK2()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        elmah->GenerateMesh("mesh/test/data/2D_0_to_1mm_800_elements");

        SimConfig2D config;
        commonConfig(config);

        config.sName  = SolverName::RK2;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmahBidomainFE()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        elmah->GenerateMesh("mesh/test/data/2D_0_to_1mm_800_elements");

        SimConfig2D config;
        commonConfig(config);

        config.sName  = SolverName::Euler;

        elmah->StartSimulation(&config);

 
        TS_ASSERT_EQUALS(1, 1);
    }

};

#endif /*TEST_ELMAH_HPP_*/
