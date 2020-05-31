#ifndef TEST_ELMAH_HPP_
#define TEST_ELMAH_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah.hpp"
#include "PetscSetupAndFinalize.hpp"


class testElmah : public CxxTest::TestSuite
{
public:

    void testElmahMono()
    {
        ElmahCore elmah;
        elmah.SetMeshFileName("/home/agrishin/Chaste/apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um");
        elmah.SetIntracellularConductivities(Create_c_vector(1.75, 0.19, 0.19));
        elmah.SetDuration(5.0);
        elmah.SetOdePdeAndPrintingTimeSteps(0.02, 0.1, 0.2);

        HeartCellFactory cell_factory;
        MonodomainProblem<3> monodomain_problem( &cell_factory );
        monodomain_problem.SetWriteInfo();
        monodomain_problem.Initialise();
        monodomain_problem.Solve();

        TS_ASSERT_EQUALS(1, 1);
    }
};

#endif /*TEST_ELMAH_HPP_*/
