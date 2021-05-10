#ifndef TEST_ELMAH3D_MECH_HPP_
#define TEST_ELMAH3D_MECH_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmah3D : public CxxTest::TestSuite
{
public:

    void commonConfig_2D(SimConfig2D& config)
    {
        config.prName = ProblemName::ELECTRO_MECHA;
        config.dim = Dimension::dim2D;
    }


    void testElmahELectroMecha50ms2D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        SimConfig2D config;
        commonConfig_2D(config);
        config.duration = 50.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmahELectroMecha100ms2D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        SimConfig2D config;
        commonConfig_2D(config);
        config.duration = 100.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

 };

#endif /*TEST_ELMAH_HPP_*/
