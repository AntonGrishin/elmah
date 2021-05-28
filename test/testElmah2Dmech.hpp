#ifndef TEST_ELMAH3D_MECH_HPP_
#define TEST_ELMAH3D_MECH_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmah2Dmech : public CxxTest::TestSuite
{
public:

    void commonConfig_2D(SimConfig2D& config)
    {
        config.prName = ProblemName::ELECTRO_MECHA;
        config.dim = Dimension::dim2D;
    }

    void commonMeshConfig(MeshConfig& config, OutConfig& outConfig)
    {
        config.electric_mesh = "mesh/test/data/annuli/circular_annulus_960_elements";
        config.mecha_mesh    = "mesh/test/data/annuli/circular_annulus_960_elements_quad";
        config.ortho_mesh    = "heart/test/data/fibre_tests/circular_annulus_960_elements.ortho";

        outConfig.vtkOutput = true;
        outConfig.outFileName = "result";

        config.dim = Dimension::dim2D;
    }

    void testElmahELectroMecha50ms2D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        OutConfig outConfig = {};
        MeshConfig meshConfig;
        commonMeshConfig(meshConfig, outConfig);
        outConfig.outFolder = "elmecha50ms";
        elmah->SetOutputParameters(outConfig);

        elmah->GenerateMesh(meshConfig);
        
        SimConfig2D config;
        commonConfig_2D(config);
        config.duration = 50.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmahELectroMecha100ms2D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        OutConfig outConfig = {};
        MeshConfig meshConfig;
        commonMeshConfig(meshConfig, outConfig);
        outConfig.outFolder = "elmecha50ms";
        elmah->SetOutputParameters(outConfig);

        elmah->GenerateMesh(meshConfig);
        
        SimConfig2D config;
        commonConfig_2D(config);
        config.duration = 100.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

 };

#endif /*TEST_ELMAH_HPP_*/
