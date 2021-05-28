#ifndef TEST_ELMAH3D_MECH_HPP_
#define TEST_ELMAH3D_MECH_HPP_

#include <cxxtest/TestSuite.h>
#include "elmah_core.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "Timer.hpp"
#include "elmah_core_factory.hpp"

class testElmah3Dmech : public CxxTest::TestSuite
{
public:

    void commonConfig_3D(SimConfig3D& config)
    {
        config.prName = ProblemName::ELECTRO_MECHA;
        config.dim = Dimension::dim3D;
    }

    void commonMeshConfig(MeshConfig& config, OutConfig& outConfig)
    {
        config.ortho_mesh    = "5by5by5_fibres.ortho";
        config.step   = 0.01;
        config.width  = 0.1;
        config.depth  = 0.1;
        config.heigth = 0.1;


        outConfig.vtkOutput = true;
        outConfig.outFileName = "result";

        config.dim = Dimension::dim3D;
    }

    void testElmahELectroMecha500ms3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        OutConfig outConfig = {};
        MeshConfig meshConfig;
        commonMeshConfig(meshConfig, outConfig);
        outConfig.outFolder = "part_of_hearth3D";
        elmah->SetOutputParameters(outConfig);

        elmah->GenerateMesh(meshConfig);

        SimConfig3D config;
        commonConfig_3D(config);
        config.duration = 500.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

        void testElmahELectroMecha1000ms3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        OutConfig outConfig = {};
        MeshConfig meshConfig;
        commonMeshConfig(meshConfig, outConfig);
        outConfig.outFolder = "part_of_hearth3D_1000ms";
        elmah->SetOutputParameters(outConfig);

        elmah->GenerateMesh(meshConfig);

        SimConfig3D config;
        commonConfig_3D(config);
        config.duration = 1000.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }


 };

#endif /*TEST_ELMAH_HPP_*/
