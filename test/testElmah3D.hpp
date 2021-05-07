#ifndef TEST_ELMAH3D_HPP_
#define TEST_ELMAH3D_HPP_

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
        config.odeTS  = 0.02;
        config.pdeTS  = 0.1;
        config.printingTS = 0.2;
        config.dim = Dimension::dim3D;
    }


    void testElmah50msPerfMonoBE_3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
                                                 cp::media_type::Axisymmetric);

        SimConfig3D config;
        commonConfig_3D(config);
        config.prName = ProblemName::MONO;
        config.sName  = SolverName::BackwardEuler;
        config.duration = 50.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmah100msPerfMonoBE_3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
                                                 cp::media_type::Axisymmetric);

        SimConfig3D config;
        commonConfig_3D(config);
        config.prName = ProblemName::MONO;
        config.sName  = SolverName::BackwardEuler;
        config.duration = 100.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmah50msPerfBidomainBE_3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
                                                 cp::media_type::Axisymmetric);

        SimConfig3D config;
        commonConfig_3D(config);
        config.prName = ProblemName::BI;
        config.sName  = SolverName::BackwardEuler;
        config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
        config.duration = 50.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }

    void testElmah100msPerfBidomainBE_3D()
    {
        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
        elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
                                                 cp::media_type::Axisymmetric);

        SimConfig3D config;
        commonConfig_3D(config);
        config.prName = ProblemName::BI;
        config.sName  = SolverName::BackwardEuler;
        config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
        config.duration = 100.0;

        elmah->StartSimulation(&config);

        TS_ASSERT_EQUALS(1, 1);
    }


    // void testElmah100msPerfBidomainCVode_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::BI;
    //     config.sName  = SolverName::Cvode;
    //     config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
    //     config.duration = 100.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }

    // void testElmah100msPerfBidomainCVodeOpt_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::BI;
    //     config.sName  = SolverName::CvodeOpt;
    //     config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
    //     config.duration = 100.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }




    // void testElmah200msPerfBidomainCVode_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::BI;
    //     config.sName  = SolverName::Cvode;
    //     config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
    //     config.duration = 200.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }

    // void testElmah200msPerfBidomainCVodeOpt_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::BI;
    //     config.sName  = SolverName::CvodeOpt;
    //     config.outCellConductivities = Create_c_vector(6.2, 2.4, 2.4);
    //     config.duration = 200.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }





    // void testElmah100msPerfMONOCVode_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::MONO;
    //     config.sName  = SolverName::Cvode;
    //     config.duration = 200.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }

    // void testElmah100msPerfMONOCVodeOpt_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::MONO;
    //     config.sName  = SolverName::CvodeOpt;
    //     config.duration = 100.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }






    // void testElmah200msPerfMONOCVode_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::MONO;
    //     config.sName  = SolverName::Cvode;
    //     config.duration = 200.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }

    // void testElmah200msPerfMONOCVodeOpt_3D()
    // {
    //     std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));
    //     elmah->GenerateMesh("apps/texttest/weekly/Propagation3d/OxfordRabbitHeart_482um",
    //                                              cp::media_type::Axisymmetric);

    //     SimConfig3D config;
    //     commonConfig_3D(config);
    //     config.prName = ProblemName::MONO;
    //     config.sName  = SolverName::CvodeOpt;
    //     config.duration = 200.0;

    //     elmah->StartSimulation(&config);

    //     TS_ASSERT_EQUALS(1, 1);
    // }

 };

#endif /*TEST_ELMAH_HPP_*/
