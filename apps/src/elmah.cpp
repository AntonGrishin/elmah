#include <iostream>
#include <string>

#include "ExecutableSupport.hpp"
#include "Exception.hpp"
#include "elmah_core.hpp"
#include "elmah_core_factory.hpp"
#include "PetscTools.hpp"
#include "PetscException.hpp"
#include "elmah_defs.hpp"
#include "Timer.hpp"

int main(int argc, char *argv[])
{
    // This sets up PETSc and prints out copyright information, etc.
    ExecutableSupport::InitializePetsc(&argc, &argv);

    int exit_code = ExecutableSupport::EXIT_OK;

    // You should put all the main code within a try-catch, to ensure that
    // you clean up PETSc before quitting.
    try
    {
        // if (argc<2)
        // {
        //     ExecutableSupport::PrintError("Usage: ExampleApp arguments ...", true);
        //     exit_code = ExecutableSupport::EXIT_BAD_ARGUMENTS;
        // }
        // else
        // {
        //     for (int i=1; i<argc; i++)
        //     {
        //         if (PetscTools::AmMaster())
        //         {
        //             std::string arg_i(argv[i]);
        //             Hello world(arg_i);
        //             std::cout << "Argument " << i << " is " << world.GetMessage() << std::endl << std::flush;
        //         }
        //     }
        // }

        std::unique_ptr<IElmahCore> elmah(CreateCore(CoreImpl::CHASTE));

        elmah->GenerateMesh("/home/agrishin/dev/Chaste/mesh/test/data/2D_0_to_1mm_800_elements");

        SimConfig2D config;

        config.duration = 100.0;

        config.inCellConductivities  = Create_c_vector(1.75, 0.19);
        config.outCellConductivities = Create_c_vector(6.2, 2.4);
        config.prName = ProblemName::BI;
        config.CMName = CellMlModel::LuoRudy1991;
        config.sName  = SolverName::RK4;
        config.odeTS  = 0.01;
        config.pdeTS  = 0.01;
        config.printingTS = 0.1;

        elmah->StartSimulation(&config);
    }
    catch (const Exception& e)
    {
        ExecutableSupport::PrintError(e.GetMessage());
        exit_code = ExecutableSupport::EXIT_ERROR;
    }

    // End by finalizing PETSc, and returning a suitable exit code.
    // 0 means 'no error'
    ExecutableSupport::FinalizePetsc();
    return exit_code;
}
