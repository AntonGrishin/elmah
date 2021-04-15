#pragma once

#include "elmah_defs.hpp"
#include "elmah_core.hpp"
#include "elmah_core_chaste.hpp"
#include "CardiacElectroMechanicsProblem.hpp"

IElmahCore* CreateCore(const CoreImpl& reqCoreImpl)
{
    switch(reqCoreImpl)
    {
        case CoreImpl::CHASTE:
            return new ElmahCoreChaste();
        default:
            throw("Unimplemented");
    }
}


boost::shared_ptr<AbstractIvpOdeSolver> CreateSolver(const SolverName& solver)
{
    switch(solver)
    {
        case SolverName::Euler:
            return boost::shared_ptr<AbstractIvpOdeSolver>(new EulerIvpOdeSolver);
        case SolverName::RK2:
            return boost::shared_ptr<AbstractIvpOdeSolver>(new RungeKutta2IvpOdeSolver);
        case SolverName::RK4:
            return boost::shared_ptr<AbstractIvpOdeSolver>(new RungeKutta4IvpOdeSolver);
        default:
            throw("Unimplemented");
    }
}