#ifndef ELMAH_CELL_FACTORY
#define ELMAH_CELL_FACTORY

#include "chaste_models.hpp" 
#include "MonodomainProblem.hpp"
#include "BidomainProblem.hpp"
#include "FoxModel2002BackwardEuler.hpp"
#include "LuoRudy1991BackwardEuler.hpp"
#include "LuoRudy1991.hpp"
#include "LuoRudy1991Cvode.hpp"
#include "GenericMeshReader.hpp"
#include "SimpleStimulus.hpp"
#include "DistributedTetrahedralMesh.hpp"
#include "RungeKutta4IvpOdeSolver.hpp"
#include "BackwardEulerIvpOdeSolver.hpp"
#include "RungeKutta2IvpOdeSolver.hpp"
#include "elmah_defs.hpp"

AbstractCardiacCell* CardiacCellFactory(const CellMlModel& modelName, boost::shared_ptr<AbstractIvpOdeSolver> solver, boost::shared_ptr<AbstractStimulusFunction> stim)
{
    switch(modelName)
    {
        case CellMlModel::DiFrancescoNoble1985:
            return new CellDiFrancescoNoble1985FromCellML(solver, stim);
        case CellMlModel::FaberRudy2000:
            return new CellFaberRudy2000FromCellML(solver, stim);
        case CellMlModel::FoxModel2002:
            return new CellFoxModel2002FromCellML(solver, stim);
        case CellMlModel::HodgkinHuxley1952:
            return new CellHodgkinHuxley1952FromCellML(solver, stim);
        case CellMlModel::LuoRudy1991:
            return new CellLuoRudy1991FromCellML(solver, stim);
        case CellMlModel::Mahajan2008:
            return new CellMahajan2008FromCellML(solver, stim);
        case CellMlModel::Maleckar2008:
            return new CellMaleckar2008FromCellML(solver, stim);
        case CellMlModel::NobleVargheseKohlNoble1998a:
            return new CellNobleVargheseKohlNoble1998aFromCellML(solver, stim);
        case CellMlModel::Shannon2004:
            return new CellShannon2004FromCellML(solver, stim);
        case CellMlModel::TenTusscher2006Epi:
            return new CellTenTusscher2006EpiFromCellML(solver, stim);

        case CellMlModel::DiFrancescoNoble1985BackwardEuler:
            return new CellDiFrancescoNoble1985FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::FaberRudy2000BackwardEuler:
            return new CellFaberRudy2000FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::FoxModel2002BackwardEuler:
            return new CellFoxModel2002FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::HodgkinHuxley1952BackwardEuler:
            return new CellHodgkinHuxley1952FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::LuoRudy1991BackwardEuler:
            return new CellLuoRudy1991FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::Mahajan2008BackwardEuler:
            return new CellMahajan2008FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::Maleckar2008BackwardEuler:
            return new CellMaleckar2008FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::NobleVargheseKohlNoble1998aBackwardEuler:
            return new CellNobleVargheseKohlNoble1998aFromCellMLBackwardEuler(solver, stim);
        case CellMlModel::Shannon2004BackwardEuler:
            return new CellShannon2004FromCellMLBackwardEuler(solver, stim);
        case CellMlModel::TenTusscher2006EpiBackwardEuler:
            return new CellTenTusscher2006EpiFromCellMLBackwardEuler(solver, stim);
        default:
            throw("Unimplemented");
    }
}

AbstractCvodeCell* CardiacCvodeCellFactory(const CellMlModel& modelName, boost::shared_ptr<AbstractIvpOdeSolver> solver, boost::shared_ptr<AbstractStimulusFunction> stim)
{
    switch(modelName)
    {
        case CellMlModel::DiFrancescoNoble1985Cvode:
            return new CellDiFrancescoNoble1985FromCellMLCvode(solver, stim);
        case CellMlModel::FaberRudy2000Cvode:
            return new CellFaberRudy2000FromCellMLCvode(solver, stim);
        case CellMlModel::FoxModel2002Cvode:
            return new CellFoxModel2002FromCellMLCvode(solver, stim);
        case CellMlModel::HodgkinHuxley1952Cvode:
            return new CellHodgkinHuxley1952FromCellMLCvode(solver, stim);
        case CellMlModel::LuoRudy1991Cvode:
            return new CellLuoRudy1991FromCellMLCvode(solver, stim);
        case CellMlModel::Mahajan2008Cvode:
            return new CellMahajan2008FromCellMLCvode(solver, stim);
        case CellMlModel::Maleckar2008Cvode:
            return new CellMaleckar2008FromCellMLCvode(solver, stim);
        case CellMlModel::NobleVargheseKohlNoble1998aCvode:
            return new CellNobleVargheseKohlNoble1998aFromCellMLCvode(solver, stim);
        case CellMlModel::Shannon2004Cvode:
            return new CellShannon2004FromCellMLCvode(solver, stim);
        case CellMlModel::TenTusscher2006EpiCvode:
            return new CellTenTusscher2006EpiFromCellMLCvode(solver, stim);
        default:
            throw("Unimplemented");
    }
}
class AbstractCellFactory2D: public AbstractCardiacCellFactory<2> 
{
private:
    boost::shared_ptr<SimpleStimulus> mpStimulus;
    CellMlModel m_name;
public:
    AbstractCellFactory2D(const CellMlModel& modelName, boost::shared_ptr<AbstractIvpOdeSolver> solver)
        : AbstractCardiacCellFactory<2>(solver),
          mpStimulus(new SimpleStimulus(-5e5, 0.5)),
          m_name(modelName)
    {
    }

    AbstractCardiacCell* CreateCardiacCellForTissueNode(Node<2>* pNode)
    {
        double x = pNode->rGetLocation()[0];
        double y = pNode->rGetLocation()[1];
        if (x<0.02+1e-6 && y<0.02+1e-6) // ie if x<=0.02 and y<=0.02 (and we are assuming here x,y>=0).
        {
            return CardiacCellFactory(m_name, mpSolver, mpStimulus);
        }
        else
        {
            return CardiacCellFactory(m_name, mpSolver, mpZeroStimulus);
        }
    }
};


class AbstractCellFactory3D: public AbstractCardiacCellFactory<3> 
{
private:
    boost::shared_ptr<SimpleStimulus> mpStimulus;
    CellMlModel m_name;

public:
    AbstractCellFactory3D(const CellMlModel& modelName)
        : AbstractCardiacCellFactory<3>(),
        mpStimulus(new SimpleStimulus(-8e5, 0.0)),
          m_name(modelName)
    {
    }

    AbstractCardiacCell* CreateCardiacCellForTissueNode(Node<3>* pNode)
    {
        double z = pNode->rGetLocation()[2];

        if (z <= 0.05)
        {
            return CardiacCellFactory(m_name, mpSolver, mpStimulus);
        }
        else
        {
            return CardiacCellFactory(m_name, mpSolver, mpZeroStimulus);
        }
    }
};


class AbstractCellFactory3DCVode: public AbstractCardiacCellFactory<3> 
{
private:
    boost::shared_ptr<SimpleStimulus> mpStimulus;
    CellMlModel m_name;

public:
    AbstractCellFactory3DCVode(const CellMlModel& modelName)
        : AbstractCardiacCellFactory<3>(),
          mpStimulus(new SimpleStimulus(-5e5, 0.5)),
          m_name(modelName)
    {
    }

   AbstractCvodeCell* CreateCardiacCellForTissueNode(Node<3>* pNode)
    {
        AbstractCvodeCell* p_cell;
        boost::shared_ptr<AbstractIvpOdeSolver> p_empty_solver;

        double x = pNode->rGetLocation()[0];
        double y = pNode->rGetLocation()[1];
        double z = pNode->rGetLocation()[2];

        if ((x<0.1+1e-6) && (y<0.1+1e-6) && (z<0.1+1e-6))
        {
            p_cell = CardiacCvodeCellFactory(m_name, mpSolver, mpStimulus);
        }
        else
        {
            p_cell = CardiacCvodeCellFactory(m_name, mpSolver, mpZeroStimulus);
        }
        p_cell->SetTolerances(1e-5,1e-7);

        return p_cell;
    }
};


#endif //ELMAH_CELL_FACTORY