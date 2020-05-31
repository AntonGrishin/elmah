#ifndef ELMAH_HPP_
#define ELMAH_HPP_

#include <string>
#include "MonodomainProblem.hpp"
#include "FoxModel2002BackwardEuler.hpp"
#include "GenericMeshReader.hpp"
#include "SimpleStimulus.hpp"
#include "DistributedTetrahedralMesh.hpp"


class HeartCellFactory : public AbstractCardiacCellFactory<3> // <3> here
{
private:
    boost::shared_ptr<SimpleStimulus> mpStimulus;

public:
    HeartCellFactory()
        : AbstractCardiacCellFactory<3>(), // <3> here as well!
          mpStimulus(new SimpleStimulus(-80000.0, 2))
    {
    }

    AbstractCardiacCell* CreateCardiacCellForTissueNode(Node<3>* pNode)
    {
        double z = pNode->rGetLocation()[2];

        if (z <= 0.05)
        {
            return new CellFoxModel2002FromCellMLBackwardEuler(mpSolver, mpStimulus);
        }
        else
        {
            return new CellFoxModel2002FromCellMLBackwardEuler(mpSolver, mpZeroStimulus);
        }
    }
};


class ElmahCore 
{
public:

    ElmahCore ()
    {
        HeartConfig::Instance()->SetOutputDirectory("elmah");
        HeartConfig::Instance()->SetOutputFilenamePrefix("results");
        HeartConfig::Instance()->SetVisualizeWithVtk(true);
    }

    ~ElmahCore()
    {
        
    }
    void SetMeshFileName(std::string path)
    {
        HeartConfig::Instance()->SetMeshFileName(path,
                                                 cp::media_type::Axisymmetric);
    }

    void SetDuration(double duration)
    {
        HeartConfig::Instance()->SetSimulationDuration(duration); //ms
    }

    void SetIntracellularConductivities(c_vector<double, 3> cond)
    {
        HeartConfig::Instance()->SetIntracellularConductivities(cond);
    }
    void SetOdePdeAndPrintingTimeSteps(double ode, double pde, double printing)
    {
        HeartConfig::Instance()->SetOdePdeAndPrintingTimeSteps(ode, pde, printing);
    }
};


#endif /*ELMAH_HPP_*/
