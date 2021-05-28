#pragma once

#include "MonodomainProblem.hpp"
#include "BidomainProblem.hpp"
#include "FoxModel2002BackwardEuler.hpp"
#include "LuoRudy1991BackwardEuler.hpp"
#include "LuoRudy1991.hpp"
#include "GenericMeshReader.hpp"
#include "SimpleStimulus.hpp"
#include "DistributedTetrahedralMesh.hpp"
#include "RungeKutta4IvpOdeSolver.hpp"
#include "BackwardEulerIvpOdeSolver.hpp"
#include "RungeKutta2IvpOdeSolver.hpp"
#include <string>

enum ProblemName
{
    MONO,
    BI,
    ELECTRO_MECHA
};

enum CoreImpl
{
    CHASTE,
    OPEN_CARP,
};

enum Dimension
{
    dim1D = 1,
    dim2D = 2,
    dim3D = 3
};

enum SolverName
{
    Euler, // 2d
    RK2,   // 2d
    RK4,   // 2d
    Cvode,  // 2d/3d
    CvodeOpt,  // 2d/3d
    BackwardEuler // 3d
};

enum CellMlModel
{
    DiFrancescoNoble1985,
    FaberRudy2000,
    FoxModel2002,
    HodgkinHuxley1952,
    LuoRudy1991,
    Mahajan2008,
    Maleckar2008,
    NobleVargheseKohlNoble1998a,
    Shannon2004,
    TenTusscher2006Epi,
    DiFrancescoNoble1985Cvode,
    FaberRudy2000Cvode,
    FoxModel2002Cvode,
    HodgkinHuxley1952Cvode,
    LuoRudy1991Cvode,
    Mahajan2008Cvode,
    Maleckar2008Cvode,
    NobleVargheseKohlNoble1998aCvode,
    Shannon2004Cvode,
    TenTusscher2006EpiCvode,
    DiFrancescoNoble1985BackwardEuler,
    FaberRudy2000BackwardEuler,
    FoxModel2002BackwardEuler,
    HodgkinHuxley1952BackwardEuler,
    LuoRudy1991BackwardEuler,
    Mahajan2008BackwardEuler,
    Maleckar2008BackwardEuler,
    NobleVargheseKohlNoble1998aBackwardEuler,
    Shannon2004BackwardEuler,
    TenTusscher2006EpiBackwardEuler,
    TOTAL_COUNT
};


class SimConfig
{
public:
    virtual ~SimConfig() = default;
    double                duration = 5.0;
    double                odeTS = 0.01;
    double                pdeTS = 0.01;
    double                printingTS = 0.1;
    SolverName            sName  = SolverName::Euler;
    CellMlModel           CMName = CellMlModel::LuoRudy1991;
    Dimension             dim    = Dimension::dim2D;
    ProblemName           prName = ProblemName::MONO;
};

class SimConfig2D : public SimConfig
{
public:
    c_vector<double, 2> inCellConductivities;
    c_vector<double, 2> outCellConductivities;
};
class SimConfig3D : public SimConfig
{
public:
    c_vector<double, 3> inCellConductivities;
    c_vector<double, 3> outCellConductivities;
};

struct OutConfig
{
    bool vtkOutput = true;
    std::string outFileName = "result";
    std::string outFolder   = "elmah";
};

struct MeshConfig
{
    Dimension             dim    = Dimension::dim2D;
    std::string           electric_mesh;
    std::string           mecha_mesh;
    std::string           ortho_mesh;
    double                step;
    double                width;
    double                heigth;
    double                depth; //for 3D only;
};