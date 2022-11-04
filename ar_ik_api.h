#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;

class ar_ik_api
{
private:
    trafo2d_t goal;
    std::vector<double> error_vec;
    double solve_time;
public:
    ar_ik_api() {};
    void solve(trafo2d_t);
    trafo2d_t forward_kinematics(const vector_t& q_start);
    vector_t inverse_kinematics(const vector_t& q_start, const trafo2d_t& goal);
    void stats();
    void graph_errors();
    double time_taken();
};


