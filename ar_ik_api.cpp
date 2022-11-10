#include "ar_ik_api.h"
#include <iostream>
#include <vector>
#include <ctime>
// #include "matplotlibcpp.h"

// namespace plt = matplotlibcpp;

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum { 
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
  };

  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

};

struct IkFunctor : Functor<double>
{
    IkFunctor(vector_t goal): Functor<double>(3, 3) {this->goal = goal;}
    int operator()(vector_t& q, vector_t& fvec) const {

        trafo2d_t link_offset = trafo2d_t::Identity();
        link_offset.translation()(1) = 1.;

        trafo2d_t trafo = trafo2d_t::Identity();

        for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
            trafo *= Eigen::Rotation2D<double>(q(joint_idx));
            trafo = trafo * link_offset;
        }


        fvec(0) = (trafo.translation() - goal)(0);
        fvec(1) = (trafo.translation() - goal)(1);
        fvec(2) = 0;

        return 0;
    }
    vector_t goal;
};


trafo2d_t ar_ik_api::forward_kinematics(const vector_t& q) {
    assert( q.size() == 3 );

    trafo2d_t link_offset = trafo2d_t::Identity();
    link_offset.translation()(1) = 1.;

    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        trafo = trafo * link_offset;
    }
    return trafo;
}

vector_t ar_ik_api::inverse_kinematics(const vector_t& q_start, const trafo2d_t& goal) {
    std::time_t start, end;
    std::time(&start);
    std::ios_base::sync_with_stdio(false);

    vector_t q = q_start;
    IkFunctor functor(goal.translation());
    Eigen::NumericalDiff<IkFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<IkFunctor>, double> lm(numDiff);
    lm.parameters.maxfev = 200;
    lm.parameters.xtol = 1e-3;

    double error = 1.;
    int n = 0;
    lm.minimizeInit(q);
    error_vec.clear();
    while (error > 1e-3 && n++ < 200) {
        int ret = lm.minimizeOneStep(q);
        error = abs(lm.fvec(0)) + abs(lm.fvec(1));
        error_vec.push_back(error);
    }

    q(0) -= 2 * M_PI * std::floor((q(0) + M_PI) / (2 * M_PI));
    q(1) -= 2 * M_PI * std::floor((q(1) + M_PI) / (2 * M_PI));
    q(2) -= 2 * M_PI * std::floor((q(2) + M_PI) / (2 * M_PI));

    std::time(&end);
    solve_time = double(end - start);

    return q;
}

void ar_ik_api::stats() {
    std::cout << "---------------------------------\nStats\n---------------------------------" << std::endl;
    std::cout << "Number of iterations: " << error_vec.size() << std::endl;

    std::cout << "Error vector:" << std::endl;
    std::vector<int> x;
    for (auto err : error_vec)
        std::cout << err << std::endl;
}

void ar_ik_api::graph_errors() {
    // std::vector<int> x;
    // for (int i = 0; i<error_vec.size(); i++)
    //     x.push_back(i + 1);

    // plt::scatter(x, error_vec, 30);
    // plt::title("Errors vs Iteration");
    // plt::xlabel("iteration");
    // plt::ylabel("norm (rad)");
    // plt::show();
}

double ar_ik_api::time_taken() {
    return solve_time;
}
