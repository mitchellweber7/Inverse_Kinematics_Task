#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "math.h"

typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{

  // Information that tells the caller the numeric type (eg. double) and size (input / output dim)
  typedef _Scalar Scalar;
  enum { // Required by numerical differentiation module
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
  };

  // Tell the caller the matrix sizes associated with the input, output, and jacobian
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  // Local copy of the number of inputs
  int m_inputs, m_values;

  // Two constructors:
  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  // Get methods for users to determine function input and output dimensions
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

/**************************************************
 * A function to compute the forward kinematics of
 * a planar 3-link robot.
 *************************************************/
trafo2d_t forward_kinematics(const vector_t& q ) {
    // check that the joint angle vector has the correct size
    assert( q.size() == 3 );

    // define a constant offset between two joints
    trafo2d_t link_offset = trafo2d_t::Identity();
    link_offset.translation()(1) = 1.;

    // define the start pose
    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        // add the rotation contributed by this joint
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        // add the link offset to this position
        trafo = trafo * link_offset;
    }
    return trafo;
}

vector_t inverse_kinematics(const vector_t& q_start, const trafo2d_t& goal ) {

    vector_t q = q_start;
    std::cout << "Init: " << q.transpose() << std::endl;
    std::cout << "Goal: " << goal.translation().transpose() << std::endl;

    IkFunctor functor(goal.translation());
    Eigen::NumericalDiff<IkFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<IkFunctor>, double> lm(numDiff);
    lm.parameters.maxfev = 200;
    lm.parameters.xtol = 1e-3;
    int ret = lm.minimize(q);

    q(0) -= 2 * M_PI * std::floor((q(0) + M_PI) / (2 * M_PI));
    q(1) -= 2 * M_PI * std::floor((q(1) + M_PI) / (2 * M_PI));
    q(2) -= 2 * M_PI * std::floor((q(2) + M_PI) / (2 * M_PI));

    return q;
}



/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */
int main(){

    /** initial conditions and desired goal */
    vector_t q_start(3);
    q_start.setConstant(-0.1);
    // q_start << -0.1, -M_PI / 6, -M_PI / 3;

    
    trafo2d_t goal = trafo2d_t::Identity();
    // goal.translation()(0) = 1.;

    goal.translation()(0) = -0.1;
    goal.translation()(1) = 0.9;

    /** find the joint angles \a result to reach the end-effector position \a goal */
    vector_t result = inverse_kinematics(q_start,goal);
    std::cout << "Result: " << result.transpose() << std::endl;

    auto T_goal = forward_kinematics(result);
    std::cout << "Forward: " << T_goal.translation().transpose() << std::endl;

    

    return 0;
}
