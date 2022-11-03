#include <iostream>
#include "ar_ik_api.h"

int main(){
    ar_ik_api ik_solver;

    trafo2d_t goal; 
    goal.translation() << 1, 0;
    vector_t q_start(3);
    q_start.setConstant(-1);
    auto q = ik_solver.inverse_kinematics(q_start, goal);
    std::cout << "Angles: " << q.transpose() << std::endl;

    auto pos = ik_solver.forward_kinematics(q);
    std::cout << "Check: " << pos.translation().transpose() << std::endl;

    ik_solver.stats();

    return 0;
}