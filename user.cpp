#include <iostream>
#include <iomanip>
#include "ar_ik_api.h"

int main(){
    ar_ik_api ik_solver;

    double x = 1;
    double y = 0;
    std::cout << "Input x and y goal" << std::endl;
    std::cin >> x;
    std::cin >> y;

    trafo2d_t goal; 
    goal.translation() << x, y;
    vector_t q_start(3);
    q_start.setConstant(-1);
    auto q = ik_solver.inverse_kinematics(q_start, goal);
    std::cout << "Angles: " << q.transpose() << std::endl;

    auto pos = ik_solver.forward_kinematics(q);
    std::cout << "Check: " << pos.translation().transpose() << std::endl;

    double t = ik_solver.time_taken();
    std::cout << "Time taken: " << std::setprecision(20) << t << std::endl;

    ik_solver.stats();
    ik_solver.graph_errors();

    return 0;
}