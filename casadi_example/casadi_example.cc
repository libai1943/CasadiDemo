#include <casadi/casadi.hpp>
#include <iostream>

using namespace casadi;

// 仅用于演示“带参数的 NLP 字典”写法
void example_with_parameter_nlp() {
    SX p_inf_w_ = SX::sym("p_inf_w_");
    SX iterative_var_ = SX::sym("iterative_var_", 3);
    SX iterative_objective_ = SX::sym("iterative_objective_");

    SX p = SX::vertcat({p_inf_w_});
    SXDict nlp = {{"x", iterative_var_}, {"p", p}, {"f", iterative_objective_}};
}

int main() {
    // CasADi 中的变量声明语法
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX z = MX::sym("z");
    MX w = vertcat(x, y, z);

    // 目标函数: (x-1)^2 + (y-2)^4 + (z-3)^6
    MX f = pow(x - 1, 2) + pow(y - 2, 4) + pow(z - 3, 6);

    // 一般约束构造
    MX g0 = x + z;  // 等式约束: x + z = 4
    MX g1 = x + y;  // 下界不等式约束: x + y ≥ 3
    MX g2 = y + z;  // 上界不等式约束: y + z ≤ 6
    MX G = vertcat(g0, g1, g2);    // 向量化合成 G(w)

    // 一般约束的上下界 g
    DM lbg = -DM::inf(3, 1);
    DM ubg =  DM::inf(3, 1);

    // g0: 等式约束x + z = 4
    lbg(0) = 4;
    ubg(0) = 4;

    // g1: 下界不等式约束x + y ≥ 3
    lbg(1) = 3;        // ubg(1) 仍为 +inf

    // g2: 上届不等式约束y + z ≤ 6
    ubg(2) = 6;        // lbg(2) 仍为 -inf

    // 决策变量本身的区间约束
    DM lbx = -DM::inf(3, 1);
    DM ubx =  DM::inf(3, 1);
    lbx(0) = 0;
    ubx(1) = 10;
    lbx(2) = -9.9;

    // CasADi 中的 NLP 字典
    MXDict nlp;
    nlp["x"] = w;      // 决策变量
    nlp["f"] = f;      // 代价函数
    nlp["g"] = G;      // 约束向量

    // IPOPT 求解参数设置
    Dict opts;
    opts["ipopt.linear_solver"] = "ma27";   // 第三方线性求解器
    opts["ipopt.tol"] = 1e-6;               // 收敛容差
    opts["ipopt.max_iter"] = 500;           // 最大迭代次数
    opts["ipopt.print_level"] = 5;          // 输出信息颗粒度

    // 构造优化求解器实例
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // 初始解
    DM x0 = DM::zeros(3, 1);

    // 组织本次求解所需的数值参数
    DMDict arg;
    arg["x0"]  = x0;    // 初始解向量
    arg["lbx"] = lbx;   // 决策向量下界
    arg["ubx"] = ubx;   // 决策向量上界
    arg["lbg"] = lbg;   // 约束向量下界
    arg["ubg"] = ubg;   // 约束向量上界

    // 执行求解
    DMDict res = solver(arg);

    // 取出最优解向量 w* = [x; y; z]
    DM w_opt = res.at("x");
    double x_opt = static_cast<double>(w_opt(0, 0)); 
    double y_opt = static_cast<double>(w_opt(1, 0)); 
    double z_opt = static_cast<double>(w_opt(2, 0)); 

    // 输出结果
    std::cout << "Optimal cost = " << double(res["f"]) << std::endl;
    std::cout << "Optimal solution: x = " << x_opt
              << ", y = " << y_opt
              << ", z = " << z_opt << std::endl;

    return 0;
}
