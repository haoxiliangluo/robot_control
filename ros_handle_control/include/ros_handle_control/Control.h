namespace Control { // 定义一个命名空间 Control
    class PID { // 定义一个 PID 类
        public:
            double kp, ki, kd; // PID 控制器的比例、积分和微分系数
            double error_out; // 控制器的输出误差
            double last_error; // 上一次的误差
            double integral; // 积分值
            double inte_max; // 积分限制
            double last_diff; // 上一次的微分值
            double PIDposition(double error); // 位置式 PID 控制方法
            void init(); // 初始化方法
            double PIDincrement(double error); // 增量式 PID 控制方法
    };

    // 位置式 PID 控制方法的实现
    double PID::PIDposition(double error) {
        double diff = error - last_error; // 计算当前误差与上一次误差的差值（微分）
        integral += error; // 积分累加当前误差
        // 积分限幅，防止积分项过大
        if (integral > inte_max) {
            integral = inte_max;
        } else if (integral < -inte_max) {
            integral = -inte_max;
        }
        // 计算 PID 控制器的输出
        error_out = kp * error + ki * integral + kd * diff;
        last_error = error; // 更新上一次误差
        return error_out; // 返回控制器输出
    }

    // 初始化方法的实现
    void PID::init() {
        last_error = 0.0; // 初始化上一次误差为 0
        integral = 0.0; // 初始化积分值为 0
        inte_max = 8.0; // 设置积分限制
        kp = 15.0; // 设置比例系数
        ki = 0.1; // 设置积分系数
        kd = 3.0; // 设置微分系数
        error_out = 0.0; // 初始化控制器输出为 0
        last_diff = 0.0; // 初始化上一次微分值为 0
    }

    // 增量式 PID 控制方法的实现
    double PID::PIDincrement(double error) {
        // 计算 PID 控制器的输出增量
        //error_out = kp * (error - last_error) + ki * error + kd * (error - 2 * last_error + last_diff);
        error_out = kp * (error - last_error) + ki * error + kd * (error -  last_error - last_diff);
        last_diff = error - last_error; // 更新上一次微分值
        last_error = error; // 更新上一次误差
        return error_out; // 返回控制器输出
    }
}