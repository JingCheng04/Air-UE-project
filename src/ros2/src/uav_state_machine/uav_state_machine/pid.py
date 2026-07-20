# Reference: https://blog.csdn.net/weixin_43863487/article/details/124604299

import numpy as np
import matplotlib.pyplot as plt

class PID(object):
    """位置式PID算法实现"""
    # 参考值：pid = PID(10, -5, 0.5, 100, -100, 0.2, 0.1, 0.01)

    def __init__(self, target, cur_val, dt, max, min, p, i, d) -> None:
        self.dt = dt  # 循环时间间隔
        self._max = max  # 最大输出限制，规避过冲
        self._min = min  # 最小输出限制
        self.k_p = p  # 比例系数
        self.k_i = i  # 积分系数
        self.k_d = d  # 微分系数

        self.target = target  # 目标值
        self.cur_val = cur_val  # 算法当前PID位置值，第一次为设定的初始位置
        self._pre_error = 0  # t-1 时刻误差值
        self._integral = 0  # 误差积分值


    def calculate(self):
        """
        计算t时刻PID输出值cur_val
        """
        error = self.target - self.cur_val  # 计算当前误差
        # 比例项
        p_out = self.k_p * error  
        # 积分项
        self._integral += (error * self.dt)
        i_out = self.k_i * self._integral
        # 微分项
        derivative = (error - self._pre_error) / self.dt
        d_out = self.k_d * derivative

        # t 时刻pid输出
        output = p_out + i_out + d_out

        # 限制输出值
        if output > self._max:
            output = self._max
        elif output < self._min:
            output = self._min
        
        self._pre_error = error
        self.cur_val = output
        return self.cur_val


# 作为主函数时绘图方便参数整定
if __name__ == '__main__':
    def fit_and_plot(cur_val:PID , count = 200):
        """
        绘图，使用PID拟合setPoint
        """
        counts = np.arange(count)
        outputs = []

        for i in counts:
            outputs.append(cur_val.calculate())
            print('Count %3d: output: %f' % (i, outputs[-1]))

        print('Done')
        # print(outputs)
        
        plt.figure()
        plt.axhline(cur_val.target, c='red')
        plt.plot(counts, np.array(outputs), 'b.')
        plt.ylim(min(outputs) - 0.1 * min(outputs), max(outputs) + 0.1 * max(outputs))
        plt.plot(outputs)
        plt.show()
    
    test_pid = PID(10, 15, 0.5, 100, -100, 0.1, 0.2, 0.02)
    #target, cur_val, dt, max, min, p, i, d

    fit_and_plot(test_pid)



    
