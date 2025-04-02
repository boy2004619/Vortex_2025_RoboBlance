% 假设我们有一组数据点
x = [1, 2, 3, 4, 5];
y = [2.2, 2.8, 3.6, 4.5, 5.1];

% 选择多项式的阶数，例如3
degree = 3;

% 使用polyfit函数进行多项式拟合
p = polyfit(x, y, degree);

% 使用polyval函数计算拟合多项式的值
y_fit = polyval(p, x);

% 绘制原始数据点
plot(x, y, 'o');
hold on; % 保持当前图形，以便在同一图形上绘制拟合曲线

% 绘制拟合曲线
plot(x, y_fit, '-r');

% 添加图例和标题
legend('原始数据', '拟合曲线');
title('多项式拟合示例');
xlabel('x');
ylabel('y');
hold off; % 释放图形