import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

class FiveBarMechanism:
    def __init__(self):
        # 连杆参数 (所有长度固定)
        self.l1 = self.l2 = self.l3 = self.l4 = 1
        self.l5 = 0.8
        # 初始位置
        self.x_C = 0.5
        self.y_C = 0.5
        self.phi1 = np.pi/4
        self.phi4 = np.pi/4
        self.dragging = False
        
        # 创建图形界面
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        
        # 连接鼠标事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        
        # 添加重置按钮
        reset_ax = plt.axes([0.8, 0.02, 0.1, 0.04])
        self.reset_button = Button(reset_ax, 'Reset')
        self.reset_button.on_clicked(self.reset)
        
        # 初始更新
        self.inverse_kinematics(self.x_C, self.y_C)
        self.update_plot()

    def get_angles(self, p1, p2):
        """计算从p1到p2的角度(相对于x轴正方向)"""
        return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

    def inverse_kinematics(self, x_C, y_C):
        """计算逆运动学"""
        def equations(vars):
            phi1, phi4 = vars
            
            # 计算B点位置（从左电机）
            x_B = -self.l5/2 + self.l1 * np.cos(phi1)
            y_B = self.l1 * np.sin(phi1)
            
            # 计算D点位置（从右电机）
            x_D = self.l5/2 + self.l4 * np.cos(phi4)
            y_D = self.l4 * np.sin(phi4)
            
            # 计算各点之间的相对角度
            angle_B = self.get_angles([x_B, y_B], [x_C, y_C])
            angle_D = self.get_angles([x_D, y_D], [x_C, y_C])
            
            # 计算phi2和phi3
            phi2 = (angle_B - phi1 + 2*np.pi) % (2*np.pi)
            phi3 = (angle_D - phi4 + 2*np.pi) % (2*np.pi)
            
            # 添加角度约束的惩罚项
            angle_penalty = 0
            if not (np.pi <= phi2 <= 2*np.pi):  # phi2应该是凹角
                angle_penalty += min(abs(phi2 - np.pi), abs(phi2 - 2*np.pi)) * 10
            if not (0 <= phi3 <= np.pi):  # phi3应该是凸角
                angle_penalty += min(abs(phi3), abs(phi3 - np.pi)) * 10
            
            # 计算B到C和D到C的距离
            BC = np.sqrt((x_C - x_B)**2 + (y_C - y_B)**2)
            DC = np.sqrt((x_C - x_D)**2 + (y_C - y_D)**2)
            
            return [BC - self.l2 + angle_penalty, DC - self.l3 + angle_penalty]

        # 尝试多个初始值，初始值也相应调整
        initial_guesses = [
            [self.phi1, self.phi4],
            [5*np.pi/6, np.pi/6],  # 调整初始猜测以适应新的角度约束
            [3*np.pi/4, np.pi/4],
            [2*np.pi/3, np.pi/3],
            [np.pi/2, np.pi/2],
        ]

        best_error = float('inf')
        best_solution = None

        for guess in initial_guesses:
            try:
                solution = fsolve(equations, guess, full_output=True)
                if solution[2] == 1:  # 检查是否收敛
                    phi1, phi4 = solution[0]
                    # 计算B和D点位置
                    x_B = -self.l5/2 + self.l1 * np.cos(phi1)
                    y_B = self.l1 * np.sin(phi1)
                    x_D = self.l5/2 + self.l4 * np.cos(phi4)
                    y_D = self.l4 * np.sin(phi4)
                    
                    # 计算phi2和phi3
                    phi2 = (self.get_angles([x_B, y_B], [x_C, y_C]) - phi1 + 2*np.pi) % (2*np.pi)
                    phi3 = (self.get_angles([x_D, y_D], [x_C, y_C]) - phi4 + 2*np.pi) % (2*np.pi)
                    
                    # 检查角度约束
                    if (np.pi <= phi2 <= 2*np.pi) and (0 <= phi3 <= np.pi):
                        error = np.sum(np.abs(equations([phi1, phi4])))
                        if error < best_error:
                            best_error = error
                            best_solution = (phi1, phi4)
            except:
                continue

        if best_solution is not None and best_error < 1e-3:
            self.phi1, self.phi4 = best_solution
            return True
        return False

    def calculate_points(self):
        """计算所有点的位置"""
        # 计算B点位置（从左电机）
        x_B = -self.l5/2 + self.l1 * np.cos(self.phi1)
        y_B = self.l1 * np.sin(self.phi1)
        
        # 计算D点位置（从右电机）
        x_D = self.l5/2 + self.l4 * np.cos(self.phi4)
        y_D = self.l4 * np.sin(self.phi4)
        
        # 计算phi2和phi3
        phi2 = (self.get_angles([x_B, y_B], [self.x_C, self.y_C]) - self.phi1 + 2*np.pi) % (2*np.pi)
        phi3 = (self.get_angles([x_D, y_D], [self.x_C, self.y_C]) - self.phi4 + 2*np.pi) % (2*np.pi)
        
        return x_B, y_B, x_D, y_D, phi2, phi3

    def update_plot(self):
        """更新图形"""
        self.ax.clear()
        
        # 计算各点位置和角度
        x_B, y_B, x_D, y_D, phi2, phi3 = self.calculate_points()
        
        # 绘制底座连接线
        self.ax.plot([-self.l5/2, self.l5/2], [0, 0], 'k-', linewidth=3, label='Base')
        
        # 绘制连杆
        self.ax.plot([-self.l5/2, x_B], [0, y_B], 'b-', linewidth=2, label='Link 1')
        self.ax.plot([x_B, self.x_C], [y_B, self.y_C], 'g-', linewidth=2, label='Link 2')
        self.ax.plot([self.x_C, x_D], [self.y_C, y_D], 'r-', linewidth=2, label='Link 3')
        self.ax.plot([self.l5/2, x_D], [0, y_D], 'y-', linewidth=2, label='Link 4')
        
        # 绘制关节点
        self.ax.plot(-self.l5/2, 0, 'ks', markersize=8, label='Motor 1')
        self.ax.plot(self.l5/2, 0, 'ks', markersize=8, label='Motor 2')
        self.ax.plot(x_B, y_B, 'ko', markersize=6)
        self.ax.plot(self.x_C, self.y_C, 'ko', markersize=8, label='End Point')
        self.ax.plot(x_D, y_D, 'ko', markersize=6)
        
        # 显示所有角度
        self.ax.text(0.02, 0.98, 
                    f'φ1 = {np.degrees(self.phi1):.1f}°\n'
                    f'φ2 = {np.degrees(phi2):.1f}°\n'
                    f'φ3 = {np.degrees(phi3):.1f}°\n'
                    f'φ4 = {np.degrees(self.phi4):.1f}°\n'
                    f'x = {self.x_C:.2f}, y = {self.y_C:.2f}',
                    transform=self.ax.transAxes, 
                    verticalalignment='top')
        
        # 设置图形属性
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.legend(loc='upper right')
        self.ax.set_title('Five-Bar Mechanism')
        
        plt.draw()

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        if np.sqrt((event.xdata - self.x_C)**2 + (event.ydata - self.y_C)**2) < 0.1:
            self.dragging = True

    def on_release(self, event):
        self.dragging = False

    def on_motion(self, event):
        if self.dragging and event.inaxes == self.ax:
            new_x = event.xdata
            new_y = event.ydata
            if self.inverse_kinematics(new_x, new_y):
                self.x_C = new_x
                self.y_C = new_y
                self.update_plot()

    def reset(self, event):
        self.x_C = 0.5
        self.y_C = 0.5
        self.inverse_kinematics(self.x_C, self.y_C)
        self.update_plot()

# 运行程序
if __name__ == "__main__":
    mechanism = FiveBarMechanism()
    plt.show()