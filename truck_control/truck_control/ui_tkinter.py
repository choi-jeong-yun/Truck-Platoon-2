import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import signal
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.patches as mpatches
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Circle

# ROS2 노드: 트럭 속도(실제 km/h 값) 구독
class TruckSpeedSubscriber(Node):
    def __init__(self, ui):
        super().__init__('truck_speed_subscriber')
        self.ui = ui
        # 트럭 3대의 속도를 저장 (단위: km/h)
        self.truck_speeds = {0: 0.0, 1: 0.0, 2: 0.0}

        # 각 트럭 토픽 구독 (속도 데이터가 km/h 단위로 들어온다고 가정)
        self.create_subscription(Float32, '/truck0/velocity', self.truck0_callback, 10)
        self.create_subscription(Float32, '/truck1/velocity', self.truck1_callback, 10)
        self.create_subscription(Float32, '/truck2/velocity', self.truck2_callback, 10)

        # 100ms마다 UI 업데이트
        self.create_timer(0.1, self.update_ui)

    def truck0_callback(self, msg):
        self.truck_speeds[0] = msg.data *3.6

    def truck1_callback(self, msg):
        self.truck_speeds[1] = msg.data*3.6

    def truck2_callback(self, msg):
        self.truck_speeds[2] = msg.data*3.6

    def update_ui(self):
        self.ui.update_speed_gauges(self.truck_speeds)

# Tkinter + Matplotlib 기반 속도 게이지 UI
class SpeedometerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Truck Speedometer")
        # 최대 속도 (km/h) 설정 – 이 값에 따라 게이지의 회전 각도가 결정됨
        self.max_speed = 80.0
        # 바늘 길이 (게이지 내에서 고정 길이)
        self.needle_length = 50

        # 3개의 Polar 서브플롯 생성 (각각 한 트럭용)
        self.fig, self.axs = plt.subplots(1, 3, subplot_kw={'projection': 'polar'}, figsize=(10, 5))
        self.fig.subplots_adjust(wspace=0.4)

        self.speed_lines = []   # 각 게이지의 바늘(Line2D 객체)
        self.value_texts = []   # 게이지 내부에 표시할 속도 텍스트
        for idx, ax in enumerate(self.axs):
            # 게이지의 반지름 범위를 바늘 길이에 맞춤
            ax.set_ylim(0, self.needle_length)
            ax.set_xticklabels([])   # X축 눈금 제거
            ax.set_yticklabels([])   # Y축 눈금 제거
            ax.set_title(f"Truck {idx}", fontsize=17, fontweight='bold', color='navy')
            ax.set_rticks([]) #원형 추가선 제거

            #시계 방향 조정
            ax.set_theta_direction(-1)
            ax.set_theta_offset(-3*np.pi/4)

            tick_angles = [0, 33.75, 67.5, 101.25, 135, 168.75, 202.5, 236.25, 270]

            tick_labels = [f"{(angle/180)*self.max_speed:.0f} " for angle in tick_angles]
            ax.set_thetagrids(tick_angles, labels=tick_labels)

            # 초기 바늘: 각도 0, 고정 길이로 설정 (즉, 오른쪽 방향)
            speed_line, = ax.plot([0, 0], [0, self.needle_length], color='r', linewidth=4)
            self.speed_lines.append(speed_line)

                        # 기존 기본 눈금 제거
            ax.set_rticks([])  # 반지름 눈금 제거
            ax.grid(False)    # 그리드 제거

            # 최대 속도, 바늘 길이, 틱 길이 설정
            # (이미 self.max_speed와 self.needle_length가 정의되어 있다고 가정)
            tick_length_small = 5   # 작은 틱 길이 (예: 5 단위)
            tick_length_big = 10    # 큰 틱 길이 (예: 10 단위)
            label_offset = 10       # 라벨을 바늘 외부로 배치할 오프셋

            # 작은 틱과 큰 틱의 값 계산 (예: 10km/h 간격 작은 틱, 20km/h 간격 큰 틱)
            small_ticks = np.arange(0, self.max_speed + 1, 5)
            big_ticks   = np.arange(0, self.max_speed + 1, 10)

            # 작은 틱(각도) 계산
            small_angles = np.deg2rad((small_ticks / self.max_speed) * 270) 
            big_angles   = np.deg2rad((big_ticks / self.max_speed) * 270)

            # 작은 틱 그리기 (모든 작은 틱)
            for tick, angle in zip(small_ticks, small_angles):
                # 큰 틱이면 긴 선, 아니면 짧은 선
                if tick in big_ticks:
                    inner_radius = self.needle_length - tick_length_big
                else:
                    inner_radius = self.needle_length - tick_length_small
                # Polar 좌표에서 선 그리기: [angle, angle]에 대해 반지름 [inner_radius, self.needle_length]
                ax.plot([angle, angle], [inner_radius, self.needle_length],
                        color='darkblue', lw=2)   


        # Matplotlib Figure를 Tkinter Canvas에 삽입
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        # 게이지 아래에 각 트럭의 속도를 출력할 Tkinter Label 생성
        self.label_frame = tk.Frame(self.root)
        self.label_frame.pack(pady=10)
        self.speed_labels = []
        for i in range(3):
            label = tk.Label(self.label_frame, text="Truck {}: 0.0 km/h".format(i), font=("Helvetica", 10))
            label.pack(side=tk.LEFT, padx=20)
            self.speed_labels.append(label)

    # 각 게이지 업데이트: 바늘의 회전각과 내부 텍스트, 그리고 하단 레이블 업데이트
    def update_speed_gauges(self, truck_speeds):
        for i in range(3):
            # 속도 (km/h) – ROS2로부터 받은 값
            speed = truck_speeds.get(i, 0.0)
            # 최대 속도 대비 비율을 구해서, 0 ~ 180도(π rad) 범위로 매핑
            angle = np.deg2rad((speed / self.max_speed) * 180)
            # 바늘은 항상 고정된 길이로 그리되, 회전각만 변경
            self.speed_lines[i].set_xdata([0, angle])
            self.speed_lines[i].set_ydata([0, self.needle_length])
            # 게이지 아래 레이블 업데이트
            self.speed_labels[i].config(text=f"Truck {i}: {speed:.1f} km/h")
        self.canvas.draw()

def main():
    rclpy.init()
    root = tk.Tk()
    ui = SpeedometerUI(root)
    node = TruckSpeedSubscriber(ui)

    # Tkinter 창 종료 시 호출될 함수
    def on_closing():
        node.get_logger().info("종료 중...")
        root.destroy()           # Tkinter 루프 종료
        node.destroy_node()      # ROS2 노드 종료
        rclpy.shutdown()         # ROS2 종료

    root.protocol("WM_DELETE_WINDOW", on_closing)

    # SIGINT (Ctrl+C) 이벤트 처리: Tkinter 루프 종료
    def signal_handler(sig, frame):
        on_closing()
    signal.signal(signal.SIGINT, signal_handler)

    # Tkinter와 ROS2를 함께 실행하는 루프 함수
    def ros_spin():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            root.after(1, ros_spin)

    root.after(1, ros_spin)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_closing()

if __name__ == '__main__':
    main()
