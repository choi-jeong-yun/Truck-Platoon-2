import tkinter as tk

def get_stipple(ratio):
    """
    ratio 값에 따라 stipple 패턴을 선택.
    ratio가 높을수록 (값이 클수록) 덜 투명하게 표현.
    """
    if ratio >= 0.75:
        return "gray75"   # 거의 불투명
    elif ratio >= 0.5:
        return "gray50"
    elif ratio >= 0.25:
        return "gray25"
    else:
        return "gray12"   # 매우 투명

class TkinterDashboard:
    def __init__(self, lane_following_node):
        self.lf_node = lane_following_node  # lane_following_node 인스턴스 전달
        self.root = tk.Tk()
        self.root.title("트럭 플래투닝 UI")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)  # 종료 이벤트 처리

        self.speed_bars = {}      # 각 트럭의 속도 바를 위한 Canvas
        self.distance_bars = {}   # 각 트럭의 거리 바를 위한 Canvas
        self.speed_labels = {}
        self.distance_labels = {}

        # 최대값 (임의로 설정, 필요에 따라 조정)
        self.max_speed_kmh = 35  # 예: 최대 100 km/h
        self.max_distance = 15  # 예: 최대 50 m

        # Truck0, Truck1, Truck2에 대한 프레임 생성
        for i in range(3):
            frame = tk.Frame(self.root, padx=10, pady=5)
            frame.pack()

            # 타이틀 라벨
            title_label = tk.Label(frame, text=f"Truck {i}", font=("Arial", 16))
            title_label.grid(row=0, column=0, columnspan=2)

            # 속도 라벨
            speed_label = tk.Label(frame, text="속도: N/A km/h", font=("Arial", 14))
            speed_label.grid(row=1, column=0, sticky="w")
            self.speed_labels[i] = speed_label

            # 속도 바 Canvas (폭 100, 높이 20)
            speed_canvas = tk.Canvas(frame, width=100, height=20, bg="white")
            speed_canvas.grid(row=1, column=1, padx=10)
            self.speed_bars[i] = speed_canvas

            # 거리 라벨
            distance_label = tk.Label(frame, text="거리: N/A m", font=("Arial", 14))
            distance_label.grid(row=2, column=0, sticky="w")
            self.distance_labels[i] = distance_label

            # 거리 바 Canvas (폭 100, 높이 20)
            distance_canvas = tk.Canvas(frame, width=100, height=20, bg="white")
            distance_canvas.grid(row=2, column=1, padx=10)
            self.distance_bars[i] = distance_canvas

    def update_ui(self):
        """lane_following_node의 최신 데이터를 기반으로 라벨 및 바를 갱신"""
        for i in range(3):
            # 현재 속도(m/s)를 가져와 km/h로 변환
            velocity = self.lf_node.current_velocities.get(i, 0.0)
            kmh = velocity * 3.6
            sensor = self.lf_node.distance_sensor.get(i, None)
            if sensor:
                d = sensor.get_distance()
                distance_text = f"{d:.2f} m" if d is not None else "N/A"
                distance_value = d if d is not None else 0.0
            else:
                distance_text = "N/A"
                distance_value = 0.0

            # 라벨 업데이트
            self.speed_labels[i].config(text=f"속도: {kmh:.2f} km/h")
            self.distance_labels[i].config(text=f"거리: {distance_text}")

            # 속도 바 업데이트
            speed_canvas = self.speed_bars[i]
            speed_canvas.delete("all")
            bar_width = 100
            # 속도 바 채우기 비율 계산 (0~1)
            ratio_speed = min(max(kmh / self.max_speed_kmh, 0), 1)
            fill_width_speed = ratio_speed * bar_width
            stipple_speed = get_stipple(ratio_speed)
            speed_canvas.create_rectangle(0, 0, fill_width_speed, 20, fill="red", stipple=stipple_speed)

            # 거리 바 업데이트 (리더 차량은 고정된 색상, 후행 차량은 값에 따라 stipple 적용)
            distance_canvas = self.distance_bars[i]
            distance_canvas.delete("all")
            ratio_distance = min(max(distance_value / self.max_distance, 0), 1)
            fill_width_distance = ratio_distance * bar_width
            stipple_distance = get_stipple(ratio_distance)
            distance_canvas.create_rectangle(0, 0, fill_width_distance, 20, fill="blue", stipple=stipple_distance)

        # 100ms 후에 다시 update_ui 호출 (반복 업데이트)
        self.root.after(100, self.update_ui)

    def on_closing(self):
        """창을 닫을 때 호출되는 함수로, ROS 노드를 종료하고 tkinter 창을 닫습니다."""
        try:
            # ROS 노드를 정상 종료
            self.lf_node.destroy_node()
            import rclpy
            rclpy.shutdown()
        except Exception as e:
            print("ROS shutdown 중 오류 발생:", e)
        self.root.destroy()

    def run(self):
        self.update_ui()
        self.root.mainloop()
