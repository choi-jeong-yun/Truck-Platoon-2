from std_msgs.msg import Float32

def publish_commands(throttle_publishers, current_velocities, target_velocity, last_steering, emergency_stop=False):
    for truck_id in range(len(throttle_publishers)):
        # Truck 0에 대해 emergency_stop인 경우, 무조건 0 throttle (또한 brake 명령도 함께 보낼 수 있다면)
        if truck_id == 0 and emergency_stop:
            msg = Float32()
            msg.data = -1.0  # throttle 0
            throttle_publishers[truck_id].publish(msg)
            continue

        # 기본적 publish_commands 로직
        throttle_value = 0.8
        if current_velocities[truck_id] > target_velocity:
            throttle_value = 0.0
        if abs(last_steering[truck_id]) > 3:
            throttle_value *= 0.8

        msg = Float32()
        msg.data = throttle_value
        throttle_publishers[truck_id].publish(msg)

if __name__ == '__main__':
    main()