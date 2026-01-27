import rclpy
from rclpy.node import Node
import carla
import time
import math

class CarlaSpectatorFollower:
    def __init__(self):
        """CARLA 클라이언트 연결"""
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.target_vehicle = None
        self.previous_location = None
        self.previous_rotation = None

    def find_truck1_vehicle(self):
        """같은 type_id 차량 중 role_name이 'truck1'인 차량 찾기"""
        actors = self.world.get_actors()
        vehicles = [actor for actor in actors if actor.type_id.startswith('vehicle.daf')]
        truck1_vehicles = [v for v in vehicles if v.attributes.get('role_name') == 'truck2']

        if truck1_vehicles:
            self.target_vehicle = truck1_vehicles[0]
            print(f"\n 'truck1' 찾음: ID: {self.target_vehicle.id}, 모델: {self.target_vehicle.type_id}")
        else:
            print("'truck1' 역할 차량을 찾지 못했습니다.")

    def lerp(self, start, end, alpha):
        """선형 보간 (Lerp)"""
        return start + (end - start) * alpha

    def lerp_angle(self, start, end, alpha):
        """회전 각도(Yaw) 선형 보간 (360도 불연속성 문제 해결)"""
        diff = ((end - start + 180) % 360) - 180
        return (start + diff * alpha) % 360

    def get_relative_location(self, vehicle_transform, dx, dy, dz):
        """상대좌표를 월드 좌표로 변환"""
        yaw = math.radians(vehicle_transform.rotation.yaw)
        x = vehicle_transform.location.x + dx * math.cos(yaw) - dy * math.sin(yaw)
        y = vehicle_transform.location.y + dx * math.sin(yaw) + dy * math.cos(yaw)
        z = vehicle_transform.location.z + dz
        return carla.Location(x=x, y=y, z=z)

    def follow_vehicle(self):
        """Spectator 시점을 truck1 차량에 부드럽게 고정 (동기화)"""
        spectator = self.world.get_spectator()

        if not self.target_vehicle:
            print("대상 차량이 없습니다. 프로그램을 종료합니다.")
            return


        self.previous_location = self.target_vehicle.get_transform().location
        self.previous_rotation = self.target_vehicle.get_transform().rotation


        try:
            print("🎥 Spectator 시점을 'truck1' 차량에 부드럽게 고정 중 (탑다운, 90도 회전, 원거리)...")
            
            # 스무딩 계수 (필요시 조절)
            # wait_for_tick() 사용 시 루프 속도가 느려지므로 alpha 값을 조금 높여 반응성을 올릴 수 있습니다.
            alpha_position = 0.2  # 기존 0.1에서 0.2로 상향
            alpha_rotation = 0.03  # 기존 0.03에서 0.1로 상향

            while True:
                # [핵심 수정] time.sleep() 대신 wait_for_tick()을 사용하여
                # CARLA 서버 프레임과 동기화합니다. (흔들림 제거)
                self.world.wait_for_tick()

                vehicle_transform = self.target_vehicle.get_transform()
                
                # 목표 위치 (차량 중심에서 위쪽 80m)
                target_location = self.get_relative_location(vehicle_transform, dx=0, dy=0, dz=80)

                # 위치 보간
                smooth_x = self.lerp(self.previous_location.x, target_location.x, alpha_position)
                smooth_y = self.lerp(self.previous_location.y, target_location.y, alpha_position)
                smooth_z = self.lerp(self.previous_location.z, target_location.z, alpha_position)
                
                # 목표 회전 (Yaw) - 차량 진행 방향 + 90도
                target_yaw = vehicle_transform.rotation.yaw + 90
                smooth_yaw = self.lerp_angle(self.previous_rotation.yaw, target_yaw, alpha_rotation)

                # 부드러운 Spectator 변환 적용 (pitch=-90: 수직 아래)
                spectator_transform = carla.Transform(
                    carla.Location(x=smooth_x, y=smooth_y, z=smooth_z),
                    carla.Rotation(pitch=-85, yaw=smooth_yaw, roll=0)
                )
                spectator.set_transform(spectator_transform)

                # 현재 값을 이전 값으로 업데이트
                self.previous_location = carla.Location(x=smooth_x, y=smooth_y, z=smooth_z)
                self.previous_rotation.yaw = smooth_yaw
                
                # time.sleep(0.01) # 삭제됨

        except KeyboardInterrupt:
            print("\n시뮬레이션 중지.")
        finally:
            print("프로그램 종료.")


def main(args=None):
    rclpy.init()
    follower = CarlaSpectatorFollower()
    follower.find_truck1_vehicle()
    follower.follow_vehicle()


if __name__ == '__main__':
    main()