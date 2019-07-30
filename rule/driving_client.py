from drive_controller import DrivingController
import time


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = False
        self.collision_flag = True
        self.start = time.time()
        self.tick_count = 0
        self.ideal_count = 0
        self.totalSpeed = 0
        self.area_range = 2
        self.front_check_point = 3
        self.check_range = 4
        self.area_weight_array = [0, 0, 0, 0, 0, 0, 0] * self.check_range

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):
        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        if self.is_debug:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))
            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))
            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))
            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            # print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("=========================================================")
        ###########################################################################
        steering = 0
        throttle = 1
        brake = 0

        my_area = self.get_area(sensing_info.to_middle)

        for i in range(self.check_range):
            ideal_area = self.get_ideal_area(sensing_info, my_area, i)
            print("{} : {}".format(i,ideal_area))
            steering += self.get_steering_to_area(sensing_info, my_area, ideal_area, i)
            throttle += self.get_throttle_to_area(sensing_info, my_area, ideal_area, i)
            brake += self.get_brake_to_area(sensing_info, my_area, ideal_area, i)
        print("")
        # Moving straight forward
        car_controls.steering = steering
        car_controls.throttle = throttle
        car_controls.brake = brake

        print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                              car_controls.brake))


        if sensing_info.lap_progress == 100:
            print("time :", time.time() - self.start)

        if sensing_info.lap_progress != 0:
            self.totalSpeed += sensing_info.speed
            self.tick_count += 1
            if my_area == self.get_ideal_area(sensing_info, my_area, 0):
                self.ideal_count += 1

            print("accuracy : {}".format(round(self.ideal_count / self.tick_count, 3)))
            print("avgSpeed : {}", self.totalSpeed / self.tick_count)
        #
        # Editing area ends
        # ==========================================================#
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name

    def get_area(self, to_middle):
        if to_middle < -self.half_road_limit-self.area_range:
            return 0
        elif to_middle < -self.half_road_limit+self.area_range:
            return 1
        elif to_middle < -self.area_range:
            return 2
        elif abs(to_middle) <= self.area_range:
            return 3
        elif self.area_range < to_middle < self.half_road_limit-self.area_range:
            return 4
        elif self.half_road_limit-self.area_range < to_middle < self.half_road_limit+self.area_range:
            return 5
        elif self.half_road_limit+self.area_range < to_middle:
            return 6
        return 0

    def get_ideal_area(self, sensing_info, my_area, i):
        pos_weight = 0.5
        curve_weight = 1.2
        # 가운데로 이동하도록 이상점 +
        point_arr = [1, 1, 2, 3, 2, 1, 1]

        # 현재위치에서 가까울수록 이상점수 +
        point_arr[my_area] += pos_weight * 2
        if my_area < 6:
            point_arr[my_area+1] += pos_weight
        if my_area > 1:
            point_arr[my_area-1] += pos_weight

        # 지점에서 전방 30M내에 해당 방향의 커브가 앞에 있을 수록 이상점수 +
        front_curve_angles = sum(sensing_info.track_forward_angles[i:i+self.front_check_point])

        if abs(front_curve_angles) > 30:
            direction = 1 if front_curve_angles > 0 else -1
            curve_point = (pow(front_curve_angles, 2)/2000 + 2.3) * direction
            point_arr[6] += curve_weight * (curve_point if front_curve_angles < 0 else 0)
            point_arr[5] += curve_weight * (curve_point*0.8)
            point_arr[4] += curve_weight * (curve_point*0.6)
            point_arr[3] += curve_weight * (curve_point*0.4)
            point_arr[2] -= curve_weight * (curve_point*0.6)
            point_arr[1] -= curve_weight * (curve_point*0.8)
            point_arr[0] -= curve_weight * (curve_point if front_curve_angles > 0 else 0)

        # 장애물이 있을시 주변 포인트 0
        if len(sensing_info.track_forward_obstacles):
            for obj in sensing_info.track_forward_obstacles:
                obj_dist = int(obj['dist']/10)
                if obj_dist == i or obj_dist == i-1 or obj_dist == i+1:
                    obj_area = self.get_area(obj['to_middle'])
                    point_arr[obj_area] = -999
                    point_arr[obj_area+1] = -999
                    point_arr[obj_area-1] = -999

        return point_arr.index(max(point_arr))

    def get_steering_to_area(self, sensing_info, my_area, ideal_area, i):
        area_diff = ideal_area - my_area
        area_angle = sensing_info.track_forward_angles[i] - sensing_info.moving_angle

        weight = abs(area_diff) * 0.2 + 0.1
        steering = area_angle/20 * weight
        steering = float(steering * ((self.check_range-i)/10))

        return steering

    def get_throttle_to_area(self, sensing_info, my_area, ideal_area, i):
        throttle = 0
        return throttle

    def get_brake_to_area(self, sensing_info, my_area, ideal_area, i):
        brake = 0
        area_angle = sensing_info.track_forward_angles[i] - sensing_info.moving_angle
        if i >= 3 and abs(area_angle) > 50 and sensing_info.speed > 90:
            brake = 0.3

        return brake

if __name__ == '__main__':
    client = DrivingClient()
    client.run()