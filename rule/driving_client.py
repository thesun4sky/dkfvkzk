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
        self.map_code = 3
        self.front_check_point = 3
        self.check_range = 5
        self.total_area = 7
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
        ideal_total_map = []
        self.area_range = self.half_road_limit/5

        my_area = self.get_area(sensing_info.to_middle)

        if self.tick_count % 3 == 1:
            for i in range(7):
                print("")

        for i in range(self.check_range):
            ideal_area, ideal_map = self.get_ideal_area(sensing_info, my_area, i)
            ideal_total_map.append(ideal_map)
            steering += self.get_steering_to_area(sensing_info, my_area, ideal_area, i)
            throttle += self.get_throttle_to_area(sensing_info, my_area, ideal_area, i)
            brake += self.get_brake_to_area(sensing_info, my_area, ideal_area, i)

        # Moving straight forward
        car_controls.steering = steering
        car_controls.throttle = throttle
        car_controls.brake = brake

        if self.tick_count % 3 == 1:
            for j in range(self.check_range):
                i = self.check_range-j-1
                print(ideal_total_map[i])
            print(sensing_info.lap_progress)
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                              car_controls.brake))


        if sensing_info.lap_progress == 100:
            print("time :", time.time() - self.start)

        if sensing_info.lap_progress != 0:
            self.totalSpeed += sensing_info.speed
            self.tick_count += 1
            if my_area == self.get_ideal_area(sensing_info, my_area, 0):
                self.ideal_count += 1

            # print("accuracy : {}".format(round(self.ideal_count / self.tick_count, 3)))
            # print("avgSpeed : {}", self.totalSpeed / self.tick_count)
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
        if to_middle <= -self.half_road_limit:
            return 0
        elif -self.half_road_limit < to_middle <= -self.half_road_limit+self.area_range:
            return 1
        elif -self.half_road_limit + self.area_range < to_middle < -self.area_range:
            return 2
        elif abs(to_middle) <= self.area_range:
            return 3
        elif self.area_range < to_middle < self.half_road_limit-self.area_range:
            return 4
        elif self.half_road_limit-self.area_range <= to_middle < self.half_road_limit:
            return 5
        elif self.half_road_limit <= to_middle:
            return 6
        return 0

    def get_ideal_area(self, sensing_info, my_area, i):
        pos_weight = 0.3
        curve_weight = 1.2
        # 가운데로 이동하도록 이상점 +
        point_arr = [-1, 1, 2, 3, 2, 1, -1]

        # 현재위치에서 가까울수록 이상점수 +
        self.set_near_my_area_point(my_area, point_arr, pos_weight)

        # 지점에서 전방 30M내에 해당 방향의 커브가 앞에 있을 수록 이상점수 +
        self.set_curve_point(curve_weight, i, point_arr, sensing_info)

        # 맵별 급커브 대응 포인트 +
        self.set_map_specified_point(point_arr, sensing_info)

        # 장애물이 있을시 마이너스 가중치로 차선책 찾기 알고리즘 -
        self.set_obstacles_point(i, my_area, point_arr, sensing_info)

        return point_arr.index(max(point_arr)), self.print_area_value(point_arr)

    def set_near_my_area_point(self, my_area, point_arr, pos_weight):
        point_arr[my_area] += pos_weight * 2
        if my_area < 6:
            point_arr[my_area + 1] += pos_weight
        if my_area > 1:
            point_arr[my_area - 1] += pos_weight

    def set_curve_point(self, curve_weight, i, point_arr, sensing_info):
        front_curve_angles = sum(sensing_info.track_forward_angles[i:i + self.front_check_point])
        if abs(front_curve_angles) > 30:
            direction = 1 if front_curve_angles > 0 else -1
            curve_point = (pow(front_curve_angles, 2) / 2000 + 2.3) * direction
            point_arr[6] += curve_weight * (curve_point if front_curve_angles < 0 else -10)
            point_arr[5] += curve_weight * (curve_point * 0.8)
            point_arr[4] += curve_weight * (curve_point * 0.6)
            point_arr[3] += curve_weight * (curve_point * 0.4)
            point_arr[2] -= curve_weight * (curve_point * 0.6)
            point_arr[1] -= curve_weight * (curve_point * 0.8)
            point_arr[0] -= curve_weight * (curve_point if front_curve_angles > 0 else 10)

    def set_obstacles_point(self, i, my_area, point_arr, sensing_info):
        if len(sensing_info.track_forward_obstacles):
            for obj in sensing_info.track_forward_obstacles:
                obj_dist = int(obj['dist'] / 10)
                if i <= obj_dist <= i + 3 or obj_dist == i - 1:
                    obj_area = self.get_area(obj['to_middle'])
                    point_arr[obj_area] += -900
                    if obj_area >= 1:
                        point_arr[obj_area - 1] += -800
                    if obj_area <= 5:
                        point_arr[obj_area + 1] += -800
                    if obj_dist < 40 and abs(obj['to_middle'] - sensing_info.to_middle) < 2:
                        if obj_area >= 2:
                            point_arr[obj_area - 2] += -700
                        if obj_area <= 4:
                            point_arr[obj_area + 2] += -700
                    if obj_area >= my_area:
                        for j in range(self.total_area - obj_area):
                            point_arr[obj_area + j] += -600
                    else:
                        for j in range(obj_area + 1):
                            point_arr[obj_area - j] += -600

    def set_map_specified_point(self, point_arr, sensing_info):
        if self.map_code == 3:
            if 8.5 < sensing_info.lap_progress < 10.5:
                point_arr[5] = 900
            if 10.5 < sensing_info.lap_progress < 12.5:
                point_arr[1] = 900
            if 28.5 < sensing_info.lap_progress < 30.0:
                point_arr[5] = 900



    def get_steering_to_area(self, sensing_info, my_area, ideal_area, i):
        area_diff = ideal_area - my_area
        area_angle = sensing_info.track_forward_angles[i] - sensing_info.moving_angle - sensing_info.to_middle + area_diff

        steering = float(area_angle * (abs(area_diff) * 0.1 + 0.1) * (self.check_range-i) * 0.005)
        return steering

    def get_throttle_to_area(self, sensing_info, my_area, ideal_area, i):
        throttle = 0
        return throttle

    def get_brake_to_area(self, sensing_info, my_area, ideal_area, i):
        brake = 0
        area_angle = sensing_info.track_forward_angles[i]
        if i >= 3 and abs(area_angle) > 50 and sensing_info.speed > 100:
            brake = 0.15

        return brake

    def print_area(self, area):
        map = ""
        for i in range(7):
            map += "●" if area == i else "○"
        return map

    def print_area_value(self, point_arr):
        map = ""
        for i in range(len(point_arr)):
            left = "   "
            right = " || " if i == 0 or i == 5 else "   "
            point = round(float(point_arr[i]), 1)
            center = str(point) if abs(point_arr[i]) > 10 else " " + str(point)
            center = center if point_arr[i] < 0 else " " + center
            if point_arr[i] == max(point_arr):
                center = "[" + center + "]"
            else:
                center = " " + center + " "
            map += left + center + right
        return map

if __name__ == '__main__':
    client = DrivingClient()
    client.run()