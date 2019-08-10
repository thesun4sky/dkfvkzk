from drive_controller import DrivingController
import numpy
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
        self.area_range = 0
        self.tick_count = 0
        self.ideal_count = 0
        self.totalSpeed = 0
        self.map_code = 0       # 맵 구분 코드
        self.front_check_point = 3  # 이상위치 추출 시 활용할 해당 위치에서의 전방 커브 개수
        self.check_range = 5    # 이상위치 추출 범위
        self.total_movement_value = 0
        self.total_area = 8     # 마지막 area index
        self.total_road = 7     # 도로 위 area 개수
        self.area_weight_array = [0, 0, 0, 0, 0, 0, 0, 0, 0] * self.check_range     # 이상 포인트 배열
        self.wrong_way_flag = False  # 복귀 여부 확인
        self.collision_flag = True  # 계속 막혀있는 상태인지 확인
        self.collision_time = 0  # 복귀로직 tick stack
        self.stopped_back = 0 # 후진/전진시 속도가 10이상 안나오는지 확인
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

        self.area_range = self.half_road_limit/self.total_road  # 맵 등분 크기
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

        ideal_area = self.get_speed_ideal_area(my_area, sensing_info, 0, 3)
        next_ideal_area = self.get_speed_ideal_area(my_area, sensing_info, 2, 4)
        print("now: {}, next: {}".format(ideal_area, next_ideal_area))
        steering = (self.get_ideal_angel(sensing_info, ideal_area, next_ideal_area) - sensing_info.moving_angle) / 90
        # print("moving: {}".format(sensing_info.moving_angle))
        # print("steering: {}".format(steering))

        car_controls.steering = steering
        car_controls.throttle = throttle
        car_controls.brake = brake

        if self.tick_count % 3 == 1:
            for j in range(self.check_range):
                i = self.check_range-j-1
                # print(ideal_total_map[i])
            # print(sensing_info.lap_progress)
            # print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
            #                                                   car_controls.brake))
        self.recovery(self, car_controls, sensing_info)
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

    def get_speed_ideal_area(self, my_area, sensing_info, start, end):
        arr = []
        for i in range(start, end+1):
            ideal, ideal_map = self.get_ideal_area(sensing_info, my_area, i)
            arr.append(ideal)
        avg = int(numpy.mean(arr))

        # 급커브 아웃코스 타기
        if 80 < sensing_info.speed:
            speed_index = int(sensing_info.speed/10) if sensing_info.speed < 120 else 11
            front_curve_angle = numpy.mean(sensing_info.track_forward_angles[0:speed_index-6])
            future_curve_angle = numpy.mean(sensing_info.track_forward_angles[speed_index-3:speed_index-2])
            if abs(front_curve_angle) < 10 and abs(future_curve_angle) > 40:
                print("before avg : {}, front_curve_angle : {}, future_curve_angle : {} ".format(avg, front_curve_angle, future_curve_angle))
                avg += int((sensing_info.speed/20) * -(future_curve_angle/abs(future_curve_angle)))
                print("avg : {}", avg)
                if avg <= 0:
                    avg = 1
                elif avg >= 8:
                    avg = 8

        return avg

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
        elif self.half_road_limit <= to_middle:
            return self.total_area
        else:
            return int(((to_middle+self.half_road_limit)/(self.half_road_limit * 2)) * self.total_road) + 1

    def get_distance_next_waypoint(self, sensing_info):
        return (sensing_info.distance_to_way_points[0]**2 - sensing_info.to_middle**2) ** 0.5

    def get_forward_movement_value(self, angle, speed, moving_forward):
        movement_value = numpy.sin(numpy.radians(90 - abs(angle))) * speed / 3.6 * 0.1
        self.total_movement_value += movement_value if moving_forward else -movement_value
        # print(movement_value)
        # print(self.total_movement_value)
        return movement_value

    def get_ideal_angel(self, sensing_info, ideal_area, next_ideal_area):
        # sin : 전방 값
        # cos : 좌우 값
        ideal_position = (ideal_area - 4) * self.half_road_limit / 9
        next_ideal_position = (next_ideal_area - 4) * self.half_road_limit / 9

        d = 10 - self.get_distance_next_waypoint(sensing_info)
        p = d * 0.1 * (next_ideal_position - ideal_position) + ideal_position
        w = 10
        y = p - sensing_info.to_middle
        x = (w**2 + y**2) ** 0.5
        # print(2 * x * y)
        angle = numpy.degrees(numpy.arccos((x**2 + y**2 - w**2) / (2 * x * abs(y))))

        # print("i: {}, n: {}, p: {}, d: {}, m: {}, y: {}, w: {}, x: {}".format(ideal_position, next_ideal_position, p, d, sensing_info.to_middle, y, w, x))
        # print("angle: {}".format(angle))
        if y < 0:
            angle = -90 + angle
        else:
            angle = 90 - angle
        # print("angle: {}".format(angle))

        return angle

    def get_ideal_area(self, sensing_info, my_area, i):
        pos_weight = 0.7 * (self.check_range - i)
        curve_weight = 1.2
        # 가운데로 이동하도록 이상점 +
        point_arr = [-1, 1, 2, 2, 3, 2, 2, 1, -1]

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
        if my_area < self.total_road:
            point_arr[my_area + 1] += pos_weight
        if my_area > 1:
            point_arr[my_area - 1] += pos_weight

    def set_curve_point(self, curve_weight, i, point_arr, sensing_info):
        front_curve_angles = sum(sensing_info.track_forward_angles[i:i + self.front_check_point])
        if abs(front_curve_angles) > 30:
            direction = 1 if front_curve_angles > 0 else -1
            curve_point = (pow(front_curve_angles, 2) / 2000 + 2.3) * direction
            point_arr[8] += curve_weight * (curve_point if front_curve_angles < 0 else -10)
            point_arr[7] += curve_weight * (curve_point * 0.8)
            point_arr[6] += curve_weight * (curve_point * 0.6)
            point_arr[5] += curve_weight * (curve_point * 0.4)
            point_arr[4] += curve_weight * (curve_point * 0.2)
            point_arr[3] -= curve_weight * (curve_point * 0.4)
            point_arr[2] -= curve_weight * (curve_point * 0.6)
            point_arr[1] -= curve_weight * (curve_point * 0.8)
            point_arr[0] -= curve_weight * (curve_point if front_curve_angles > 0 else 10)

    def set_obstacles_point(self, i, my_area, point_arr, sensing_info):
        if len(sensing_info.track_forward_obstacles):
            for obj in sensing_info.track_forward_obstacles:
                obj_dist = int(obj['dist'] / 10)
                if i <= obj_dist <= i + 4 or obj_dist == i - 1:
                    obj_area = self.get_area(obj['to_middle'])
                    # if self.tick_count % 3 == 1:
                        # print(self.print_area(obj_area))
                    point_arr[obj_area] += -90
                    if obj_area >= 1:
                        point_arr[obj_area - 1] += -8
                    if obj_area <= self.total_road:
                        point_arr[obj_area + 1] += -8
                    if obj_dist < 40 and abs(obj['to_middle'] - sensing_info.to_middle) < 2:
                        if obj_area >= 2: # 맵 왼쪽에서 두칸 안쪽
                            point_arr[obj_area - 2] += -7
                        if obj_area <= self.total_area - 2: # 맵 오른쪽에서 두칸 안쪽
                            point_arr[obj_area + 2] += -7
                    if obj_area >= my_area:
                        for j in range(9 - obj_area):
                            point_arr[obj_area + j] += -6
                    else:
                        for j in range(obj_area + 1):
                            point_arr[obj_area - j] += -6

    def set_map_specified_point(self, point_arr, sensing_info):
        if self.map_code == 30:
            if 8.5 < sensing_info.lap_progress < 9.5:
                point_arr[self.total_road] = 900
            if 9.5 < sensing_info.lap_progress < 11.5:
                point_arr[1] = 900
            if 28.5 < sensing_info.lap_progress < 30.0:
                point_arr[self.total_road] = 900



    def get_steering_to_area(self, sensing_info, my_area, ideal_area, i):
        area_diff = ideal_area - my_area
        area_angle = sensing_info.track_forward_angles[i] - sensing_info.moving_angle - sensing_info.to_middle + area_diff * 0.7

        steering = float(area_angle * (abs(area_diff*0.7) * 0.1 + 0.1) * (self.check_range-i) * 0.005)
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
        for i in range(self.total_area+1):
            map += "●" if area == i else "○"
        return map

    def print_area_value(self, point_arr):
        map = ""
        for i in range(len(point_arr)):
            left = "   "
            right = " || " if i == 0 or i == self.total_road else "   "
            point = round(float(point_arr[i]), 1)
            center = str(point) if abs(point_arr[i]) > 10 else " " + str(point)
            center = center if point_arr[i] < 0 else " " + center
            if point_arr[i] == max(point_arr):
                center = "[" + center + "]"
            else:
                center = " " + center + " "
            map += left + center + right
        return map

    def recovery(self, car_controls, sensing_info):
        if sensing_info.moving_forward is False:
            if self.wrong_way_flag is False:
                self.wrong_way_flag = True
                if sensing_info.to_middle > 0:
                    self.steering_while_return = 1
                else:
                    self.steering_while_return = -1
            else:
                car_controls.steering = self.steering_while_return
        else:
            self.wrong_way_flag = False
            self.steering_while_return = 0

        if sensing_info.collided == True or self.collision_time > 0:
            if self.collision_flag is True:
                self.collision_flag = False
            else:
                self.collision_flag = False
                self.collision_time += 1
                if sensing_info.speed < -10 or sensing_info.speed > 10:
                    self.stopped_back += 1
                if self.collision_time < 13:
                    car_controls.throttle = -1
                    car_controls.steering = sensing_info.to_middle
                elif self.collision_time < 20:
                    car_controls.throttle = 1
                elif self.stopped_back < 1 and self.collision_time < 200:
                    car_controls.throttle = -1
                    car_controls.steering = 0
                else:
                    self.collision_time = 0
                    self.collision_flag = True
                    self.soppted_back = 0

        else:
            self.collision_flag is True

if __name__ == '__main__':
    client = DrivingClient()
    client.run()