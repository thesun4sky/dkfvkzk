from drive_controller import DrivingController
import time


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = True
        self.collision_flag = True
        self.start = time.time()
        self.tickCount = 0
        self.totalSpeed = 0

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
        targetPosition = 0
        center = 1.2
        maxSpeed = 100
        positionWeight = 1.2
        dodgeWeight = 0.7
        directionWeight = 2
        futureWeight = 2.5

        if len(sensing_info.track_forward_obstacles):
            print(sensing_info.track_forward_obstacles[0])
            if sensing_info.track_forward_obstacles[0]['to_middle'] > center:
                target = -2.75
            elif sensing_info.track_forward_obstacles[0]['to_middle'] < -center:
                target = 2.75
            elif sensing_info.track_forward_obstacles[0]['to_middle'] > 0:
                target = -3
            elif sensing_info.track_forward_obstacles[0]['to_middle'] < 0:
                target = 3

            targetPosition = target * ((100 - sensing_info.track_forward_obstacles[0]['dist']) / 100)
            positionWeight = positionWeight + (
                        (100 - sensing_info.track_forward_obstacles[0]['dist']) / 100) * dodgeWeight

            print("targetPosition: {}, positionWeight: {}".format(targetPosition, positionWeight))
            # sensing_info.to_middle *= 2

        if abs(sum(sensing_info.track_forward_angles)) < 50:
            throttle = 1
        elif sensing_info.speed > maxSpeed:
            throttle = 0
        else:
            throttle = 1

        totalWeight = positionWeight + directionWeight + futureWeight
        positionValue = -((sensing_info.to_middle - targetPosition) / 10 * positionWeight)
        directionValue = -(sensing_info.moving_angle / 90 * directionWeight)
        futureValue = (sensing_info.track_forward_angles[1] * 1.2 + sensing_info.track_forward_angles[2] * 0.2 +
                       sensing_info.track_forward_angles[3] * 0.1) / 90 * futureWeight * sensing_info.speed / 70

        # Moving straight forward
        car_controls.steering = (positionValue + directionValue + futureValue) / totalWeight
        car_controls.throttle = throttle
        car_controls.brake = 0 if sensing_info.speed < 30 else abs(car_controls.steering) * 1.5
        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                              car_controls.brake))
        if sensing_info.lap_progress == 100:
            print("time :", time.time() - self.start)

        if sensing_info.lap_progress != 0:
            self.totalSpeed += sensing_info.speed
            self.tickCount += 1
            print("avgSpeed : {}", self.totalSpeed / self.tickCount)
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


if __name__ == '__main__':
    client = DrivingClient()
    client.run()