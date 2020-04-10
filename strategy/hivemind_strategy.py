from typing import List, Optional, Dict

from maneuvers.air.recovery import Recovery
from maneuvers.driving.stop import Stop
from maneuvers.kickoffs.simple_kickoff import SimpleKickoff
from maneuvers.kickoffs.speed_flip_kickoff import SpeedFlipKickoff
from maneuvers.refuel import Refuel
from maneuvers.shadow_defense import ShadowDefense
# from maneuvers.defend import Defend
from maneuvers.strikes.clear_into_corner import ClearIntoCorner
from maneuvers.strikes.strike import Strike
from rlutilities.simulation import Car, Pad
from rlutilities.linear_algebra import vec3
from strategy.kickoffs import KickoffStrategy
from strategy.offense import Offense
from utils.arena import Arena
from utils.drawing import DrawingTool
from utils.drone import Drone
from utils.game_info import GameInfo
from utils.intercept import Intercept, estimate_time
from utils.vector_math import align, ground, ground_distance, distance, nearest_point


GOOD_ALIGNMENT_MIN = 0.0

class HivemindStrategy:
    def __init__(self, info: GameInfo, logger):
        self.info: GameInfo = info
        self.offense: Offense = Offense(info)

        self.logger = logger

        # the drone that is currently committed to hitting the ball
        self.drone_going_for_ball: Optional[Drone] = None
        self.defending_drone: Optional[Drone] = None

        self.boost_reservations: Dict[Drone, Pad] = {}

    def set_maneuvers(self, drones: List[Drone]):
        info = self.info
        their_goal = ground(info.their_goal.center)
        our_goal = ground(info.my_goal.center)

        if self.drone_going_for_ball is not None and self.drone_going_for_ball.maneuver is None:
            self.drone_going_for_ball = None

        if self.defending_drone is not None and self.defending_drone.maneuver is None:
            self.defending_drone = None

        # recovery
        for drone in drones:
            if drone.maneuver is None and not drone.car.on_ground:
                drone.maneuver = Recovery(drone.car)

        # ready if on ground and alive
        ready_drones = [drone for drone in drones if drone.car.on_ground and not drone.controls.jump and not drone.car.demolished]
        if not ready_drones:
            return
        
        # kickoff
        if info.kickoff_pause:
            self.drone_going_for_ball = min(drones, key=lambda drone: distance(vec3(0, 0, 0), drone.car.position))
            self.drone_going_for_ball.maneuver = KickoffStrategy.choose_kickoff(info, self.drone_going_for_ball.car)

        else:
            info.predict_ball()
            our_intercepts = [Intercept(drone.car, info.ball_predictions) for drone in ready_drones]
            good_intercepts = [i for i in our_intercepts if align(i.car.position, i.ball, their_goal) > GOOD_ALIGNMENT_MIN]

            # shot opportunities exist
            if good_intercepts:
                best_intercept = min(good_intercepts, key=lambda intercept: intercept.time)
                strike = self.offense.any_shot(best_intercept.car, their_goal, best_intercept)

            else:
                opp_intercepts = [Intercept(opponent, info.ball_predictions) for opponent in info.get_opponents(drone.car)]
                best_opp_time = min(map(lambda intercept: intercept.time, opp_intercepts))
                faster_intercepts = [intercept for intercept in our_intercepts if intercept.time < best_opp_time]

                # we are faster than the fastest opponent, try to get the best shot
                if faster_intercepts:
                    best_intercept = max(faster_intercepts, key=lambda i: align(i.car.position, i.ball, their_goal))
                    strike = self.offense.any_shot(best_intercept.car, their_goal, best_intercept)

                # looks like we are slower, try to be first at the ball
                else:
                    best_intercept = min(our_intercepts, key=lambda intercept: intercept.time)
                    if ground_distance(best_intercept, our_goal) > 6000:
                        strike = self.offense.any_shot(best_intercept.car, their_goal, best_intercept)
                    else:
                        strike = ClearIntoCorner(best_intercept.car, info)

            # 🥴
            self.drone_going_for_ball = next(drone for drone in ready_drones if drone.car == best_intercept.car)
            self.drone_going_for_ball.maneuver = strike

        # clear expired boost reservations
        for drone in drones:
            if not isinstance(drone.maneuver, Refuel) and drone in self.boost_reservations:
                del self.boost_reservations[drone]

        for drone in drones:
            if drone.maneuver is None:
                if drone.car.boost < 40:
                    reserved_pads = {self.boost_reservations[drone] for drone in self.boost_reservations}
                    drone.maneuver = Refuel(drone.car, info, info.ball.position, forbidden_pads=reserved_pads)
                    self.boost_reservations[drone] = drone.maneuver.pad  # reserve chosen boost pad
                else:
                    shadow_distance = 3000
                    if self.defending_drone is None:
                        self.defending_drone = drone
                        shadow_distance = 8000
                    drone.maneuver = ShadowDefense(drone.car, info, info.ball.position, shadow_distance)

    def render(self, draw: DrawingTool):
        pass
