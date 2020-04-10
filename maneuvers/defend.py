from maneuvers.driving.travel import Travel
from maneuvers.driving.stop import Stop
from maneuvers.maneuver import Maneuver
from rlutilities.linear_algebra import vec3, norm, dot, normalize
from rlutilities.simulation import Car
from utils.drawing import DrawingTool
from utils.game_info import GameInfo
from utils.vector_math import distance, direction
from utils.math import sign


STOP_DIST = 500
ROUNDABOUT_DIST_MIN = 1300
ROUNDABOUT_DIST_MAX = 4500
ROUNDABOUT_SIDE_X = 3500
GO_ANYWAY_DIST = 1500
PAD_SEEK_RADIUS = 800
PAD_ALIGN_LIMIT = 0.85


class ReturnToFarPost(Maneuver):

    def __init__(self, car: Car, info: GameInfo):
        super().__init__(car)
        self.info = info

        team_sign = 1 - 2 * info.team
        self.far_post = info.far_post + vec3(200 * sign(info.far_post[0]), 500 * team_sign, 0)

        self.travel = Travel(car)
        self.travel.target = self.far_post

    def step(self, dt: float):
        far_post_dist = distance(self.car.position, self.far_post)
        if far_post_dist > STOP_DIST:
            target = None
            if far_post_dist > ROUNDABOUT_DIST_MIN:
                # check for good pads to collect on the way
                speed = norm(self.car.velocity)
                good_pads = [
                    pad.position for pad in self.info.pads 
                    if distance(pad.position, self.car.position) < PAD_SEEK_RADIUS
                    and (pad.is_active or distance(pad.position, self.car.position) / speed > pad.timer)
                    and dot(normalize(self.car.velocity), normalize(pad.position - self.car.position)) > PAD_ALIGN_LIMIT
                ]
                # go to closest good pad to far post (to encourage correct direction)
                if good_pads:
                    target = min(good_pads, key=lambda pad: distance(self.far_post, pad))

                # roundabout move where it goes from the side
                elif far_post_dist < ROUNDABOUT_DIST_MAX:
                    ratio = ROUNDABOUT_DIST_MIN / far_post_dist
                    off_to_side = vec3(sign(self.far_post[0]) * ROUNDABOUT_SIDE_X, self.car.position[1], 0)
                    target = (ratio * self.far_post + (1.0 - ratio) * off_to_side)

            # go directly to far post
            if target is None:
                target = self.far_post

            self.travel.target = target
            self.travel.step(dt)
            self.controls = self.travel.controls

        else:
            self.finished = True

    def render(self, draw: DrawingTool):
        self.travel.render(draw)


class Defend(Maneuver):
    
    def __init__(self, car: Car, info: GameInfo):
        super().__init__(car)
        self.info = info

        self.return_to_far_post = ReturnToFarPost(car, info)
        self.stop = Stop(car)

    def step(self, dt: float):
        if not self.return_to_far_post.finished:
            self.return_to_far_post.step(dt)
            self.controls = self.return_to_far_post.controls
        else:
            self.stop.step(dt)
            self.controls = self.stop.controls

            self.finished = self.info.about_to_be_scored_on

        if distance(self.info.ball.position, self.info.my_goal.center) < GO_ANYWAY_DIST:
            self.finished = True

    def render(self, draw: DrawingTool):
        if not self.return_to_far_post.finished:
            self.return_to_far_post.render(draw)
        else:
            self.stop.render(draw)
