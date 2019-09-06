from maneuvers.kit import *

from maneuvers.driving.arrive import Arrive

class Kickoff(Maneuver):
    '''The simplest boost and dodge at the end kickoff.'''
    def __init__(self, car: Car, info: GameInfo):
        super().__init__(car)
        self.info = info

        is_corner_kickoff = abs(car.pos[0]) > 1000

        self.action = Arrive(car)
        self.action.target = info.ball.pos
        self.action.target_direction = direction(info.ball, info.their_goal.center)
        # self.action.lerp_t = 0.3 if is_corner_kickoff else 0.45
        self.action.lerp_t = 0.4
        self.action.allow_dodges_and_wavedashes = False

        self.dodging = False
        self.dodge = AirDodge(car, 0.05, info.ball.pos)

    def step(self, dt):
        if not self.dodging and distance(self.car, self.info.ball) < 800:

            # detect if an opponent is going for kickoff
            is_opponent_going_for_kickoff = False
            for opponent in self.info.opponents:
                if distance(self.info.ball, opponent) < 1500:
                    is_opponent_going_for_kickoff = True

            if is_opponent_going_for_kickoff:
                self.action = self.dodge
                self.dodging = True
            else:
                # if not, don't dodge and steer a bit to the side to aim for a top-corner
                self.action.target = self.info.ball.pos + vec3(100, 0, 0)

        self.action.step(dt)
        self.controls = self.action.controls
        self.finished = self.info.ball.pos[0] != 0

    def render(self, draw: DrawingTool):
        if not self.dodging:
            self.action.render(draw)
