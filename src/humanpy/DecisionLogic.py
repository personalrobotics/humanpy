from Model2 import Data


class DecisionLogic:

    ROUND_LIMIT = 2

    def __init__(self):
        self.x = Data(1)
        self.current_table_theta = self.x.startStateTheta
        self.is_game_over = False
        self.round_count = 0


    def does_agree_to_move(self, direction):
        assert(0 <= direction <= 1)
        [new_table_theta, next_state] = self.x.stateUpdateFromHumanAction(
            direction, self.x)
        if self.current_table_theta != new_table_theta:
            print "Agreed. current theta = {}".format(self.current_table_theta)
            self.current_table_theta = new_table_theta
            return True
        else:
            print "Disagreed. current theta = {}".format(self.current_table_theta)
            return False

    def update_is_game_over(self):
        if self.current_table_theta == self.x.goal1StateTheta:
            self.x.currState = self.x.goal1RestartStateIndx
        elif self.current_table_theta == self.x.goal2StateTheta:
            self.x.currState = self.x.goal2RestartStateIndx
        else:
            self.game_over = False
            return False
        self.current_table_theta = self.x.startStateTheta
        self.game_over = True
        self.round_count += 1
        return True
