from Drone_Design import DroneDesign
import Solution
import numpy as np
import random
import copy


# TODO: CHECK ALL OF THIS
class Agent:

    def __init__(self):
        self.design = Solution.Solution()
        self.design.convert_design_to_string()
        self.design.save_current_design_as_input()
        self.design.evaluate_design()

        self.candidate_design = copy.deepcopy(self.design)

    def iterate(self):
        self.candidate_design = copy.deepcopy(self.design)

        rule = 1
        self.candidate_design.apply_rule(rule)
        self.candidate_design.convert_design_to_string()
        self.candidate_design.save_current_design_as_input()
        self.candidate_design.evaluate_design()

    # TODO
    def evaluate(self):
        if self.candidate_design.design_successful == 'Success':
            print('SUCCESS')
            if self.candidate_design.cost > self.design.cost:
                self.design = copy.deepcopy(self.candidate_design)
            print(self.design.cost, self.candidate_design.cost)


    def random_rule_select(self):
        rule = self.design.select_random_rule()
        return rule

