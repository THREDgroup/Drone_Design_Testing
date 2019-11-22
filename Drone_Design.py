# ORIGINAL DESIGN FROM ARL: *aMM0+++++*bNM2++*cMN1++*dLM2++*eML1++^ab^ac^ad^ae,2,3
#
# * separates nodes
# ^ separates connections
# first lower-case letter is node name
# first capital letter is direction 1 location
# second capital letter is direction 2 location
# number is what's on top
#
# first number at end is payload
# second number at end is controller number
#
# coordinate system starts at M,M

import string
import numpy as np
import random
import matplotlib.pyplot as plt
import subprocess
import copy


class DroneDesign:
    # This alphabet is needed to convert the design to the grammar form used by the testbed
    # M is 13th letter, 12th index
    alphabet_upper = string.ascii_uppercase
    alphabet_lower = string.ascii_lowercase

    # This is the directory where the testbed is located
    testbed_directory = 'C:\\Users\\lucas\\Documents\\Penn State\\Research\\Drone Design Testing\\ateams-eval-x64-2019-11-04'

    # TODO: Rework nodes/connections to better delete existing nodes
    def __init__(self):
        ## Define initial design

        # String used as input to testbed
        self.full_design_string = ''
        # Contains information of all nodes
        self.node_information = []
        # Contains all connections between nodes
        self.connections = []
        # Payload weight
        self.payload_weight = 0
        # Controller number
        self.controller_number = 0

        # Generate the initial design (design currently used is what was in ARL's original file)
        self.generate_initial_design()
        # Converts the current design to the form used in testbed
        self.convert_design_to_string()

        # Defines which nodes are currently in use
        self.current_grid = np.zeros((26, 26))
        # Defines which nodes are available to add
        self.available_grid = np.zeros((26, 26))
        # Specifies the locations available
        self.available_node_locations = []
        # Specifies where new connections are available
        self.available_connections = []
        # Specifies which current nodes are motors
        self.current_motor_nodes = []

        ## Design evaluation metrics
        # Says whether tested design was successful or failed
        self.design_successful = 'Success'
        # Design successful range
        self.successful_range = 0
        # Design velocity
        self.velocity = 0
        # Design cost
        self.cost = 0

        # Number of rules in grammar ruleset
        self.num_rules = 19

    # This function generates the initial design (this current one is ARL's default)
    def generate_initial_design(self):
        nodes = [(12, 12, 0, 5), (13, 12, 2, 2), (12, 13, 1, 2), (11, 12, 2, 2), (12, 11, 1, 2)]
        connections = [(0, 1), (0, 2), (0, 3), (0, 4)]

        for node in nodes:
            self.add_node(node[0], node[1], node[2], node[3])
        for connection in connections:
            self.add_connection(connection[0], connection[1])
        self.change_payload_weight(2)
        self.change_controller(3)

    # This function converts the current design into a representation usable within the testbed
    def convert_design_to_string(self):
        all_nodes_string = ''
        for node_index in range(len(self.node_information)):
            node = self.node_information[node_index]
            if node[0] != -1:
                node_letter = self.alphabet_lower[node_index]
                direction_a = self.alphabet_upper[node[0]]
                direction_b = self.alphabet_upper[node[1]]
                node_top = str(node[2])
                node_top_size = ''
                for i in range(node[3]):
                    node_top_size = node_top_size + '+'
                node_string = node_letter + direction_a + direction_b + node_top + node_top_size
                all_nodes_string = all_nodes_string + '*' + node_string

        all_connections_string = ''
        for connect in self.connections:
            if connect[0] != -1:
                node_a = self.alphabet_lower[connect[0]]
                node_b = self.alphabet_lower[connect[1]]
                connection_string = node_a + node_b
                all_connections_string = all_connections_string + '^' + connection_string

        self.full_design_string = all_nodes_string + all_connections_string + ',' + \
                             str(self.payload_weight) + ',' + str(self.controller_number)

    # This function converts the design into correct representation and then saves it as the input file used in the testbed
    def save_current_design_as_input(self):
        self.convert_design_to_string()
        design_input_file = 'input.txt'
        file = open(self.testbed_directory + '\\' + design_input_file, 'w')
        file.write(self.full_design_string)

    # This function runs the testbed using the current saved design and extracts the design evaluation metrics
    def evaluate_design(self):
        testbed_execution_line = '.\\ateams.exe -quit -batchmode -output -outputAll'
        results_file = 'data.txt'

        subprocess.call(testbed_execution_line, shell=True, cwd=self.testbed_directory)

        results_file_data = str(open(self.testbed_directory+'\\'+results_file).read())

        # Design evaluation metrics are extracted from data file and stored
        _, self.design_successful, self.successful_range, self.velocity, self.cost = results_file_data.split(";")


#######################################################################################################################
    # Many of these are basic functions needed to alter the design as used in the grammar rules

    # Add a new node at the specified location, with specified type and size
    def add_node(self, coord_a, coord_b, node_top, node_top_size):
        node = (coord_a, coord_b, node_top, node_top_size)
        self.node_information.append(node)

    # Add new connection between two nodes
    def add_connection(self, node_a, node_b):
        connection = (node_a, node_b)
        self.connections.append(connection)

    # Change the payload carried by the drone
    def change_payload_weight(self, new_weight):
        self.payload_weight = new_weight

    # Change the controller used to control the drone
    def change_controller(self, new_controller):
        self.controller_number = new_controller

    def remove_node(self, node_index):
        self.node_information[node_index] = (-1, -1, -1, -1)

    def remove_connection(self, connection_index):
        self.connections[connection_index] = (-1, -1)

    def add_and_connect_node(self, coord_a, coord_b, node_top, node_top_size, node_to_connect):
        self.add_node(coord_a, coord_b, node_top, node_top_size)
        node_index = len(self.node_information) - 1
        self.add_connection(node_to_connect, node_index)

    def remove_node_and_connections(self, node_index):
        self.remove_node(node_index)
        for connection_index in range(len(self.connections)):
            connection = self.connections[connection_index]
            if connection[0] == node_index or connection[1] == node_index:
                self.remove_connection(connection_index)

#######################################################################################################################
    # The following are the functions used to apply each rule, given the specified inputs
    # TODO: Finish these

    # Rule 1
    def add_clockwise_motor_rule(self, location, connected_node, propeller_size):
        coord_a = location[0]
        coord_b = location[1]
        node_top = 1
        node_top_size = propeller_size
        node_to_connect = connected_node
        self.add_and_connect_node(coord_a, coord_b, node_top, node_top_size, node_to_connect)

    # Rule 2
    def add_counterclockwise_motor_rule(self, location, connected_node, propeller_size):
        coord_a = location[0]
        coord_b = location[1]
        node_top = 2
        node_top_size = propeller_size
        node_to_connect = connected_node
        self.add_and_connect_node(coord_a, coord_b, node_top, node_top_size, node_to_connect)

    # Rule 3
    def add_structure_rule(self):
        pass

    # Rule 4
    def add_foil_rule(self):
        pass

    # Rule 5
    def add_new_connection_rule(self):
        pass

    # Rule 6
    def remove_motor_and_connections_rule(self, node_index):
        self.remove_node_and_connections(node_index)

    # Rule 7
    def remove_structure_rule(self):
        pass

    # Rule 8
    def remove_foil_rule(self):
        pass

    # Rule 9
    def remove_connection_rule(self):
        pass

    # Rule 10
    def change_motor_direction_rule(self, node_index):
        node = self.node_information[node_index]
        old_direction = node[2]
        if old_direction == 1:
            new_direction = 2
        else:
            new_direction = 1
        self.node_information[node_index] = (node[0], node[1], new_direction, node[3])

    # Rule 11
    def reposition_motor_rule(self, node_index, new_location):
        node = self.node_information[node_index]
        new_node = node
        new_node[0] = new_location[0]
        new_node[1] = new_location[1]
        self.remove_node_and_connections(node_index)
        # Todo: FIX
        self.add_and_connect_node(new_node[0], new_node[1], new_node[2], new_node[3])

    # Rule 12
    def reposition_structure_rule(self):
        pass

    # Rule 13
    def reposition_foil_rule(self):
        pass

    # Rule 14
    def increase_motor_size_rule(self):
        pass

    # Rule 15
    def increase_structure_size_rule(self):
        pass

    # Rule 16
    def increase_foil_size_rule(self):
        pass

    # Rule 17
    def decrease_motor_size_rule(self):
        pass

    # Rule 18
    def decrease_structure_size_rule(self):
        pass

    # Rule 19
    def decrease_foil_size_rule(self):
        pass


#######################################################################################################################
    # This function randomly selects a rule from all available rules
    def select_random_rule(self):
        rule = random.randint(1, self.num_rules)
        return rule

    # This function applies the specified rule. The inputs used for each individual rule are randomly chosen if not given
    # TODO
    def apply_rule(self, rule_number, **kwargs):
        minimum_propeller_size = 0
        maximum_propeller_size = 5
        self.assess_possible_design_changes()

        if rule_number == 1:
            node_to_add = self.available_node_locations[random.randint(0, len(self.available_node_locations) - 1)]
            propeller_size = random.randint(minimum_propeller_size, maximum_propeller_size)
            self.add_clockwise_motor_rule(node_to_add[0], node_to_add[1], propeller_size)
        elif rule_number == 2:
            node_to_add = self.available_node_locations[random.randint(0, len(self.available_node_locations) - 1)]
            propeller_size = random.randint(minimum_propeller_size, maximum_propeller_size)
            self.add_counterclockwise_motor_rule(node_to_add[0], node_to_add[1], propeller_size)
        elif rule_number == 3:
            pass
        elif rule_number == 4:
            pass
        elif rule_number == 5:
            pass
        elif rule_number == 6:
            pass
        elif rule_number == 7:
            pass
        elif rule_number == 8:
            pass
        elif rule_number == 9:
            pass
        elif rule_number == 10:
            node_to_change = self.current_motor_nodes[random.randint(0, len(self.current_motor_nodes))]
            self.change_motor_direction_rule(node_to_change)
        elif rule_number == 11:
            pass
        elif rule_number == 12:
            pass
        elif rule_number == 13:
            pass
        elif rule_number == 14:
            pass
        elif rule_number == 15:
            pass
        elif rule_number == 16:
            pass
        elif rule_number == 17:
            pass
        elif rule_number == 18:
            pass
        elif rule_number == 19:
            pass

        self.assess_possible_design_changes()

#######################################################################################################################
    # This function analyzes the current design information to determine which aspects are valid for alteration
    def assess_possible_design_changes(self):
        self.current_grid = np.zeros((26, 26))
        for n in range(len(self.node_information)):
            node = self.node_information[n]
            coords = (node[0], node[1])
            self.current_grid[coords] = 1
        self.available_grid = np.zeros((26, 26))
        for i in range(26):
            for j in range(26):
                if i > 0 and i < 25 and j > 0 and j < 25:
                    if self.current_grid[(i, j)] != 1:
                        if self.current_grid[(i+1, j)] == 1 or self.current_grid[(i-1, j)] == 1 or \
                                self.current_grid[(i, j+1)] == 1 or self.current_grid[(i, j-1)] == 1:
                            self.available_grid[(i, j)] = 1

        self.available_node_locations = []
        for i in range(26):
            for j in range(26):
                location = (i, j)
                if self.available_grid[location] == 1:
                    valid_connection_locations = []
                    if self.current_grid[location[0] + 1, location[1]] == 1:
                        valid_connection_locations.append((location[0] + 1, location[1]))
                    if self.current_grid[location[0], location[1] + 1] == 1:
                        valid_connection_locations.append((location[0], location[1] + 1))
                    if self.current_grid[location[0] - 1, location[1]] == 1:
                        valid_connection_locations.append((location[0] - 1, location[1]))
                    if self.current_grid[location[0], location[1] - 1] == 1:
                        valid_connection_locations.append((location[0], location[1] - 1))

                    for connection_location in valid_connection_locations:
                        for node_index in range(len(self.node_information)):
                            node = self.node_information[node_index]
                            if connection_location[0] == node[0] and connection_location[1] == node[1]:
                                connected_node = node_index
                                break
                        self.available_node_locations.append([location, connected_node])

        self.current_motor_nodes = []
        for node_index in range(len(self.node_information)):
            node = self.node_information[node_index]
            type = node[2]
            if type == 1 or type == 2:
                self.current_motor_nodes.append(node_index)

        # TODO
        self.available_connections = []
        for i in range(len(self.node_information)):
            for j in range(len(self.node_information)):
                if i != j and i != -1 and j != -1:
                    pass

    # This function is used to visually represent the design, if desired
    def display_design(self):
        red = [255, 0, 0]
        blue = [0, 0, 255]
        white = [255, 255, 255]
        gray = [150, 150, 150]
        black = [0, 0, 0]

        current_design_coded = np.ones((26, 26, 3))
        current_design_coded = current_design_coded*255.
        for n in range(len(self.node_information)):
            node = self.node_information[n]
            location = (node[0], node[1])
            type = node[2]
            size = node[3]
            if type == 0:
                current_design_coded[location] = black
            elif type == 1:
                current_design_coded[location] = red
            elif type == 2:
                current_design_coded[location] = blue

        for i in range(26):
            for j in range(26):
                if self.available_grid[i, j] == 1.:
                    current_design_coded[i, j] = gray

        current_design_coded = current_design_coded / 255
        fig, ax = plt.subplots()
        im = ax.imshow(current_design_coded, interpolation='nearest')
        ax.grid(color='k', linestyle='-', linewidth=0.25)
        ax.set_xticks(np.arange(-0.5, 25, 1))
        ax.set_yticks(np.arange(-0.625, 25, 1))
        ax.set_xticklabels([])
        ax.set_yticklabels([])

        for n in range(len(self.node_information)):
            size = self.node_information[n][3]
            location = (self.node_information[n][0], self.node_information[n][1])
            text = ax.text(location[1], location[0], size, ha="center", va="center", color="w")

        plt.show()

    # Function needed to make copy of Drone object by agent
    def __deepcopy__(self, memo):
        deepcopy_method = self.__deepcopy__
        self.__deepcopy__ = None
        cp = copy.deepcopy(self, memo)
        self.__deepcopy__ = deepcopy_method
        return cp