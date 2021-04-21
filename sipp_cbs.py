"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
"""
import sys
sys.path.insert(0, '../')
import argparse
from math import fabs
from itertools import combinations
from copy import deepcopy
import time as timer
from sipp_astar import SippPlanner
from graph_generation import SippGraph, State
import random

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors
    
    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

class HighLevelNode(object):
    def __init__(self):
        self.solution = []
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class SIPP_CBSSolver(object):
    #TODO: bool variable - collision constraint to indicate interval allow
    #TODO: add bool variable to plan in find soln
    #TODO: create find_soln()
    #TODO: parse path to find neg paths to add as dynamic obstacles
    #TODO: create_constraint_from_conflict() - change vertex and edge constraint -> time interval constraint ; +/- 1 timestep of safe interval
    #TODO: update edge & vertex collision -> dynamic obstacles 
    #TODO: lovel search - astar

    def __init__(self, filename, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.filename = filename
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.open_set = set()
        self.closed_set = set()
    
    # finds actual collision and returns path
    def detect_collision(self,path1, path2):    
        max_t = max(len(path1), len(path2))

        if max_t == len(path1):
            longer = 1
        else:
            longer = 2
        
        # list of obstacles
        # may need to consider edge collision
        for t in range(max_t):
            if t <= len(path1):
                loc_1 =(path1[len(path1)-1]['x'], path1[len(path1)-1]['y'])
            else:    
                loc_1 =(path1[t]['x'], path1[t]['y'])

            if t <= len(path2):
                loc_2 =(path2[len(path2)-1]['x'], path2[len(path2)-1]['y'])
            else:    
                loc_2 =(path2[t]['x'], path2[t]['y'])

            if loc_1 == loc_2:
                if longer == 1:
                    return path1[t]
                else:
                    return path2[t]
    
    # iterates through agents to find collisions
    def agent_get_first_collision(self, paths):
        result = []
        index = 0
        for i in range(len(paths)):
            for k in range(i+1, len(paths)):
                result = {'agent1': i, 'agent2': k, 'collision_loc': self.detect_collision(paths[i], paths[k]) }
                #collision detected
                if result['collision_loc'] != None:
                    return result
        # no collision found
        return None
    
    #x,y,t (path) == dynamic obstacles
    def add_constraint(self, constraint):
        constraint_set = {}
        for loc in constraint: # same loc diff ts
            constraint_set.append( {'x':loc['x'], 'y':loc['y'], t:loc['t'] } )
        return constraint_set

    # disjoint splitting
    # TODO: potential edge constraints
    def create_constraints_from_conflict(self, conflict):
        value = random.randint(0,1)
        if value == 0:
            chosen_agent = conflict['agent1']
        else:
            chosen_agent = conflict['agent2']

        constraint_dict = {}

        temp = {}
        for i in range(-1, 2): #two sets of constraint
            print("\tCONFLICT:")
            print(conflict)
            if conflict['t'] == 0:
                continue
            temp.append({'agent': chosen_agent, 'x':conflict['x'], 'y': conflict['y'], 't': conflict['t']+i}) #choosen agent can't be in this loc at ts +/- 1 -> negative constraint
        conflict_dict[0] = temp
        
        temp = {}
        for i in range(-1, 2): #two sets of constraint
            if conflict['t'] == 0:
                continue
            temp.append({'agent': -1, 'x':conflict['x'], 'y': conflict['y'], 't': conflict['t']+i}) #applies to every other agent can't be in this loc at ts +/- 1 -> positive constraint i.e. dynamical obstacle for other agents
        conflict_dict[1] = temp
        
        # if conflict.type == Conflict.VERTEX:
        #     v_constraint = VertexConstraint(conflict.time, conflict.location_1)
        #     constraint = Constraints()
        #     constraint.vertex_constraints |= {v_constraint}
        #     constraint_dict[conflict.agent_1] = constraint
        #     constraint_dict[conflict.agent_2] = constraint

        # elif conflict.type == Conflict.EDGE:
        #     constraint1 = Constraints()
        #     constraint2 = Constraints()

        #     e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
        #     e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

        #     constraint1.edge_constraints |= {e_constraint1}
        #     constraint2.edge_constraints |= {e_constraint2}

        #     constraint_dict[conflict.agent_1] = constraint1
        #     constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution])

    def process_result(self, result):
        """" Converts dictionary path to tuple array path for all agents """

        presult = []

        for i in range(len(result)):
            li=[]
            for k in range(len(result[i])):
                li.append((result[i][k]['x'], result[i][k]['y']))
            presult.append(li)            
        return presult    

    def find_solution(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        # TODO: low level search for each agent with SIPP
        
        start.constraint_dict = {}
        #low level search
        for agent in range(self.num_of_agents):
            start.constraint_dict[agent] = {}
            sipp_planner = SippPlanner(self.filename, self.my_map.agent_info, agent, start.constraint_dict[agent])
            if sipp_planner.compute_plan():
                start.solution.append(sipp_planner.get_plan())
        print("Start.Solution")
        print(start.solution)
        start.cost = self.compute_solution_cost(start.solution)             

        # for agent in self.env.agent_dict.keys():
        #     start.constraint_dict[agent] = Constraints()
        # start.solution = self.env.compute_solution()
        # if not start.solution:
        #     return {}
        # start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            # self.env.constraint_dict = P.constraint_dict

            conflict_dict = self.agent_get_first_collision(P.solution)
            if not conflict_dict:
                print("solution found")
                result = self.process_result(self.generate_plan(P.solution))
                return result

            tmp_constraint_dict = self.create_constraints_from_conflict(conflict_dict)
            # constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for constraint in tmp_constraint_dict:
                new_node = deepcopy(P)
                if constraint['agent'] >= 0:
                    curr_agent = constraint['agent']
                    new_node.constraint_dict[constraint['agent']].append(self.add_constraint(constraint))
    
                    sipp_planner = SippPlanner(self.filename, self.my_map.agent_info, constraint['agent'], new_node.constraint_dict[constraint['agent']])
                    if sipp_planner.compute_plan():
                        new_node.solution[constraint['agent']] = sipp_planner.get_plan()
                        if not new_node.solution[curr_agent]:
                                emptyPathFound = True
                else:
                    tmp = self.add_constraint(constraint)
                    for agent in range(self.num_agents):
                        if agent == constraint['agent']:
                            continue   
                        curr_agent = agent
                        new_node.constraint_dict[agent].append(tmp)
                        sipp_planner = SippPlanner(self.filename, self.my_map.agent_info, agent, new_node.constraint_dict[agent])
                        if sipp_planner.compute_plan():
                            new_node.solution[agent] = sipp_planner.get_plan()
                            if not new_node.solution[curr_agent]:
                                emptyPathFound = True
                # self.env.constraint_dict = new_node.constraint_dict
                # new_node.solution = self.env.compute_solution()
                if emptyPathFound:
                    continue
                # new_node.cost = self.env.compute_solution_cost(new_node.solution)
                new_node.cost = self.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}
        return {}

    def generate_plan(self, solution):
        plan = []
        for agent_path in solution:
            path_dict_list=[]
            for loc in agent_path:
                path_dict_list.append( {'x':loc['x'], 'y':loc['y'], 't':loc['t']})
            #plan[agent]= path_dict_list
            plan.append(path_dict_list)

            # path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            # plan[agent] = path_dict_list
        return plan


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("param", help="input file containing map and obstacles")
#     parser.add_argument("output", help="output file with the schedule")
#     args = parser.parse_args()

#     # Read from input file
#     with open(args.param, 'r') as param_file:
#         try:
#             param = yaml.load(param_file, Loader=yaml.FullLoader)
#         except yaml.YAMLError as exc:
#             print(exc)

#     dimension = param["map"]["dimensions"]
#     obstacles = param["map"]["obstacles"]
#     agents = param['agents']

#     env = Environment(dimension, agents, obstacles)

#     # Searching
#     cbs = CBS(env)
#     solution = cbs.search()
#     if not solution:
#         print(" Solution not found" )
#         return

#     # Write to output file
#     with open(args.output, 'r') as output_yaml:
#         try:
#             output = yaml.load(output_yaml, Loader=yaml.FullLoader)
#         except yaml.YAMLError as exc:
#             print(exc)

#     output["schedule"] = solution
#     output["cost"] = env.compute_solution_cost(solution)
#     with open(args.output, 'w') as output_yaml:
#         yaml.safe_dump(output, output_yaml)


# if __name__ == "__main__":
#     main()