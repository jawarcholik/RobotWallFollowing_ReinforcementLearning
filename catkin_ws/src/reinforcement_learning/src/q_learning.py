#!/usr/bin/python2

import sys
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
import math
import numpy as np
import random
import time
import json
import matplotlib.pyplot as plt


"""
State Space
"""
#Regions: Left, Front, Front-Right, Right, ?Orientation?
#Distances:
    #Left: Close, Far
    #Front: Too Close, Medium, Far
    #Front-Right: Close, Far
    #Right: Too Close, Close, Medium, Far, Too Far

"""
Action spaces
"""
ACTIONS = {
    'F': [.2,0],
    'L': [.2,math.pi/4],
    'R': [.2,-math.pi/4],
    'S': [0,0]
}

"""
Q-Table
"""
class qTable():
    def __init__(self):
        # print("NEW QTABLE")
        self.table = dict()

        # Test that table is updating
        self.initialTable = dict()

    def __getitem__(self,i): #TODO: Is This Function Ever Used?????????????????????
        if i in self.table:
            return self.table[i]

        print("NEW KEY")
        self.table[i] = [0, 0, 0]
        return self.table[i]

    def maxAction(self, state, actions):
        forwardWeight = 0
        leftWeight = 0
        rightWeight = 0
        tup = tuple(state)

        if tup not in self.table:
            #### Naeve Implementation ####
            # if state[1] != 'TC':
            #     forwardWeight += 10
            # if state[1] == 'TC' or state[1] == 'C':
            #     forwardWeight -= 5
            # if state[1] == 'M':
            #     leftWeight += 10
            # if state[0] != 'C':
            #     leftWeight += 5
            # if state[2] == 'C':
            #     leftWeight += 5
            #     rightWeight -= 5
            #     forwardWeight -= 2
            # if state[3] == 'TC':
            #     leftWeight += 10
            # if state[3] == 'F' or state[3] == 'TF':
            #     rightWeight += 10
            # print("New State: " + str(tup) + " " + str(forwardWeight) + " " + str(leftWeight) + " " + str(rightWeight))
            ########################################

            ### Less Naeve ###
            # if state[1] not in ['TC', 'C'] and state[3] not in ['TF','TC']:
            #     forwardWeight += 5
            #     if state[3] == 'M':
            #         forwardWeight += 5
            # if state[0] == 'C':
            #     leftWeight -= 1
            #     rightWeight += 1
            # if state[2] == 'C':
            #     leftWeight += 3
            #     rightWeight -= 3
            #     forwardWeight -= 2
            # if state[3] in ['F', 'TF']:
            #     rightWeight += 1
            #     # if state[3] == 'TF':
            #         # leftWeight -= 2
            # if state[4] in ['U']:
            #     forwardWeight += 3
            #     leftWeight += 1
            ##################

            self.table[tup] = [forwardWeight, leftWeight, rightWeight]
            self.initialTable[tup] = [forwardWeight, leftWeight, rightWeight]

        actionWeights = self.table[tup]
        indexMax = -1
        maxWeight = -99999999999
        for i in range(len(actionWeights)):
            if actionWeights[i] > maxWeight:
                indexMax = i
                maxWeight = actionWeights[i]
        return actions[indexMax], indexMax

"""
Enviornment Setup
"""
class wallFollowEnv():
    def __init__(self):
        # General variables defining the environment
        self.robot = tritonRobot()

        # Initializing
        self.current_step = -1
        self.current_episode = -1
        self.good_policy_step = 0

        # Define actions
        self.actionSpace = ACTIONS
        self.possibleActions = ['F','L','R','S']

        self.previousStucks = 0
        # self.previousStuckFront = 0
        # self.previousStuckLeft = 0
        # self.previousStuckRight = 0

        # Store memory of the agents actions
        # These will be the state change the agent made during one runthrough of the algorithm
        self.action_episode_memory = []
        self.state_episode_memory = []
        self.previousEpisodeScans = []
        self.currentEpisodeScans = []

        rospy.wait_for_service('/gazebo/get_model_state')
        self.getPosition = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.previousPosition = ModelState()

    def _step_env(self, action='S'):
        self.robot.step(action)
        self.current_step += 1
        observation = self._get_observation()
        done = self._isDone(observation)
        reward = self._get_reward()

        info = {}
        #Test info
        self.action_episode_memory.append(action)
        self.state_episode_memory.append(observation)
        self.currentEpisodeScans.append(self.robot.ranges)

        return observation, reward, done, info
        # return observation

    def _get_reward(self):
        return self.robot.rewardFunction()

    def _get_observation(self):
        # print("GET OBSERVATION")
        obs = self.robot.get_observation()
        return obs

    def _isDone(self, obs):
        if self.isStuck(obs):
            print("Stuck")
            # if self.current_step < 10:
            #     print(obs)
            #     print('Front Stuck: ', self.previousStuckFront)
            #     print('Right Stuck: ', self.previousStuckRight)
            #     with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/previousScans.txt', 'w') as fp:
            #         json.dump(self.previousEpisodeScans, fp)
            #     with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/currentScans.txt', 'w') as fp:
            #         json.dump(self.currentEpisodeScans, fp)
            #     exit()
            return True
        if self.current_step >= 10000:
            print("Full Episode")
            return True
        if self.good_policy_step >= 1000:
            print("Followed Wall Success")
            return True

        return False

    def isStuck(self, obs):
        # Is the Step Good
        if obs[1] not in ['TC','C'] and obs[3] == 'M':
            self.good_policy_step += 1
        else:
            self.good_policy_step = 0

        currentPosition = self.getPosition('triton_lidar','world')

        if np.isclose(currentPosition.pose.position.x, self.previousPosition.pose.position.x, atol=.01) and np.isclose(currentPosition.pose.position.y, self.previousPosition.pose.position.y, atol=.01):
            self.previousStucks += 1
        else:
            self.previousStucks = 0

        if self.previousStucks > 3:
            return True

        self.previousPosition = currentPosition

        return False

    def reset(self):
        #Reset the World
        rospy.wait_for_service('/gazebo/reset_world')
        resetSim = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resetSim()

        #Delete the Robot Instance in Attempt to Clear Buffer
        del self.robot
        self.robot = tritonRobot()

        #Choose a random Spawn
        rospy.wait_for_service('/gazebo/set_model_state')
        setPosition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        spawn = ModelState()
        spawn.model_name = 'triton_lidar'
        spawn.pose.position.x = random.randint(-4,3) + .5
        spawn.pose.position.y = random.randint(-4,3) + .5
        setPosition(spawn)

        self.current_step = -1
        self.previousStuckFront = 0
        self.previousStuckRight = 0
        # self.previousStuckLeft = 0
        self.good_policy_step = 0
        self.current_episode += 1
        self.previousEpisodeScans = self.currentEpisodeScans
        self.currentEpisodeScans = []
        time.sleep(2)
        obs = self._step_env()[0] #CHANGED: This could be an issue should this be an enviornment step?
        return obs

"""
Agent Definition
"""
class tritonRobot:
    def __init__(self):
        rospy.init_node('triton')
        self.pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=2)
        # self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        # self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.scan = LaserScan()

        self.rate = rospy.Rate(10)
        self.ranges = []
        self.previousRanges = []
        self.state = ['','','','','']
        self.previousState = ['','','','','']
        self.discreteDistances = ['TTC','TC','C','M','F','TF']
        self.discreteOrientations = ['U','P','A','MA'] # Undefined, Parallel, Approaching, Moving Away

    def callback(self, msg):
        # rospy.wait_for_service('/gazebo/pause_physics')
        # try:
        #     self.pause()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/pause_physics service call failed")
        # print("Entering Callback")
        self.ranges = msg.ranges

        #Interpret the LaserScan ranges
        front = 9999
        rightFront = 9999
        right = 9999
        left = 9999

        #Getting left min distance
        for i in range(60,121):
            if self.ranges[i] < left:
                left = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if left <= .25:
            self.state[0] = self.discreteDistances[0]
        elif left <= .5:
            self.state[0] = self.discreteDistances[2]
        elif left > .5:
            self.state[0] = self.discreteDistances[4]
        else:
            print("Error with left values")


        #Getting front min distance
        for i in range(-30,31):
            if self.ranges[i] < front:
                front = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if front <= .25:
            self.state[1] = self.discreteDistances[0]
        elif front < .5:
            self.state[1] = self.discreteDistances[1]
        elif front < .6:
            self.state[1] = self.discreteDistances[2]
        elif front <= 1.2:
            self.state[1] = self.discreteDistances[3]
        elif front > 1.2:
            self.state[1] = self.discreteDistances[4]
        else:
            print("Error with front values")


        #Getting right min distance
        for i in range(-120,-59):
            if self.ranges[i] < right:
                right = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if right <= .25:
            self.state[3] = self.discreteDistances[0]
        elif right < .5:
            self.state[3] = self.discreteDistances[1]
        elif right < .6:
            self.state[3] = self.discreteDistances[2]
        elif right <= .8:
            self.state[3] = self.discreteDistances[3]
        elif right <= 1.2:
            self.state[3] = self.discreteDistances[4]
        elif right > 1.2:
            self.state[3] = self.discreteDistances[5] #####FIXME Took out TF  [4]
        else:
            print("Error with right values")


        #Getting rightFront min distance
        for i in range(-60,-29):
            if self.ranges[i] < rightFront:
                rightFront = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if rightFront <= .25:
            self.state[2] = self.discreteDistances[0]
        elif rightFront <= 1.2:
            self.state[2] = self.discreteDistances[2]
        elif rightFront > 1.2:
            self.state[2] = self.discreteDistances[4]
        else:
            print("Error with rightFront values")

        #Determine the Orientation
        if len(self.previousRanges) == 0 or (self.state[3] == 'TF' and self.previousState[3] == 'TF'):
            self.state[4] = self.discreteOrientations[0]
        elif self.state[3] == self.previousState[3]:
            self.state[4] = self.discreteOrientations[1]
        else:
            if self.state[3] in ['TF','F'] and self.previousState[3] in ['F','M','C']:
                self.state[4] = self.discreteOrientations[3]
            elif self.state[3] in ['C','TC'] and self.previousState[3] in ['F','M','C']:
                self.state[4] = self.discreteOrientations[2]


        self.previousRanges = self.ranges
        self.previousState = self.state

    def get_observation(self):
        # print("Robot Observation")
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        # self.scan = rospy.wait_for_message('/scan', LaserScan, timeout=10)


        # self.rate.sleep()
        # print("Exiting Observation")
        return self.state

    def step(self, action='S'):
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # try:
        #     self.unpause()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/unpause_physics service call failed")


        geoValues = ACTIONS[action]

        msg = Pose2D()
        msg.x = geoValues[0]
        msg.y = 0
        msg.theta = geoValues[1]

        backwardsException = 0

        self.pub.publish(msg)
        try:
            self.rate.sleep()
        except rospy.ROSTimeMovedBackwardsException as e:
            # pass
            rate._reset
            backwardsException += 1
            # print("Time Backwards")

        return

    """
    Reward functions
    """
    def rewardFunction(self):
        ## TODO: Create Reward Function
        left = self.state[0]
        front = self.state[1]
        rightFront = self.state[2]
        right = self.state[3]
        orientation = self.state[4]

        reward = 0

        # Basic reward Function
        # if right in ['TF', 'TC']:
        #     reward -= 1
        # # if left == 'C':
        # #     reward -= 2
        # if right in ['F', 'C']:
        #     reward += 2
        # if front == 'TC':
        #     reward -= 7
        # if front == 'C':
        #     reward -= 3
        # if right == 'M' and front != 'TC':
        #     reward += 10
        # if rightFront == 'C':
        #     reward -= 2
        # if right == 'M' and orientation == 'P' and front != 'TC':
        #     reward += 15
        #######################
        # elif right == 'M':
            # reward += 5

        #### New Attempt ####
        if left == 'C' or front == 'TC' or right in ['TC','TF']:
            reward = -1
        if right in ['C','M','F'] and front != 'TC':
            reward = 1
        if right in ['C','M','F'] and front != 'TC':
            reward += .5

        return reward

"""
Training simulation
"""
# def train():

"""
Dictionary Cleaner
"""
def stringify_keys(d):
    """Convert a dict's keys to strings if they are not."""
    for key in d.keys():

        # check inner dict
        if isinstance(d[key], dict):
            value = stringify_keys(d[key])
        else:
            value = d[key]

        # convert nonstring to string if needed
        if not isinstance(key, str):
            try:
                d[str(key)] = value
            except Exception:
                try:
                    d[repr(key)] = value
                except Exception:
                    raise

            # delete old key
            del d[key]
    return d

"""
Main
"""
if __name__ == '__main__':
    time.sleep(2)
    print("Opening Gazebo")
    time.sleep(5)
    while True:
        mode = input("To train enter 0 if model is trained enter 1: ")
        if mode == 0:
            print("Beginning Training Simulation")
            env = wallFollowEnv()
            qTable = qTable()

            EPS = .9
            ALPHA = .2
            GAMMA = .8
            DECAY = .985

            numGames = 150
            totalRewards = np.zeros(numGames)

            for i in range(numGames):
                env.current_episode = i
                if i % 5 == 0:
                    print('starting game', i)

                eps = EPS*(DECAY**i)
                done = False
                epRewards = 0
                epSteps = 0
                observation = env.reset()

                while not done:
                    rand = np.random.random()
                    ## IS THIS SARSA OR TD??
                    if rand < eps:
                        action, actionIndex = qTable.maxAction(observation, env.possibleActions)
                    else:
                        actionIndex = random.randint(0,len(env.possibleActions)-2) # The minus two will prevent Stop action from bieng chosen
                        action = env.possibleActions[actionIndex]

                    # #Always Go Forward
                    # actionIndex = 2
                    # action = env.possibleActions[actionIndex]

                    # print("This is the action: " + action)
                    observation_, reward, done, info = env._step_env(action)
                    epRewards += reward
                    epSteps += 1

                    action_, actionIndex_ = qTable.maxAction(observation_, env.possibleActions)



                    #Update Function
                    oldValue = qTable.table[tuple(observation)][actionIndex]
                    maxValue = qTable.table[tuple(observation_)][actionIndex_]

                    # print(type(oldValue))
                    # print(type(maxValue))

                    qTable.table[tuple(observation)][actionIndex] =  oldValue + ALPHA*(reward + GAMMA*maxValue - oldValue)

                    observation = observation_
                # print("Episode Steps: " + str(epSteps))

                # Stop the Robot
                env.robot.step(env.possibleActions[3])
                totalRewards[i] = epRewards
                # print("Next Episode")


            #Save the QTable Training
            cleanFinalTable = stringify_keys(qTable.table)
            # cleanInitialTable = stringify_keys(qTable.initialTable)

            with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/qFtable.txt', 'w') as fp:
                json.dump(cleanFinalTable, fp)
            # with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/qITable.txt', 'w') as fp:
            #     json.dump(cleanInitialTable, fp)
            # with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/ActionInfo.txt', 'w') as fp:
            #     json.dump(env.action_episode_memory, fp)
            # with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/StateInfo.txt', 'w') as fp:
            #     json.dump(env.state_episode_memory, fp)

            # Plot the Episode epRewards
            plt.plot(totalRewards)
            plt.ylabel('Episode Rewards')
            plt.show()


            break
        if mode == 1:
            print("Already Trained")
            with open('qFTable.txt', 'r') as fp:
                data = json.load(fp)
            print(data) #####THE KEYS ARE CHANGED TO UNICODE STRINGS INSTEAD OF SETS MAY CAUSE A PROBLEM#####
            break
        else:
            print("There is a mistake")


"""
The agent makes a decision in the environment

Parameters
----------
action_state: int - Represents which light configuration the agent took

Returns
-------
ob, reward, episode_over, info : tuple
    ob (object) :
        an environment-specific object representing your observation of
        the environment.
    reward (float) :
        amount of reward achieved by the previous action. The scale
        varies between environments, but the goal is always to increase
        your total reward.
    episode_over (bool) :
        whether it's time to reset the environment again. Most (but not
        all) tasks are divided up into well-defined episodes, and done
        being True indicates the episode has terminated. (For example,
        perhaps the pole tipped too far, or you lost your last life.)
    info (dict) :
         diagnostic information useful for debugging. It can sometimes
         be useful for learning (for example, it might contain the raw
         probabilities behind the environment's last state change).
         However, official evaluations of your agent are not allowed to
         use this for learning.
"""
# def isStuck(self, obs):
#     # Is the Step Good
#     if obs[1] not in ['TC','C'] and obs[3] == 'M':
#         self.good_policy_step += 1
#     else:
#         self.good_policy_step = 0
#
#     # Is the Robot Stuck Front
#     if obs[1] == 'TTC':
#         # print("Stuck")
#         self.previousStuckFront+=1
#         if self.previousStuckFront > 7:
#             # print(self.robot.ranges)
#             return True
#     if obs[1] != 'TTC':
#         # if self.previousStucks != 0:
#             # print("Unstuck")
#         self.previousStuckFront = 0
#
#     # Is the Robot Stuck Left
#     # if obs[0] == 'C':
#     #     self.previousStuckLeft+=1
#     #     if self.previousStuckLeft > 3:
#     #         return True
#     # if obs[0] != 'C':
#     #     self.previousStuckLeft = 0
#
#     # Is the Robot Stuck Right
#     if obs[3] == 'TTC':
#         self.previousStuckRight+=1
#         if self.previousStuckRight > 7:
#             return True
#     if obs[3] != 'TTC':
#         self.previousStuckRight = 0
#
#
#     return False
