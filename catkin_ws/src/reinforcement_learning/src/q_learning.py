#!/usr/bin/python2

import sys
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import math
import numpy as np
import random
import time

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
    'F': [.3,0],
    'L': [.2,-math.pi/4],
    'R': [.2,math.pi/4],
    'S': [0,0]
}

"""
Q-Table
"""
class qTable():
    def __init__(self):
        self.table = {}

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
        if tup  not in self.table:
            #### Naeve Implementation ####
            if state[1] != 'TC':
                forwardWeight += 10
            if state[1] == 'TC' or state[1] == 'C':
                forwardWeight -= 5
            if state[1] == 'M':
                leftWeight += 10
            if state[0] != 'C':
                leftWeight += 5
            if state[2] == 'C':
                leftWeight += 5
                rightWeight -= 5
                forwardWeight -= 2
            if state[3] == 'TC':
                leftWeight += 10
            if state[3] == 'F' or state[3] == 'TF':
                rightWeight += 10
            print("New State: " + str(tup) + " " + str(forwardWeight) + " " + str(leftWeight) + " " + str(rightWeight))
            ########################################
            self.table[tup] = [forwardWeight, leftWeight, rightWeight]

        actionWeights = self.table[tup]
        indexMax = -1
        maxWeight = -9999
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

        # Define actions
        self.actionSpace = ACTIONS
        self.possibleActions = ['F','L','R','S']

        self.previousStucks = 0

        # Define observations
        # self.observation_space = spaces.Discrete(self.num_lanes ** (self.lane_size + 1))

        # Store memory of the agents actions
        # These will be the state change the agent made during one runthrough of the algorithm
        self.action_episode_memory = []

    def step(self, action=0):
        self.robot.step(action)
        self.current_step += 1
        observation = self._get_observation()
        done = self.isStuck(observation)
        reward = self._get_reward()
        info = {}
        return observation, reward, done, info
        # return observation

    def _get_reward(self):
        #TODO
        return 0

    def _get_observation(self):
        # print("GET OBSERVATION")
        obs = self.robot.get_observation()
        return obs

    def isStuck(self, obs):
        if obs[1] == 'TC':
            print("Stuck")
            self.previousStucks+=1
            if self.previousStucks > 10:
                print(self.robot.ranges)
                return True
        if obs[1] != 'TC':
            if self.previousStucks != 0:
                print("Unstuck")
            self.previousStucks = 0
        return False

    def reset(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        resetSim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        resetSim()
        self.current_step = -1
        self.previousStucks = 0
        obs = self._get_observation()
        return obs

"""
Agent Definition
"""
class tritonRobot:
    def __init__(self):
        rospy.init_node('triton')
        self.pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=2)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.scan = LaserScan()

        self.rate = rospy.Rate(5)
        self.ranges = []
        self.state = ['','','','']
        self.discreteDistances = ['TC','C','M','F','TF']

    def callback(self, msg):
        # print("Entering Callback")
        self.ranges = msg.ranges
        # self.sub.unregister()

        #Interpret the LaserScan ranges
        front = 9999
        rightFront = 9999
        right = 9999
        left = 9999

        #Getting left min distance
        for i in range(30):
            if self.ranges[i] < left:
                left = self.ranges[i]

        for i in range(329,359):
            if self.ranges[i] < left:
                left = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if left <= .5:
            self.state[0] = self.discreteDistances[1]
        elif left > .5:
            self.state[0] = self.discreteDistances[3]
        else:
            print("Error with left values")


        #Getting front min distance
        for i in range(60,121):
            if self.ranges[i] < front:
                front = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if front < .5:
            self.state[1] = self.discreteDistances[0]
        elif front < .6:
            self.state[1] = self.discreteDistances[1]
        elif front <= 1.2:
            self.state[1] = self.discreteDistances[2]
        elif front > 1.2:
            self.state[1] = self.discreteDistances[3]
        else:
            print("Error with front values")


        #Getting right min distance
        for i in range(150,210):
            if self.ranges[i] < right:
                right = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if right < .5:
            self.state[3] = self.discreteDistances[0]
        elif right < .6:
            self.state[3] = self.discreteDistances[1]
        elif right <= .8:
            self.state[3] = self.discreteDistances[2]
        elif right <= 1.2:
            self.state[3] = self.discreteDistances[3]
        elif right > 1.2:
            self.state[3] = self.discreteDistances[4] #####FIXME Took out TF  [4]
        else:
            print("Error with right values")


        #Getting rightFront min distance
        for i in range(121,150):
            if self.ranges[i] < rightFront:
                rightFront = self.ranges[i]

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if rightFront <= 1.2:
            self.state[2] = self.discreteDistances[1]
        elif rightFront > 1.2:
            self.state[2] = self.discreteDistances[3]
        else:
            print("Error with rightFront values")

    def get_observation(self):
        # print("Robot Observation")
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        # self.rate.sleep()
        # print("Exiting Observation")
        return self.state

    def step(self, action=0):
        geoValues = ACTIONS[action]

        msg = Pose2D()
        msg.x = 0
        msg.y = geoValues[0]
        msg.theta = geoValues[1]

        self.pub.publish(msg)
        try:
            self.rate.sleep()
        except rospy.ROSTimeMovedBackwardsException as e:
            print("Time Backwards")

        return

    """
    Reward functions
    """
    def rewardFunction(self):
        ## TODO: Create Reward Function
        return

"""
Main
"""
if __name__ == '__main__':
    time.sleep(2)
    print("Opening Gazebo")
    time.sleep(5)
    print("Beginning Simulation")
    env = wallFollowEnv()
    qTable = qTable()

    EPS = .1
    ALPHA = .2
    GAMMA = .8

    numGames = 2
    totalRewards = np.zeros(numGames)

    for i in range(numGames):
        if i % 100 == 1:
            print('starting game', i)

        done = False
        epRewards = 0
        observation = env.reset()

        while not done:
            rand = np.random.random()
            ## IS THIS SARSA OR TD??
            if rand < 1-EPS:
                action, actionIndex = qTable.maxAction(observation, env.possibleActions)
            else:
                actionIndex = random.randint(0,len(env.possibleActions)-2) # The minus two will prevent Stop action from bieng chosen
                action = env.possibleActions[actionIndex]

            # #Always Go Forward
            # actionIndex = 0
            # action = env.possibleActions[actionIndex]

            # print("This is the action: " + action)
            observation_, reward, done, info = env.step(action)

            action_, actionIndex_ = qTable.maxAction(observation_, env.possibleActions)

            #### Update Q Value in Deliverable 2 ####
            qTable[tuple(observation)][actionIndex] = qTable[tuple(observation)][actionIndex] + ALPHA*(reward + \
                    GAMMA*qTable[tuple(observation_)][actionIndex_] - qTable[tuple(observation)][actionIndex])

            observation = observation_

        #Stop the Robot
        env.step(env.possibleActions[3])
        print("Next Episode")
        time.sleep(2)




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
