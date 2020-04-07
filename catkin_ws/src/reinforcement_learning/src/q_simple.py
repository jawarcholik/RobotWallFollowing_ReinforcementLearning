#!/usr/bin/python2

import sys
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import random
import time
import json
import matplotlib.pyplot as plt
from scipy import stats # Used for linear regression

"""
Action spaces
"""
ACTIONS = {
    'F': [.3,0],
    'L': [.3,math.pi/4],
    'R': [.3,-math.pi/4],
    'S': [0,0]
}


"""
Q-Table
"""
class qTable():
    def __init__(self):
        self.table = dict()

    def maxAction(self, state, actions):
        forwardWeight = 0
        leftWeight = 0
        rightWeight = 0

        tup = tuple(state)

        left = state[0]
        front = state[1]
        rightFront = state[2]
        right = state[3]
        orientation = state[4]

        if tup not in self.table:
            ### Prior Knowledge Rules ###

            # Don't Turn if all sensors are far
            if left == 'F' and front in ['TF','F'] and right in ['TF','F']:
                forwardWeight += 5
                leftWeight -= 5
                rightWeight -= 5

            # Turn Right if FrontRight is Far and Right is not
            if rightFront == 'F' and right in ['M','C']:
                rightWeight += 5
                forwardWeight -= 5
                leftWeight -= 5

            # Turn Left if Front is Close and Left is not Near wall
            if front in ['M','C','TC'] and left != 'C':
                leftWeight += 5
                forwardWeight -= 5
                rightWeight -= 5
            elif front in ['M','C','TC']:
                leftWeight -= 5
                forwardWeight -= 5
                rightWeight += 5

            self.table[tup] = [forwardWeight, leftWeight, rightWeight]

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
        rospy.wait_for_service('/gazebo/get_model_state')
        self.getPosition = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.previousPosition = ModelState()
        random.seed()

        # Initializing
        self.current_step = -1
        self.current_episode = -1
        self.good_policy_step = 0

        # Define actions
        self.actionSpace = ACTIONS
        self.possibleActions = ['F','L','R','S']

        self.previousStucks = 0

        # Store memory of the agents actions
        # These will be the state change the agent made during one runthrough of the algorithm
        self.action_episode_memory = ["","","","","","","","","",""]
        # self.state_episode_memory = []
        # self.previousEpisodeScans = []
        # self.currentEpisodeScans = []


    def _step_env(self, action='S'):
        # Track the Actions
        self.action_episode_memory.append(action)
        info = self.action_episode_memory[-100:]

        self.robot.step(action, info)
        self.robot.do_callback = True
        self.current_step += 1
        time.sleep(.5)
        observation = self._get_observation()
        done = self._isDone(observation)
        reward = self._get_reward(done)

        #Test info
        # self.state_episode_memory.append(observation)
        # self.currentEpisodeScans.append(self.robot.ranges)


        return observation, reward, done, info

    def _get_reward(self, done):
        return self.robot.rewardFunction(done)

    def _get_observation(self):
        obs = self.robot.get_observation()
        return obs

    def _isDone(self, obs):
        if self.isStuck(obs):
            if self.current_step < 5:
                print("Short Episode")
            return True
        if self.current_step >= 10000:
            print("Full Episode")
            return True
        if self.good_policy_step >= 1000:
            print("Followed Wall Success")
            return True

        return False

    def isStuck(self, obs):
        # Is the Step Good CHECK THIS WORKS
        if obs[1] not in ['TC','C'] and obs[3] == 'M':
            self.good_policy_step += 1
        else:
            self.good_policy_step = 0

        currentPosition = self.getPosition('triton_lidar','world')

        if (np.isclose(currentPosition.pose.position.x, self.previousPosition.pose.position.x, atol=.1) and np.isclose(currentPosition.pose.position.y, self.previousPosition.pose.position.y, atol=.1)):
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
        # del self.robot
        # self.robot = tritonRobot()

        #Choose a random Spawn
        rospy.wait_for_service('/gazebo/set_model_state')
        setPosition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        spawn = ModelState()
        spawn.model_name = 'triton_lidar'
        # spawn.pose.position.x = random.randint(-4,3) + .5
        # spawn.pose.position.y = random.randint(-4,3) + .5
        spawn.pose.position.x = 0
        spawn.pose.position.y = -1.5

        setPosition(spawn)

        ## CHECK THIS SECTION CLOSER
        self.current_step = -1
        self.previousStucks = 0
        self.good_policy_step = 0
        self.current_episode += 1
        self.action_episode_memory = ["","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","",""]
        # self.previousEpisodeScans = self.currentEpisodeScans
        # self.currentEpisodeScans = []
        time.sleep(2)
        obs = self._step_env()[0] #CHANGED: This could be an issue should this be an enviornment step?
        return obs


"""
Agent Definition
"""
class tritonRobot:
    def __init__(self):
        rospy.init_node('triton')
        self.pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        # self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.scan = LaserScan()

        self.do_callback = False

        self.rate = rospy.Rate(10)
        self.ranges = []
        self.previousRanges = []

        self.state = ['','','','','']
        self.previousState = ['','','','','']
        self.twoStatesAgo = ['','','','','']

        self.discreteDistances = ['TC','C','M','F','TF']
        self.discreteOrientations = ['U','P','A','MA'] # Undefined, Parallel, Approaching, Moving Away

        self.state_memory = []

    def callback(self, msg):
        if not self.do_callback:
            return
        else:
            self.do_callback = False

        # print(len(msg.ranges))
        self.ranges = msg.ranges

        #Interpret the LaserScan ranges
        front = 9999
        rightFront = 9999
        right = 9999
        left = 9999

        elemCount = 0

        totalLeft = 0
        totalFront = 0
        totalRightFront = 0
        totalRight = 0

        #Getting left min distance
        for i in range(60,121):
            # Average Distance
            totalLeft += self.ranges[i]
            elemCount += 1

        left = totalLeft/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if left <= .5:
            self.state[0] = self.discreteDistances[1]
        elif left > .5:
            self.state[0] = self.discreteDistances[3]
        else:
            print("Error with left values")


        elemCount = 0
        #Getting front min distance
        for i in range(-30,31):
            # Average Distance
            totalFront += self.ranges[i]
            elemCount += 1

        front = totalFront/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if front < .5:
            self.state[1] = self.discreteDistances[0]
        elif front < .6:
            self.state[1] = self.discreteDistances[1]
        elif front < 1.2:
            self.state[1] = self.discreteDistances[2]
        elif front >= 1.2:
            self.state[1] = self.discreteDistances[3]
        else:
            print("Error with front values")


        elemCount = 0
        #Getting rightFront min distance
        for i in range(-60,-29):
            # Average Distance
            totalRightFront += self.ranges[i]
            elemCount += 1

        rightFront = totalRightFront/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if rightFront <= 1.0:
            self.state[2] = self.discreteDistances[1]
        elif rightFront > 1.0:
            self.state[2] = self.discreteDistances[3]
        else:
            print("Error with rightFront values")


        elemCount = 0
        #Getting right min distance
        for i in range(-120,-59):
            # Average Distance
            totalRight += self.ranges[i]
            elemCount += 1

        right = totalRight/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if right < .5:
            self.state[3] = self.discreteDistances[0]
        elif right <= .6:
            self.state[3] = self.discreteDistances[1]
        elif right <= .8:
            self.state[3] = self.discreteDistances[2]
        elif right <= 1.2:
            self.state[3] = self.discreteDistances[3]
        elif right > 1.2:
            self.state[3] = self.discreteDistances[4]
        else:
            print("Error with right values")


        #Determine the Orientation THIS MAY NEED TO CHANGE
        if self.state[1] in ['C','TC'] or self.state[3] in ['TF','TC']:
            self.state[4] = self.discreteOrientations[0]
        else:
            y = np.array(self.ranges[-60:-120:-1])
            x = np.array([i for i in range(-30, 30, 1)])

            # print(x.shape)
            # print(y.shape)

            slope = stats.linregress(x,y)[0]

            if slope > .0075:
                self.state[4] = self.discreteOrientations[2]
            elif slope < -.0075:
                self.state[4] = self.discreteOrientations[3]
            else:
                self.state[4] = self.discreteOrientations[1]




    def get_observation(self):
        # print("Robot Observation")
        # self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        return self.state

    def step(self, action='S', info=[]):
        self.state_memory = info

        geoValues = ACTIONS[action]

        msg = Pose2D()
        msg.x = geoValues[0]
        msg.y = 0 #geoValues[0]
        msg.theta = geoValues[1]

        backwardsException = 0

        self.pub.publish(msg)
        self.rate.sleep()

        return

    def rewardFunction(self, isDone):
        ## TODO: Create Reward Function

        left = self.state[0]
        front = self.state[1]
        rightFront = self.state[2]
        right = self.state[3]
        orientation = self.state[4]

        reward = 0

        previousRight = ''

        ########### Late Attempt ##########
        # Punish Being in Bad Sensor Ranges
        if right in ['TC','TF'] or front in ['TC'] or left in ['C']:
            reward -= 1
        # if front in ['TC','C']:
        #     reward -= 1
        # if left in ['C']:
        #     reward -= 1

        # # Reward Not Running into Wall
        # if isDone:
        #     reward -= 5
        # # else:
        # #     reward += 1
        #
        # Reward Wall Followed
        if right in ['M']:
            # print("Following")
            reward += 1
            if orientation in ['P']:
                reward += 1
        #
        # # Punish not following a wall SHOULD PREVENT CIRCLES
        # if orientation in ['U'] and right not in ['TC']:
        #     # print("Stupid")
        #     reward -= 1
        #
        # # Reward if moving towards a good distance
        # if orientation in ['A'] and right in ['M','F']:
        #     reward += 1
        # elif orientation in ['MA'] and right in ['C','M']:
        #     reward += 1
        # elif orientation in ['MA','A']:
        #     reward -= 1

        # # Punish Turning in Circles
        # if len(self.state_memory) != 0:
        #     rightTurns = self.state_memory.count('R')
        #     leftTurns = self.state_memory.count('L')
        #     if leftTurns > 75 or rightTurns > 75:
        #         print("Circle Warning")
        #         reward -= 2



        return reward


"""
Training simulation
"""
#Train the Model
def train():
    print("Beginning Training Simulation")
    env = wallFollowEnv()
    qTab = qTable()

    EPS = .9
    ALPHA = .2
    GAMMA = .8
    DECAY = .985

    numGames = 150
    totalRewards = np.zeros(numGames)
    totalMaxChoices = np.zeros(numGames)

    for i in range(numGames):
        env.current_episode = i
        if i % 5 == 0:
            print('starting game', i)

        eps = EPS*(DECAY**i)
        done = False
        epRewards = 0
        epMaxChoices = 0
        epSteps = 0
        observation = env.reset()

        while not done:
            rand = np.random.random()

            if rand > eps:
                action, actionIndex = qTab.maxAction(observation, env.possibleActions)
                epMaxChoices += 1
            else:
                actionIndex = random.randint(0,len(env.possibleActions)-2) # The minus two will prevent Stop action from bieng chosen
                action = env.possibleActions[actionIndex]

            # #Always Go Forward
            # actionIndex = len(env.possibleActions)-2
            # action = env.possibleActions[actionIndex]

            observation_, reward, done, info = env._step_env(action)
            epRewards += reward
            epSteps += 1

            action_, actionIndex_ = qTab.maxAction(observation_, env.possibleActions)

            #Update Function
            oldValue = qTab.table[tuple(observation)][actionIndex]
            maxValue = qTab.table[tuple(observation_)][actionIndex_]


            qTab.table[tuple(observation)][actionIndex] =  oldValue + ALPHA*(reward + GAMMA*maxValue - oldValue)

            observation = observation_

            #Display Step information
            print("State: " + str(observation))
            print("Reward: " + str(reward))
            # print("State Memory: " + str(info))

        # Stop the Robot
        env.robot.step(env.possibleActions[len(env.possibleActions)-1])
        totalRewards[i] = epRewards
        totalMaxChoices[i] = epMaxChoices


    #Save the QTable Training
    cleanFinalTable = stringify_keys(qTab.table)

    filepath = raw_input("Enter filepath to save Q-Table: ")

    with open(filepath, 'w') as fp:
        json.dump(cleanFinalTable, fp)

    # Plot the Episode epRewards
    plt.subplot(2, 2, 1)
    plt.plot(totalRewards)
    plt.ylabel('Episode Rewards')

    plt.subplot(2, 2, 2)
    plt.plot(totalMaxChoices)
    plt.ylabel('Number of Best Choices')

    plt.show()



#Test the Reward Function
def test():
    print("Beginning Testing Simulation")
    env = wallFollowEnv()
    qTab = qTable()

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
            act = raw_input("Choose an Action [L, R, F]: ")

            actionIndex = env.possibleActions.index(act)

            observation_, reward, done, info = env._step_env(act)
            epRewards += reward
            epSteps += 1

            action_, actionIndex_ = qTab.maxAction(observation_, env.possibleActions)

            # Update Function
            oldValue = qTab.table[tuple(observation)][actionIndex]
            maxValue = qTab.table[tuple(observation_)][actionIndex_]


            qTab.table[tuple(observation)][actionIndex] =  oldValue + ALPHA*(reward + GAMMA*maxValue - oldValue)

            observation = observation_

            #Stop Robot after each Step
            o = env._step_env()[0]

            #Display Step information
            print("State: " + str(observation))
            print("Reward: " + str(reward))
            print("State Memory: " + str(info))

        # Stop the Robot
        env.robot.step(env.possibleActions[len(env.possibleActions)-1])
        totalRewards[i] = epRewards


    #Save the QTable Training
    cleanFinalTable = stringify_keys(qTab.table)

    with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/qLateAttempttable2.txt', 'w') as fp:
        json.dump(cleanFinalTable, fp)

    # Plot the Episode epRewards
    plt.plot(totalRewards)
    plt.ylabel('Episode Rewards')
    plt.show()

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
            train()
            # test()

            break
        if mode == 1:
            print("Already Trained")
            filepath = raw_input("Enter filepath to the QTable File you would like to use: ")
            with open(filepath, 'r') as fp:
                data = json.load(fp)
            # print(data) #####THE KEYS ARE CHANGED TO UNICODE STRINGS INSTEAD OF SETS MAY CAUSE A PROBLEM#####

            env = wallFollowEnv()
            qTable = qTable()

            observation = env.reset()

            done = False

            while not done:
                action, actionIndex = qTable.maxAction(observation, env.possibleActions)

                observation_, reward, done, info = env._step_env(action)

                observation = observation_

            break
        else:
            print("There is a mistake")
