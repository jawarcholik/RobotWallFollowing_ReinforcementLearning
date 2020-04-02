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
# ACTIONS = {
#     'F': [.2,0],
#     'L': [.2,math.pi/4],
#     'R': [.2,-math.pi/4],
#     'S': [0,0]
# }

ACTIONS = {
    'HL': [.2,np.deg2rad(40)],
    'ML': [.2,np.deg2rad(20)],
    'L': [.2,np.deg2rad(10)],
    'SL': [.2,np.deg2rad(.3)],
    'F': [.2,0],
    'SR': [.2,np.deg2rad(-.3)],
    'R': [.2,np.deg2rad(-10)],
    'MR': [.2,np.deg2rad(-20)],
    'HR': [.2,np.deg2rad(-40)],
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
        # self.initialTable = dict()

    def __getitem__(self,i): #TODO: Is This Function Ever Used?????????????????????
        if i in self.table:
            return self.table[i]

        print("NEW KEY")
        self.table[i] = [0, 0, 0]
        return self.table[i]

    def maxAction(self, state, actions):
        # forwardWeight = 0
        # leftWeight = 0
        # rightWeight = 0

        hardLeft = 0
        mediumLeft = 0
        normLeft = 0
        slightLeft = 0
        forward = 0
        slightRight = 0
        normRight = 0
        mediumRight = 0
        hardRight = 0

        tup = tuple(state)

        leftBack = state[0]
        left = state[1]
        leftFront = state[2]
        front = state[3]
        rightFront = state[4]
        right = state[5]
        rightBack = state[6]
        orientation = state[7]

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

            ### Latest Attempt ###
            if front in ['M','C','TC']:
                forward -= 10
                if left in ['C','TC']:
                    hardRight += 5
                    mediumRight += 4
                    normRight += 3
                    slightRight += 2
                else:
                    hardLeft += 5
                    mediumLeft += 4
                    normLeft += 3
                    slightLeft += 2
            if left in ['C','TC','TTC']:
                hardRight += 5
                mediumRight += 4
                normRight += 3
                slightRight += 2
                hardLeft -= 5
                mediumLeft -= 4
                normLeft -= 3
                slightLeft -= 2
            if right in ['TC','TTC']:
                hardLeft += 5
                mediumLeft += 4
                normLeft += 3
                slightLeft += 2
                hardRight -= 5
                mediumRight -= 4
                normRight -= 3
                slightRight -= 2
            if rightBack in ['C','M'] and (right not in ['C','M'] or rightFront not in ['C','M']):
                hardRight += 10
                mediumRight += 8
                normRight += 4
                slightRight += 2
            if leftFront in ['TTC', 'C'] and rightFront not in ['TTC','C']:
                hardRight += 5
                mediumRight += 4
                normRight += 3
                slightRight += 2
            if rightFront in ['TTC', 'C'] and leftFront not in ['TTC','C']:
                hardLeft += 5
                mediumLeft += 4
                normLeft += 3
                slightLeft += 2

            # self.table[tup] = [forwardWeight, leftWeight, rightWeight]
            # self.initialTable[tup] = [forwardWeight, leftWeight, rightWeight]

            self.table[tup] = [hardLeft, mediumLeft, normLeft, slightLeft, forward, slightRight, normRight, mediumRight, hardRight]

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
        # self.possibleActions = ['F','L','R','S']
        self.possibleActions = ['HL','ML','L','SL','F','SR','R','MR','HR','S']

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

        random.seed()

    def _step_env(self, action='S'):
        self.robot.step(action)
        self.current_step += 1
        observation = self._get_observation()
        done = self._isDone(observation)
        reward = self._get_reward(done)

        info = {}
        #Test info
        self.action_episode_memory.append(action)
        self.state_episode_memory.append(observation)
        self.currentEpisodeScans.append(self.robot.ranges)

        return observation, reward, done, info
        # return observation

    def _get_reward(self, done):
        return self.robot.rewardFunction(done)

    def _get_observation(self):
        # print("GET OBSERVATION")
        obs = self.robot.get_observation()
        return obs

    def _isDone(self, obs):
        if self.isStuck(obs):
            # print("Stuck")
            if self.current_step < 5:
                print("Short Episode")
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
        # spawn.pose.position.x = -1.5
        # spawn.pose.position.y = -.5

        # # Set a random orientation
        # orient = quaternion_from_euler(0,0,np.deg2rad(random.randint(0,360)))
        # spawn.pose.orientation.x = orient[0]
        # spawn.pose.orientation.y = orient[1]
        # spawn.pose.orientation.z = orient[2]
        # spawn.pose.orientation.w = orient[3]

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
        self.state = ['','','','','','','','']
        self.previousState = ['','','','','','','','']
        self.twoStatesAgo = ['','','','','','','','']
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
        rightBack = 9999
        leftFront = 9999
        left = 9999
        leftBack = 9999

        elemCount = 0

        totalLeftBack = 0
        totalLeft = 0
        totalLeftFront = 0
        totalFront = 0
        totalRightFront = 0
        totalRight = 0
        totalRightBack = 0


        elemCount = 0
        #Getting leftBack min distance
        for i in range(120,171):
            # if self.ranges[i] < leftBack:
            #     leftBack = self.ranges[i]

            # Average Distance
            totalLeftBack += self.ranges[i]
            elemCount += 1

        leftBack = totalLeftBack/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if leftBack <= .25:
            self.state[0] = self.discreteDistances[0]
        elif leftBack <= .9:
            self.state[0] = self.discreteDistances[2]
        elif leftBack > .9:
            self.state[0] = self.discreteDistances[4]
        else:
            print("Error with leftBack values")


        #Getting left min distance
        for i in range(60,121):
            # Closest Distance
            # if self.ranges[i] < left:
            #     left = self.ranges[i]

            # Average Distance
            totalLeft += self.ranges[i]
            elemCount += 1

        left = totalLeft/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if left <= .2:
            self.state[1] = self.discreteDistances[0]
        elif left <= .4:
            self.state[1] = self.discreteDistances[1]
        elif left <= .6:
            self.state[1] = self.discreteDistances[2]
        elif left <= .8:
            self.state[1] = self.discreteDistances[3]
        elif left > .8:
            self.state[1] = self.discreteDistances[4]
        else:
            print("Error with left values")


        elemCount = 0
        #Getting leftFront min distance
        for i in range(21,60):
            # if self.ranges[i] < leftFront:
            #     leftFront = self.ranges[i]

            # Average Distance
            totalLeftFront += self.ranges[i]
            elemCount += 1

        leftFront = totalLeftFront/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if leftFront <= .25:
            self.state[2] = self.discreteDistances[0]
        elif leftFront <= .9:
            self.state[2] = self.discreteDistances[2]
        elif leftFront > .9:
            self.state[2] = self.discreteDistances[4]
        else:
            print("Error with leftFront values")


        elemCount = 0
        #Getting front min distance
        for i in range(-30,31):
            # if self.ranges[i] < front:
            #     front = self.ranges[i]

            # Average Distance
            totalFront += self.ranges[i]
            elemCount += 1

        front = totalFront/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if front <= .25:
            self.state[3] = self.discreteDistances[0]
        elif front < .5:
            self.state[3] = self.discreteDistances[1]
        elif front < .6:
            self.state[3] = self.discreteDistances[2]
        elif front <= 1.0:
            self.state[3] = self.discreteDistances[3]
        elif front > 1.0:
            self.state[3] = self.discreteDistances[4]
        else:
            print("Error with front values")


        elemCount = 0
        #Getting rightFront min distance
        for i in range(-60,-20):
            # if self.ranges[i] < rightFront:
            #     rightFront = self.ranges[i]

            # Average Distance
            totalRightFront += self.ranges[i]
            elemCount += 1

        rightFront = totalRightFront/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if rightFront <= .25:
            self.state[4] = self.discreteDistances[0]
        elif rightFront <= .9:
            self.state[4] = self.discreteDistances[2]
        elif rightFront > .9:
            self.state[4] = self.discreteDistances[4]
        else:
            print("Error with rightFront values")



        elemCount = 0
        #Getting right min distance
        for i in range(-120,-59):
            # if self.ranges[i] < right:
            #     right = self.ranges[i]

            # Average Distance
            totalRight += self.ranges[i]
            elemCount += 1

        right = totalRight/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if right <= .2:
            self.state[5] = self.discreteDistances[0]
        elif right < .3:
            self.state[5] = self.discreteDistances[1]
        elif right < .4:
            self.state[5] = self.discreteDistances[2]
        elif right <= .6:
            self.state[5] = self.discreteDistances[3]
        elif right <= .9:
            self.state[5] = self.discreteDistances[4]
        elif right > .9:
            self.state[5] = self.discreteDistances[5] #####FIXME Took out TF  [4]
        else:
            print("Error with right values")


        elemCount = 0
        #Getting rightBack min distance
        for i in range(-170,-119):
            # if self.ranges[i] < rightBack:
            #     rightBack = self.ranges[i]

            # Average Distance
            totalRightBack += self.ranges[i]
            elemCount += 1

        rightBack = totalRightBack/elemCount

        #Determining Discrete Distance Measure: Close, Medium, Far, etc.
        if rightBack <= .25:
            self.state[6] = self.discreteDistances[0]
        elif rightBack <= .9:
            self.state[6] = self.discreteDistances[2]
        elif rightBack > .9:
            self.state[6] = self.discreteDistances[4]
        else:
            print("Error with rightBack values")




        #Determine the Orientation
        if len(self.previousRanges) == 0 or (self.state[5] == 'TF' and self.previousState[5] == 'TF'):
            self.state[7] = self.discreteOrientations[0]
        elif self.state[5] == self.previousState[5]:
            if self.state[5] == self.twoStatesAgo[5]:
                self.state[7] = self.discreteOrientations[1]
            else:
                self.state[7] = self.discreteOrientations[0]
        else:
            if self.state[5] in ['TF','F'] and self.previousState[5] in ['F','M','C']:
                self.state[7] = self.discreteOrientations[3]
            elif self.state[5] in ['C','TC','TTC'] and self.previousState[5] in ['F','M','C']:
                self.state[7] = self.discreteOrientations[2]

        # # Test Sensor Ranges
        # if self.state[0] in ['C','TC']:
        #     print("Left")
        # if self.state[1] in ['C','TC']:
        #     print("Front")
        # if self.state[3] in ['C','TC']:
        #     print("Right")


        self.previousRanges = self.ranges
        self.twoStatesAgo = self.previousState
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
        msg.y = 0 #geoValues[0]
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
    def rewardFunction(self, isDone):
        ## TODO: Create Reward Function
        leftBack = self.state[0]
        left = self.state[1]
        leftFront = self.state[2]
        front = self.state[3]
        rightFront = self.state[4]
        right = self.state[5]
        rightBack = self.state[6]
        orientation = self.state[7]

        reward = 0

        previousRight = ''
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
        if left in ['TTC','TC','C']:
            # print('Left Bad Reward')
            if left == 'C':
                reward -= 1
            else:
                reward -= 3
        if front in ['TTC','TC','C']:
            # print('Front Bad Reward')
            reward -= 5
        if right in ['TF','TTC']:
            reward -= 5
            if right == 'TF':
                # print('Right Really Bad Reward')
                reward -= 5
            else:
                # print('Right Bad Reward')
                pass
        if right in ['C','M','F'] and front not in ['TTC','TC']:
            # print('Positive')
            reward += 2
            if front not in ['TTC','TC','C','M']:
                reward += 1
                if right == 'M' and orientation == 'P':
                    # print('Best Reward')
                    reward += 2
                else:
                    # print('Great Reward')
                    pass
            elif front not in ['TTC','TC','C']:
                # print('Good Reward')
                reward += 2


        # if right in ['C','M','F'] and front != 'TC':
        #     reward += .5
        if isDone:
            # print('Died Reward')
            reward -= 10
        else:
            # print('Alive Reward')
            reward += 1

        if previousRight == 'M' and right == 'M':
            reward += 2
            if front in ['F','TF']:
                reward += 3
            elif front in ['TTC','TC','C']:
                reward -= 5
        elif previousRight in ['F','C'] and right == 'M':
            reward += 2
        elif previousRight == 'M' and right in ['F', 'C']:
            reward -= 3

        if previousRight == 'M' and right in ['F','TF'] and front not in ['TCC','TC','C','M']:
            reward -= 10

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

                    if rand > eps:
                        action, actionIndex = qTable.maxAction(observation, env.possibleActions)
                    else:
                        actionIndex = random.randint(0,len(env.possibleActions)-2) # The minus two will prevent Stop action from bieng chosen
                        action = env.possibleActions[actionIndex]

                    # #Always Go Forward
                    # actionIndex = len(env.possibleActions)-1
                    # action = env.possibleActions[actionIndex]
                    # print(observation)

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
                env.robot.step(env.possibleActions[len(env.possibleActions)-1])
                totalRewards[i] = epRewards
                # print("Next Episode")


            #Save the QTable Training
            cleanFinalTable = stringify_keys(qTable.table)
            # cleanInitialTable = stringify_keys(qTable.initialTable)

            with open('/home/jawarcholik/Stingray-Simulation/catkin_ws/src/reinforcement_learning/src/qNewtable.txt', 'w') as fp:
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
