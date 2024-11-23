from inferenceengine.domain.repos.command_repository import CommandRepository
from inferenceengine.domain.entities import *
import numpy as np

class CommandRepositoryPetri(CommandRepository):
    """
    Petri network implementation of HRIrepositoryAPI
    ____________________________________
    """

    __doc__ += CommandRepository.__doc__

    __n_points = None
    __phase = None
    __action = None
    __suture_point = None
    __command = None

    __action_dict = {
        0: Action(grab=True),
        1: Action(go_to_stitch=True),
        2: Action(stitch=True),
        3: Action(stretch_thread=True),
        4: Action(grab=True),
    }

    __A = np.array([[-1,0,0,0,0],[1,-1,0,0,1],[0,1,-1,0,0],[0,0,1,-1,0],[0,0,0,1,-1]])  # Incidence matrix
    __SI = np.array([1,0,0,0,0]).T                                                      # State - INITIAL
    __SP = np.array([1,0,0,0,0]).T                                                      # State - PREVIOUS
    __SA = np.array([0,0,0,0,0]).T                                                      # State - ACTUAL
    __TV = np.array([1,0,0,0,0]).T

    def initialize(self, initial_phase, n_points, initial_suture_point):

        self.__phase = initial_phase
        self.__n_points = n_points
        self.__suture_point = initial_suture_point
        self.__action = self.__action_dict[self.__phase]
        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        self.__SA = np.zeros((5,1))
        self.__SA[initial_phase,0] = 1

        return True

    def set_phase(self, phase):
        self.__phase = phase
        self.__SA = np.zeros((5,1))
        self.__SA[phase,0] = 1
        return True

    def start(self): 
        if self.__phase == 0:
            self.__phase = Phase(1)
            self.__SA = np.array([0,1,0,0,0]).T
            self.__action = self.__action_dict[self.__phase]
            self.command = Command(self.__phase, self.__action, self.__suture_point, None)
            return True
        else:
            return False

    def step(self, predicatesVector):

        if self.__phase != 0:
            # Petri normal operation only happens out of state 0
            # To get out of state 0 use method .start()

            # Build transition vector from predicates vector
            if   predicatesVector == [1,0,0,0,0]: self.__TV = np.array(predicatesVector)
            elif predicatesVector == [0,1,0,0,0]: self.__TV = np.array(predicatesVector)
            elif predicatesVector == [0,0,1,0,0]: self.__TV = np.array(predicatesVector)
            elif predicatesVector == [0,0,0,1,0]: self.__TV = np.array(predicatesVector)
            elif predicatesVector == [0,0,0,0,1]: self.__TV = np.array(predicatesVector)
            else : self.__TV = np.array([0,0,0,0,0])

            # If TV is valid save transition variable so that next if executes
            if 1 in self.__TV:
                array = (np.nonzero(self.__TV == 1))
                transition = array[0][0]
            else:
                transition = 0

            # Petri only executes if transition is viable
            if transition == self.__phase:
                self.__SP = self.__SA.copy()
                self.__SA = self.__SP + np.matmul(self.__A, self.__TV)      # SA = SP + A*TV
                phase = (np.nonzero(self.__SA == 1))                        # Update output phase variable
                self.__phase = Phase(phase[0][0])

            # self.__SP = self.__SA.copy()
            # self.__SA = self.__SP + self.__A @ self.__TV        # SA = SP + A*TV
            # phase = (np.nonzero(self.__SA == 1))                # Update output phase variable
            # self.__phase = phase[0][0]

        # Output actions
        # For now output action is the same as actual state
        self.__action = self.__action_dict[self.__phase]

        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        return self.__command

    def get_phase(self):
        return self.__phase

    def get_action(self):
        return self.__action

    def get_suture_point(self):
        return self.__suture_point

    def get_command(self):
        self.command = Command(self.__phase, self.__action, self.__suture_point, None)
        return self.__command

if __name__ == "__main__":
    initial_phase = Phase(0)
    n_points = 5
    initial_suture_point = 0
    repo = CommandRepositoryPetri()
    repo.initialize(initial_phase,n_points,initial_suture_point)
    print(repo.get_command())
    # PrintHRIrepositoryAttributes(repo)