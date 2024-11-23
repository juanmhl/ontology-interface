from inferenceengine.domain.repos.command_repository import CommandRepository
from inferenceengine.domain.entities import *

class CommandRepositoryMock(CommandRepository):
    """
    Mock implementation of CommandRepository
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

    def initialize(self, initial_phase, n_points, initial_suture_point):

        self.__phase = initial_phase
        self.__n_points = n_points
        self.__suture_point = initial_suture_point
        self.__action = self.__action_dict[self.__phase.value]
        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        return True

    def set_phase(self, phase):
        self.__phase = phase
        return True

    def start(self): 
        if self.__phase.value == 0:
            self.__phase = Phase(1)
            return True
        else:
            return False

    def step(self, predicatesVector):
        if self.__phase.value < 4:
            self.__phase = self.__phase + 1
        else:
            self.__phase = 1
            if self.__suture_point < self.__n_points:
                self.__suture_point = self.__suture_point + 1
            else:
                print('ALGORITMO TERMINADO')

        self.__action = self.__action_dict[self.__phase.value]

        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        return self.__command

    def get_phase(self):
        return self.__phase

    def get_action(self):
        return self.__action

    def get_suture_point(self):
        return self.__suture_point

    def get_command(self):
        return self.__command

if __name__ == "__main__":
    repo = CommandRepositoryMock()
    repo.initialize(Phase(3),5,1)
    # PrintHRIrepositoryAttributes(repo)