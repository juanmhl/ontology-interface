
class InferenceMotorService:
    def __init__(self, command_repo_implementation):
        self.__command_repo = command_repo_implementation()

    def initialize(self, initial_phase=None, n_points=None, initial_suture_point=None, java_path='/usr/bin/java', ontology_path='/home/juanmhl/ontology.owl'):
        return self.__command_repo.initialize(initial_phase, n_points, initial_suture_point, java_path, ontology_path)

    def set_phase(self, phase):
        return self.__command_repo.set_phase(phase)

    def start(self):
        return self.__command_repo.start()

    def step(self, predicates_vector):
        return self.__command_repo.step(predicates_vector)

    def get_phase(self):
        return self.__command_repo.get_phase()

    def get_action(self):
        return self.__command_repo.get_action()

    def get_suture_point(self):
        return self.__command_repo.get_suture_point()

    def get_command(self):
        return self.__command_repo.get_command()