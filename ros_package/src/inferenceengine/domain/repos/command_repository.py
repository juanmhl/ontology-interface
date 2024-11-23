from abc import ABCMeta, abstractmethod

class CommandRepository:
    """
    This Abstract Base Class defines the API for the
    Command Repository (RACE frameword) at the Medical
    Robotics Laboratory of the University of Malaga

    It has the following methods:
    - initialize(initialPhase:Phase, nPoints:int, initialActionPoint:int)  
        -> sets:
            -> n_points(int) -> number of suture points recognized at externally analyzed wound
            -> initial_phase(Phase) -> int with the procedure phase at the begining of the iteration
            -> initial_suture_point(int) -> index of the point in which the initial aciton will proceed
        -> used only once at the beginning
    - setPhase(phase:Phase)
        -> allows to force the algorithm phase/state at any moment
    - start()
        -> launches the algorithm transition from phase P0 to phase P1
    - step(predicates_vector:bool[5])
        -> recieves a boolean vector of size 5 with the values of each predicate of the used logic
        -> returns
            -> actual_phase:Phase           -> string with the name of the actual phase
            -> actual_action:Action         -> dict with the name of the actual actuation actions as keys and bools as values
            -> actual_suture_point:int      -> int which points to the suture point in which the action
                                               is being done
    - get_phase() -> Phase
        -> gets last computed Phase
    - get_action() -> Action
        -> gets last computed Action
    - get_suture_point() -> int
        -> gets last computed suture point index

    
    ---------------------------------------------
    Universidad de Malaga
    Medical Robotics Laboratory
    RACE framework
    ---------------------------------------------
                                           
    """

    __metaclass__ = ABCMeta

    # Parameters initialization
    @abstractmethod
    def initialize(self, initial_phase, n_points, initial_suture_point): pass

    # Force HRI phase to input
    @abstractmethod
    def set_phase(self, phase): pass

    # Go from phase 0 to phase 1
    @abstractmethod
    def start(self): pass
        
    # Iterate HRI
    @abstractmethod
    def step(self, predicates_vector): pass

    # This 3 next methods together with .step() will provide the HRI ouput.
    # The diference is that only .setp() will actually execute an
    # algorithm (ontology,petri,FSM) iteration (question and answer).
    @abstractmethod
    def get_phase(self): pass

    @abstractmethod
    def get_action(self) : pass

    @abstractmethod
    def get_suture_point(self): pass

    @abstractmethod
    def get_command(self): pass