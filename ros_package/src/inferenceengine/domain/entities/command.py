# Import the necessary packages
from .phase import Phase
from .action import Action

class Command:
    """
    A class to represent all possible high level commands that can be taken by the robot.

    Public Attributes
    -----------------
    phase: Phase
    action: Action
    suturePoint: int
    near: bool
    """

    def __init__(self, phase, action, suturePoint, near):
        """"
        Constructs all the necessary attributes for the Command object.

        Parameters
        ----------
        phase: Phase
        action: Action
        suturePoint: int
        near: bool
        """

        self.phase = phase
        self.action = action
        self.suturePoint = suturePoint
        self.near = near

    def __str__(self):
        """
        Returns a string representation of the Command object.
        """
        # action_str = ""
        # if self.action.grab:
        #     action_str = "GRAB"
        # elif self.action.go_to_stitch:
        #     action_str = "GO_TO_STITCH"
        # elif self.action.stitch:
        #     action_str = "STITCH"
        # elif self.action.stretch_thread:
        #     action_str = "STRETCH_THREAD"

        action_str = ""
        if self.action:
            if self.action.agarreAguja:
                action_str = action_str + "-AGARRE_AGUJA-"
            if self.action.aperturaD:
                action_str = action_str + "-APERTURA_D-"
            if self.action.aperturaI:
                action_str = action_str + "-APERTURA_I-"
            if self.action.cierreD:
                action_str = action_str + "-CIERRE_D-"
            if self.action.cierreI:
                action_str = action_str + "-CIERRE_I-"
            if self.action.desplazamiento_cambioD:
                action_str = action_str + "-DESPLAZAMIENTO_CAMBIO_D-"
            if self.action.desplazamiento_cambioI:
                action_str = action_str + "-DESPLAZAMIENTO_CAMBIO_I-"
            if self.action.desplazamiento_suturaD:
                action_str = action_str + "-DESPLAZAMIENTO_SUTURA_D-"
            if self.action.desplazamiento_suturaI:
                action_str = action_str + "-DESPLAZAMIENTO_SUTURA_I-"
            if self.action.estiramiento_hilo:
                action_str = action_str + "-ESTIRAMIENTO_HILO-"
            if self.action.puncion:
                action_str = action_str + "-PUNCION-"
        else:
            action_str = "NONE"

        if self.phase:
            phase_str = self.phase
        else:
            phase_str = "NONE"

        if self.suturePoint:
            suture_point_str = str(self.suturePoint)
        else:
            suture_point_str = "NONE"

        if self.near:
            near_str = str(self.near)
        else:
            near_str = "NONE"

        return "Phase: {}, Action: {}, Suture Point: {}, Near: {}".format(phase_str, action_str, suture_point_str, near_str)


# If the file is executed as a script, run the following:
if __name__ == "__main__":
    action = Action(grab=True)
    phase = Phase(1)
    command = Command(phase, action, 1, None)
    print(command)