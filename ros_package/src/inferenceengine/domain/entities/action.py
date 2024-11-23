class Action:
    """
    A class to represent all possible actions that can be taken by the robot.

    Public Attributes
    -----------------
    agarreAguja: bool
    aperturaD: bool
    aperturaI: bool
    cierreD: bool
    cierreI: bool
    desplazamiento_cambioD: bool
    desplazamiento_cambioI: bool
    desplazamiento_suturaD: bool
    desplazamiento_suturaI: bool
    estiramiento_hilo: bool
    puncion: bool

    """
    grab = None
    go_to_stitch = None
    stitch = None
    stretch_thread = None
    agarreAguja = None
    aperturaD = None
    aperturaI = None
    cierreD = None
    cierreI = None
    desplazamiento_cambioD = None
    desplazamiento_cambioI = None
    desplazamiento_suturaD = None
    desplazamiento_suturaI = None
    estiramiento_hilo = None
    puncion = None

    # def __init__(self, grab=False, go_to_stitch=False, stitch=False, stretch_thread=False):
    def __init__(self, agarreAguja=False, aperturaD=False, aperturaI=False, cierreD=False, cierreI=False, desplazamiento_cambioD=False, desplazamiento_cambioI=False, desplazamiento_suturaD=False, desplazamiento_suturaI=False, estiramiento_hilo=False, puncion=False):
        """
        Constructs all the necessary attributes for the Action object.

        Parameters
        ----------
        agarreAguja: bool
        aperturaD: bool
        aperturaI: bool
        cierreD: bool
        cierreI: bool
        desplazamiento_cambioD: bool
        desplazamiento_cambioI: bool
        desplazamiento_suturaD: bool
        desplazamiento_suturaI: bool
        estiramiento_hilo: bool
        puncion: bool

        """
        # self.grab = grab
        # self.go_to_stitch = go_to_stitch
        # self.stitch = stitch
        # self.stretch_thread = stretch_thread
        # self.post_init()
        self.agarreAguja = agarreAguja
        self.aperturaD = aperturaD
        self.aperturaI = aperturaI
        self.cierreD = cierreD
        self.cierreI = cierreI
        self.desplazamiento_cambioD = desplazamiento_cambioD
        self.desplazamiento_cambioI = desplazamiento_cambioI
        self.desplazamiento_suturaD = desplazamiento_suturaD
        self.desplazamiento_suturaI = desplazamiento_suturaI
        self.estiramiento_hilo = estiramiento_hilo
        self.puncion = puncion

    def __add__(self, other):
        """
        Returns a new Action object with the sum of the attributes of the two objects.

        Parameters
        ----------
        other: Action
            The Action object to be added.

        Returns
        -------
        Action
            The Action object with the sum of the attributes of the two objects.
        """
        # return Action(self.grab or other.grab, self.go_to_stitch or other.go_to_stitch,
        #               self.stitch or other.stitch, self.stretch_thread or other.stretch_thread)
        return Action(self.agarreAguja or other.agarreAguja, self.aperturaD or other.aperturaD, self.aperturaI or other.aperturaI, self.cierreD or other.cierreD, self.cierreI or other.cierreI, self.desplazamiento_cambioD or other.desplazamiento_cambioD, self.desplazamiento_cambioI or other.desplazamiento_cambioI, self.desplazamiento_suturaD or other.desplazamiento_suturaD, self.desplazamiento_suturaI or other.desplazamiento_suturaI, self.estiramiento_hilo or other.estiramiento_hilo, self.puncion or other.puncion)

    # This method could be disabled in the future
    def post_init(self):
        if sum([self.grab, self.go_to_stitch, self.stitch, self.stretch_thread]) > 1:
            raise ValueError("Only one action can be True at a time")



# When file is executed as main, for testing:
if __name__ == '__main__':
    print("Testing Action class: test 1")
    action = Action(grab=True)
    print(action.grab)
    print(action.go_to_stitch)
    print(action.stitch)
    print(action.stretch_thread)

    print("Testing Action class: test 2")
    action = Action(go_to_stitch=True)
    print(action.grab)
    print(action.go_to_stitch)
    print(action.stitch)
    print(action.stretch_thread)

    print("Testing Action class: test 3")
    action = Action(stitch=True)
    print(action.grab)
    print(action.go_to_stitch)
    print(action.stitch)
    print(action.stretch_thread)

    print("Testing Action class: test 4")
    action = Action(stretch_thread=True)
    print(action.grab)
    print(action.go_to_stitch)
    print(action.stitch)
    print(action.stretch_thread)

    # From here, all actions should raise an error:
    print("Testing Action class: test 5")
    try:
        action = Action(grab=True, go_to_stitch=True, stitch=True, stretch_thread=True)
        print(action.grab)
        print(action.go_to_stitch)
        print(action.stitch)
        print(action.stretch_thread)
    except ValueError:
        print("Error raised")

    print("Testing Action class: test 6")
    try:
        action = Action(grab=True, go_to_stitch=True, stitch=True)
        print(action.grab)
        print(action.go_to_stitch)
        print(action.stitch)
        print(action.stretch_thread)
    except ValueError:
        print("Error raised")

    print("Testing Action class: test 7")
    try:
        action = Action(grab=True, go_to_stitch=True, stretch_thread=True)
        print(action.grab)
        print(action.go_to_stitch)
        print(action.stitch)
        print(action.stretch_thread)
    except ValueError:
        print("Error raised")

    print("Testing Action class: test 8")
    try:
        action = Action(grab=True, stitch=True, stretch_thread=True)
        print(action.grab)
        print(action.go_to_stitch)
        print(action.stitch)
        print(action.stretch_thread)
    except ValueError:
        print("Error raised")