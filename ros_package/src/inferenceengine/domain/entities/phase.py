from enum import IntEnum

class Phase(IntEnum):
    """
    Phase of the Inference Motor Feature:

    Phase 0: Phase.P0 - 0
    Phase 1: Phase.P1 - 1
    Phase 2: Phase.P2 - 2
    Phase 3: Phase.P3 - 3
    Phase 4: Phase.P4 - 4
    Phase 5: Phase.P0 - 5
    Phase 6: Phase.P1 - 6
    Phase 7: Phase.P2 - 7
    Phase 8: Phase.P3 - 8
    Phase 9: Phase.P4 - 9

    ---------------------------------------------
    Universidad de Malaga
    Medical Robotics Laboratory
    RACE framework
    ---------------------------------------------

    """

    P0 = 0
    P1 = 1
    P2 = 2
    P3 = 3
    P4 = 4
    P5 = 5
    P6 = 6
    P7 = 7
    P8 = 8
    P9 = 9

# When file is executed as main, for testing:
if __name__ == '__main__':
    fase = Phase.P0
    print(type(fase))
    fase.value += 1
    print(type(fase))
    print(fase)
    print(fase + 1)
    print(fase.value)

    fase = fase + 2
    print(fase)
    print(fase == Phase.P2)
    print(fase == 2)
    print(fase == 3)
    print(fase == Phase.P3)

    print(type(fase))
