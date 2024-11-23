from inferenceengine.srv import initialize
from inferenceengine import *
import contextlib

# repo_implementation = CommandRepositoryPetri
repo_implementation = CommandRepositoryOntology
service = InferenceMotorService(repo_implementation)

# Initiazlize the service
service.initialize(n_points=3, ontology_path='/catkin_ws/src/inferenceengine/ontology.owl')
print('After initialization')
print(service.get_command())

# Start the service and print the command
service.start()
print('After start')
print(service.get_command())

vp1 = [True, False, False, False, False, False, True, [False, False, False]]
vp2 = [True, False, False, False, True, False, True, [False, False, False]]
vp3 = [True, False, False, False, True, True, False, [False, False, False]]
vp4 = [True, True, False, False, True, True, False, [False, False, False]]
vp5 = [False, True, False, False, True, True, False, [False, False, False]]
vp6 = [False, True, False, True, True, False, False, [False, False, False]]
vp7 = [False, True, True, True, False, False, False, [False, False, False]]
vp8 = [True, True, True, True, False, False, False, [False, False, False]]
vp9 = [True, False, True, True, False, False, False, [False, False, False]]
vpt = [True, True, True, True, True, True, True, [False, False, False]]
vpf = [False, False, False, False, False, False, False, [False, False, False]]

predicates_vectors = [vp1, vp2, vp3, vp4, vp5, vp6, vp7, vp8, vp9, vpt, vpf, vp2, vp3, vp5]

for vp in predicates_vectors:
    with contextlib.redirect_stdout(None):
        comando = service.step(vp)
    print(comando)