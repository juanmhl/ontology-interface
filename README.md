# Inference Engine for the RACE Project

The `inferenceengine` feature encapsulates the inference engine for the RACE project, a collaboration between UMA, UMH, and UVA. It implements an ontology developed at UMA to automate laparoscopic suturing, either partially or fully, using a robotic system.

### File Structure:

- **`ros_package/`**: Python package that encapsulates the inference engine as a ROS service with its ontology.
- **`primitivegenerator/`**: Python package for primitive generation, translating qualitative actions from the inference engine into quantitative actions for robots. *NOT COMPLETED*
- **`umatransforms/`**: Python package with typical homogeneous transform functions based on MATLAB functions from ISA-UMA. *NOT USED*
- **`ros_package/ontology.owl`**: File containing the ontology developed at UMA by Álvaro Galán Cuenca, Marta Fernández Naranjo, and Alfredo Burrieza.
- **`Dockerfile`**: Description of the Docker image used for app development. To build the development environment image, run:
  ```bash
  docker build -t ros-noetic-race .
  ```
- **`launch_container.sh`**: Grant execution permissions with:
  ```bash
  chmod +x launch_container.sh
  ```
  Then execute with:
  ```bash
  ./launch_container.sh
  ```
  to start the container with the ROS development environment. The script maps `ros_package/` on the host to `/catkin_ws/src/inferenceengine/` in the container.
- **`requirements.txt`**: File generated using:
  ```bash
  pip list --format=freeze > requirements.txt
  ```
  Contains all Python package versions used during development. Dependencies are installed automatically in the container.
- **`ros_package/scripts/`**: Test files for validating the `inferenceengine` feature. These are only for testing. The included Jupyter notebook cannot be executed in Docker but contains prior test cases.
- **`ros_package/src/inferenceengine/`**: Inference engine feature, implemented as a service encapsulating the ontology and other experimental implementations (*non-functional*).

### Notes on Docker:

- **Build the Docker image:**
  ```bash
  docker build -t ros-noetic-race .
  ```
- **Run the container:**
  ```bash
  docker run -it --rm --user root -v $PWD/ros_package:/catkin_ws/src/inferenceengine:rw --network=host --ipc=host ros-noetic-race
  ```
  Or, more conveniently:
  ```bash
  ./launch_container.sh
  ```
- **Compile the package in the container:**
  Open a terminal in the container and run:
  ```bash
  catkin_make
  ```
  After compiling, use the node as usual:
  ```bash
  rosrun inferenceengine inferenceengine_node.py
  ```
- **Access additional terminals within the same container:**
  ```bash
  docker exec -it <container_tag> bash
  ```
