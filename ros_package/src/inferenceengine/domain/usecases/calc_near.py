# Threshold for considering near values
NEAR_THRESHOLD = 0.5

def calc_near(pointA, pointB):
    """"
    Calculates the distance between two points and returns true
    if it is lower than NEAR_THRESHOLD.

    Parameters
    ----------
    pointA: Point3D
    pointB: Point3D

    Returns
    -------
    bool
    """
    # Calculate the distance between the two points
    difference = pointA - pointB
    distance = difference.magnitude()

    # Return whether the distance is less than the near threshold
    return distance < NEAR_THRESHOLD



# When file is executed as main:
if __name__ == '__main__':
    # Import the Point3D class
    from inferenceengine.domain.entities import Point3D

    # Create two points
    pointA = Point3D(0, 0, 0)
    pointB = Point3D(0.3, 0, 0.2)

    # Calculate the distance between the two points
    near = calc_near(pointA, pointB)

    # Print the result
    print(near)