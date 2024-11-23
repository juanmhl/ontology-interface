import math

class Point3D:
    """
    A 3D point class with x, y, and z coordinates.

    This class is designed to behave like a 3D vector, allowing you to perform vector operations such as addition,
    subtraction, scalar and vector products, and magnitude computation.

    ---------------------------------------------
    Universidad de Malaga
    Medical Robotics Laboratory
    RACE framework
    ---------------------------------------------
    """
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def scalar_product(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def vector_product(self, other):
        return Point3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def magnitude(self):
        return math.sqrt(self.scalar_product(self))

    def __str__(self):
        return 'Point3D(x={}, y={}, z={})'.format(self.x, self.y, self.z)



# When file is executed as main:
if __name__ == '__main__':
    p1 = Point3D(1, 2, 3)
    p2 = Point3D(4, 5, 6)

    p3 = p1 + p2  # p3 is a Point3D object with x=5, y=7, z=9
    print(p3)  # prints "Point3D(x=5, y=7, z=9)"