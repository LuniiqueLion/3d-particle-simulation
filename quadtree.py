class Point3D:
    def __init__(self, x, y, z, mass=10**20):
        self.x = x
        self.y = y
        self.z = z
        self.mass = mass
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0


class Octree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.subdivided = False
        self.center_of_mass_x = 0
        self.center_of_mass_y = 0
        self.center_of_mass_z = 0
        self.total_mass = 0

    def insert(self, point):
        if not self.boundary.contains_point(point):
            return False  # Point is outside the boundary, cannot insert

        if len(self.points) < self.capacity:
            self.points.append(point)
            return True  # Inserted successfully

        if not self.subdivided:
            self.subdivide()

        if self.northeast.insert(point):
            return True
        if self.northwest.insert(point):
            return True
        if self.southeast.insert(point):
            return True
        if self.southwest.insert(point):
            return True

    def subdivide(self):
        x = self.boundary.x
        y = self.boundary.y
        z = self.boundary.z
        w = self.boundary.width / 2
        h = self.boundary.height / 2
        d = self.boundary.depth / 2

        ne_boundary = Boundary(x + w, y, z, w, h, d)
        nw_boundary = Boundary(x, y, z, w, h, d)
        se_boundary = Boundary(x + w, y + h, z, w, h, d)
        sw_boundary = Boundary(x, y + h, z, w, h, d)

        self.northeast = Octree(ne_boundary, self.capacity)
        self.northwest = Octree(nw_boundary, self.capacity)
        self.southeast = Octree(se_boundary, self.capacity)
        self.southwest = Octree(sw_boundary, self.capacity)

        self.subdivided = True

    def compute_center_of_mass(self):
        total_mass = 0
        center_of_mass_x = 0
        center_of_mass_y = 0
        center_of_mass_z = 0

        for point in self.points:
            total_mass += point.mass
            center_of_mass_x += point.x * point.mass
            center_of_mass_y += point.y * point.mass
            center_of_mass_z += point.z * point.mass

        if self.subdivided:
            for child in [self.northeast, self.northwest, self.southeast, self.southwest]:
                child_total_mass, child_center_of_mass_x, child_center_of_mass_y, child_center_of_mass_z = child.compute_center_of_mass()
                total_mass += child_total_mass
                center_of_mass_x += child_center_of_mass_x
                center_of_mass_y += child_center_of_mass_y
                center_of_mass_z += child_center_of_mass_z

        if total_mass > 0:
            self.center_of_mass_x = center_of_mass_x / total_mass
            self.center_of_mass_y = center_of_mass_y / total_mass
            self.center_of_mass_z = center_of_mass_z / total_mass
            self.total_mass = total_mass
        return total_mass, center_of_mass_x, center_of_mass_y, center_of_mass_z

    def calculate_force(self, point):
        gravitational_constant = 6.67 * (10**-11)  # Universal gravitational constant
        theta = 6 # Theta parameter for the Barnes-Hut approximation
        dx = self.center_of_mass_x - point.x
        dy = self.center_of_mass_y - point.y
        dz = self.center_of_mass_z - point.z
        distance = max(1, (dx**2 + dy**2 + dz**2)**0.5)  # Ensure a minimum distance to prevent singularities
        direction_x = dx / distance
        direction_y = dy / distance
        direction_z = dz / distance

        if self.subdivided:
            if self.boundary.width / distance < theta:
                # Use the center of mass as a single particle
                force_magnitude = (gravitational_constant * point.mass * self.total_mass) / (distance**2)
                force_x = force_magnitude * direction_x
                force_y = force_magnitude * direction_y
                force_z = force_magnitude * direction_z
                return force_x, force_y, force_z
            else:
                # Recursively calculate forces from children
                total_force_x = 0
                total_force_y = 0
                total_force_z = 0
                for child in [self.northeast, self.northwest, self.southeast, self.southwest]:
                    child_force_x, child_force_y, child_force_z = child.calculate_force(point)
                    total_force_x += child_force_x
                    total_force_y += child_force_y
                    total_force_z += child_force_z
                return total_force_x, total_force_y, total_force_z
        else:
            # No subdivision, point is itself
            return 0, 0, 0

class Boundary:
    def __init__(self, x, y, z, width, height, depth):
        self.x = x
        self.y = y
        self.z = z
        self.width = width
        self.height = height
        self.depth = depth

    def contains_point(self, point):
        return (
            self.x <= point.x <= self.x + self.width and
            self.y <= point.y <= self.y + self.height and
            self.z <= point.z <= self.z + self.depth
        )
