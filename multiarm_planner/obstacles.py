import pybullet as p


class Obstacle(object):
    def __init__(self, urdf_file, position, scale=1.0) -> None:
        self.body_id = p.loadURDF(urdf_file, position, globalScaling=scale)

    def __del__(self):
        p.removeBody(self.body_id)

    @staticmethod
    def load_obstacles(obstacles):
        return list(Obstacle(obstacle['urdf_file'], obstacle['position'], obstacle['scale']) for obstacle in obstacles)
