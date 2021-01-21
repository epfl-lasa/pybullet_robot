import pybullet as pb


class Simulation(object):

    def __init__(self, realtime_sim=False, realtime_sim_freq=500.0):
        """
        Constructor of the Simulation class. This class creates the PyBullet server / GUI and steps the simulation.

        :param realtime_sim: Use realtime simulation
        :param realtime_sim_freq: If realtime simulation, the simulation frequency

        :type realtime_sim: bool
        :type realtime_sim_freq: float
        """
        self.uid = pb.connect(pb.GUI)
        pb.resetSimulation(physicsClientId=self.uid)

        self.rt_sim = realtime_sim
        if realtime_sim:
            pb.setRealTimeSimulation(1, physicsClientId=self.uid)
            pb.setTimeStep(1.0 / float(realtime_sim_freq), physicsClientId=self.uid)

    def is_alive(self):
        """
        Check if the physics server is still conntected

        :rtype: bool
        """
        return pb.isConnected(self.uid)

    def __del__(self):
        """
        Disconnect the physics server
        """
        pb.disconnect(self.uid)

    def step(self):
        """
        Step the simulation.
        """
        if not self.rt_sim:
            pb.stepSimulation(self.uid)

    @staticmethod
    def add_robot_models_path(path):
        """
        Add the specified directory (absolute) to Pybullet's searchpath for easily adding models from the path.

        :param path: the absolute path to the directory
        :type path: str

        :return: isdir: Boolean if specified directory exists or not
        :rtype: isdir: bool
        """
        import os
        if os.path.isdir(path):
            pb.setAdditionalSearchPath(path)
            print("Added {} to Pybullet path.".format(path))
            return True
        else:
            print("Error adding to Pybullet path! {} not a directory.".format(path))
            return False

    @staticmethod
    def add_PyB_models_path():
        """
        Adds pybullet's in-built models path to the pybullet path for easily retrieving the models.
        """
        import pybullet_data
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
