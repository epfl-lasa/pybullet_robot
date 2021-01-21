import pybullet as pb


class WorldObjects(object):

    def __init__(self, object_dict={}):
        for k in object_dict:
            setattr(self, k, object_dict[k])

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def __getitem__(self, key):
        return getattr(self, key)


class EmptyWorld(object):

    def __init__(self, uid, add_plane=False, gravity=[0, 0, -9.81]):
        """
        Constructor of the EmptyWorld class. Specify the gravity in this world and add objects to the world.

        :param uid: server id of PyBullet
        :param add_plane: Add plane to world
        :param gravity: Gravity vector [X, Y, Z]

        :type uid: int
        :type add_plane: bool
        :type gravity: list of float
        """
        self._uid = uid
        pb.setGravity(gravity[0], gravity[1], gravity[2], physicsClientId=self._uid)
        self._objects = WorldObjects()
        if add_plane:
            self.add_object_from_urdf('plane', 'plane.urdf', [0, 0, 0], [0, 0, 0, 1])

    def add_object_from_urdf(self, name, object_description, position_xyz, orientation_wxyz, scaling=1.,
                             fixed_base=True):
        """
        Add an object to the world.

        :param name: Name of the object
        :param object_description: urdf file of the object
        :param position_xyz: Desired position of the object
        :param orientation_wxyz: Desired orientaton of the object (quaternion xyzw)
        :param scaling: Optional scaling factor
        :param fixed_base: Optional fixed base boolean

        :type name: str
        :type object_description: str
        :type position_xyz: list of float
        :type orientation_wxyz: list of float
        :type scaling: float
        :type fixed_base: bool
        """
        try:
            obj = pb.loadURDF(object_description, useFixedBase=fixed_base, globalScaling=scaling,
                              physicsClientId=self._uid)
        except Exception as ex:
            print("Error: ", ex)
            print("Adding the PyBullet data path to the search path and trying again.")
            self.add_PyB_models_path()
            obj = pb.loadURDF(object_description, useFixedBase=fixed_base, globalScaling=scaling,
                              physicsClientId=self._uid)
        pb.resetBasePositionAndOrientation(obj, position_xyz, orientation_wxyz, physicsClientId=self._uid)
        self._add_object({name: obj})

    def _add_object(self, object):
        """
        Add object to the list of world objects.

        :param object: Name and PyBullet object ID of the object in form of a dict
        :type object: dict[str, int]
        """
        for k in object:
            print("Adding", k)
            if not hasattr(self._objects, k):
                self._objects[k] = object
            else:
                print("Object with keyname '{}' already present. Not adding to world.".format(k))

    def remove_objects(self, objects=[]):
        """
        Remove object from the list of world objects.

        :param objects: List of objects to be removed
        :type objects: list of str
        """
        if isinstance(objects, str):
            objects = [objects]
        for obj in objects:
            print("Removing", obj)
            pb.removeBody(self._objects[obj][obj])
            delattr(self._objects, obj)

    @property
    def objects(self):
        return self._objects

    @staticmethod
    def add_PyB_models_path():
        """
        Adds pybullet's in-built models path to the pybullet path for easily retrieving the models.
        """
        import pybullet_data
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
