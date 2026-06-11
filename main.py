import pybullet as p
import pybullet_data
import time
import math

class Robot:
    def __init__(self, urdf_path, start_pos=[0, 0, 0], start_euler=[0, 0, 0]):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_ori = p.getQuaternionFromEuler(start_euler)
        self.robot_id = None
        
        self.movable_joints = []
        self.sliders = []
        
        # Grab state variables managed by the Robot
        self.is_grabbing = False
        self.grab_constraint_id = None
        
        self._load_urdf()
        self._setup_joints_and_sliders()

    def _load_urdf(self):
        try:
            self.robot_id = p.loadURDF(self.urdf_path, self.start_pos, self.start_ori, useFixedBase=True)
            print("Wczytano URDF robota")
        except p.error as e:
            print(f"Błąd podczas wczytywania URDF: {e}")
            p.disconnect()
            exit()

    def _setup_joints_and_sliders(self):
        num_joints = p.getNumJoints(self.robot_id)
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            joint_name = joint_info[1].decode("utf-8")
            
            if joint_type != p.JOINT_FIXED:
                self.movable_joints.append(i)
                
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                
                # Ustalanie zakresu suwaka
                if lower_limit < upper_limit:
                    slider_min = lower_limit
                    slider_max = upper_limit
                else:
                    slider_min = -math.pi
                    slider_max = math.pi
                    
                slider = p.addUserDebugParameter(joint_name, slider_min, slider_max, 0)
                self.sliders.append(slider)
                
    def update_from_sliders(self):
        for i, joint_idx in enumerate(self.movable_joints):
            target_angle = p.readUserDebugParameter(self.sliders[i])
            # Funkcja PyBullet do sterowania pozycją
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=100
            )

    def get_link_index(self, link_name):
        """Helper to find the PyBullet link index by its URDF name."""
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode("utf-8") == link_name:
                return i
        return -1

    def toggle_grab(self, target_id):
        """Wrapper to easily switch between grabbing and releasing."""
        if not self.is_grabbing:
            self.grab(target_id)
        else:
            self.release(target_id)

    def grab(self, target_id):
        """Locks the target object to the manipulator, preserving its current relative position."""
        ee_idx = self.get_link_index("manipulator_link")
        if ee_idx == -1:
            print("Manipulator link not found!")
            return

        # Get positions of the end-effector and the target object
        ee_state = p.getLinkState(self.robot_id, ee_idx)
        ee_pos = ee_state[4] # linkWorldPosition
        ee_ori = ee_state[5] # linkWorldOrientation
        target_pos, target_ori = p.getBasePositionAndOrientation(target_id)

        # Calculate distance
        dist = math.sqrt(sum((a - b)**2 for a, b in zip(ee_pos, target_pos)))

        # Distance threshold (0.15 meters)
        if dist < 0.15:
            print("Chwytam obiekt w jego obecnej pozycji!")
            
            # Disable collisions between robot and the object
            for i in range(-1, p.getNumJoints(self.robot_id)):
                p.setCollisionFilterPair(self.robot_id, target_id, i, -1, enableCollision=0)

            # Calculate relative position and orientation so it doesn't snap to center
            inv_ee_pos, inv_ee_ori = p.invertTransform(ee_pos, ee_ori)
            rel_pos, rel_ori = p.multiplyTransforms(inv_ee_pos, inv_ee_ori, target_pos, target_ori)

            # Create the constraint using the calculated offset
            self.grab_constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=ee_idx,
                childBodyUniqueId=target_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=rel_pos,
                childFramePosition=[0, 0, 0],
                parentFrameOrientation=rel_ori
            )
            self.is_grabbing = True
        else:
            print("Obiekt jest za daleko!")

    def release(self, target_id):
        """Removes the constraint and restores collisions."""
        print("Puszczam obiekt!")
        
        # Remove constraint
        if self.grab_constraint_id is not None:
            p.removeConstraint(self.grab_constraint_id)
            self.grab_constraint_id = None
            
        # Re-enable collisions
        for i in range(-1, p.getNumJoints(self.robot_id)):
            p.setCollisionFilterPair(self.robot_id, target_id, i, -1, enableCollision=1)
            
        self.is_grabbing = False


class Simulation:
    def __init__(self):
        # Inicjalizacja PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Wczytanie środowiska domyślnego
        self.plane_id = p.loadURDF("plane.urdf")
        
        try:
            # Moved closer so the arm can reach it easily
            self.cube_id = p.loadURDF("kostka.urdf", [0, -0.4, 0.4])
        except p.error:
            print("Nie udało się wczytać kostki.")
            self.cube_id = None
            
        self.robot = None

    def add_robot(self, robot):
        self.robot = robot

    def handle_keyboard(self):
        """Checks for keyboard input 'c'."""
        keys = p.getKeyboardEvents()
        g_key = ord('c')
        
        # Check if 'g' was just triggered
        if g_key in keys and keys[g_key] & p.KEY_WAS_TRIGGERED:
            if self.robot and self.cube_id is not None:
                self.robot.toggle_grab(self.cube_id)

    def run(self):
        print("\nSymulacja uruchomiona. Wciśnij 'g' na klawiaturze (w oknie symulacji), aby chwycić/puścić kostkę.")
        try:
            while True:
                if self.robot:
                    self.robot.update_from_sliders()
                self.handle_keyboard()
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                
        except KeyboardInterrupt:
            print("\nZakończono symulację.")
        finally:
            self.cleanup()

    def cleanup(self):
        p.disconnect()


if __name__ == "__main__":
    # Stworzenie symulacji
    sim = Simulation()
    # Wczytanie robota
    my_robot = Robot("model1.urdf")
    # Dodanie robota do symulacji
    sim.add_robot(my_robot)
    # Uruchomienie symulacji
    sim.run()