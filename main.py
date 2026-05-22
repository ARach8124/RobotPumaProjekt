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
        
        self._load_urdf()
        self._setup_joints_and_sliders()

    def _load_urdf(self):
        """Prywatna metoda do wczytywania modelu z pliku URDF."""
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


class Simulation:
    def __init__(self):
        # Inicjalizacja PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Wczytanie środowiska domyślnego
        self.plane_id = p.loadURDF("plane.urdf")
        
        try:
            self.cube_id = p.loadURDF("kostka.urdf", [0,-1,1])
        except p.error:
            print("Nie udało się wczytać kostki.")
            
        self.robot = None

    def add_robot(self, robot):
        self.robot = robot

    def run(self):
        # Główna pętla symualacji    
        print("\nSymulacja uruchomiona.")
        try:
            while True:
                self.robot.update_from_sliders()
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