import pybullet as p
import pybullet_data
import time
import math

class Robot:
    """Klasa reprezentująca robota w symulacji."""
    
    def __init__(self, urdf_path, start_pos=[0, 0, 0], start_euler=[0, 0, 0]):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_ori = p.getQuaternionFromEuler(start_euler)
        self.robot_id = None
        
        self.movable_joints = []
        self.sliders = []
        
        # Automatyczne wczytanie robota i inicjalizacja suwaków przy tworzeniu obiektu
        self._load_urdf()
        self._setup_joints_and_sliders()

    def _load_urdf(self):
        """Prywatna metoda do wczytywania modelu z pliku URDF."""
        try:
            print(f"Wczytywanie {self.urdf_path}...")
            self.robot_id = p.loadURDF(self.urdf_path, self.start_pos, self.start_ori, useFixedBase=True)
            print("Robot wczytany pomyślnie!")
        except p.error as e:
            print(f"Błąd podczas wczytywania URDF: {e}")
            print(f"Upewnij się, że plik {self.urdf_path} znajduje się w folderze ze skryptem.")
            p.disconnect()
            exit()

    def _setup_joints_and_sliders(self):
        """Prywatna metoda do analizy stawów i generowania interfejsu suwaków."""
        num_joints = p.getNumJoints(self.robot_id)
        print("\nZnalezione ruchome stawy:")
        
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
                
                print(f"- {joint_name} (Indeks: {i}, Zakres: {slider_min:.2f} do {slider_max:.2f})")

    def update_from_sliders(self):
        """Metoda aktualizująca pozycję stawów na podstawie wartości z suwaków."""
        for i, joint_idx in enumerate(self.movable_joints):
            target_angle = p.readUserDebugParameter(self.sliders[i])
            
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=100
            )


class Simulation:
    """Klasa zarządzająca środowiskiem PyBullet i pętlą symulacji."""
    
    def __init__(self):
        # Inicjalizacja PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Wczytanie środowiska domyślnego
        self.plane_id = p.loadURDF("plane.urdf")
        
        try:
            self.cube_id = p.loadURDF("kostka.urdf")
        except p.error:
            print("Uwaga: Nie znaleziono pliku kostka.urdf. Symulacja będzie kontynuowana bez niej.")
            
        # Lista robotów znajdujących się w symulacji
        self.robots = []

    def add_robot(self, robot):
        """Dodaje obiekt robota do świata symulacji."""
        self.robots.append(robot)

    def run(self):
        """Główna pętla symulacji."""
        print("\nSymulacja uruchomiona. Możesz sterować suwakami po prawej stronie. Naciśnij Ctrl+C, aby zakończyć.")
        try:
            while True:
                # Aktualizuj pozycje każdego robota w środowisku
                for robot in self.robots:
                    robot.update_from_sliders()
                
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                
        except KeyboardInterrupt:
            print("\nZakończono symulację.")
        finally:
            self.cleanup()

    def cleanup(self):
        """Odłączenie od serwera fizyki po zakończeniu pracy."""
        p.disconnect()


# --- Punkt wejścia programu ---
if __name__ == "__main__":
    # 1. Tworzymy świat symulacji
    sim = Simulation()
    
    # 2. Tworzymy instancję naszego robota
    my_robot = Robot("model1.urdf")
    
    # 3. Dodajemy robota do symulacji
    sim.add_robot(my_robot)
    
    # 4. Uruchamiamy pętlę główną
    sim.run()