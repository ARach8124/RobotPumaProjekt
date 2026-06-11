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

class Camera:
    def __init__(self):
        self.yaw = 0.0
        self.pitch = -20.0
        self.dist = 5.0
        self.target_pos = [0, 0, 0]
        self.lmb_pressed = False
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.sensitivity = 0.2
        self.custom_camera_enabled = False
        p.resetDebugVisualizerCamera(self.dist, self.yaw, self.pitch, self.target_pos)
    
    def toggle_c(self):
        keys = p.getKeyboardEvents()
        if ord('c') in keys:  
            if keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                self.custom_camera_enabled = not self.custom_camera_enabled
    def handle_camera(self):
        self.toggle_c()
        if self.custom_camera_enabled == True:
            if self.custom_camera_enabled:
                mouse_events = p.getMouseEvents()
                for event in mouse_events:
                    event_type = event[0]
                    mouse_x = event[1]
                    mouse_y = event[2]
                    
                    if event_type == 2:
                        button_index = event[3]
                        button_state = event[4]
            
                        if button_index == 0:
                            if button_state == 3:       
                                self.lmb_pressed = True
                                self.prev_x = mouse_x
                                self.prev_y = mouse_y
                            elif button_state == 4:     
                                self.lmb_pressed = False

                    elif event_type == 1:
                        # Zmieniamy parametry yaw/pitch TYLKO, gdy system jest włączony
                        if self.lmb_pressed and self.custom_camera_enabled:
                            dx = mouse_x - self.prev_x
                            dy = mouse_y - self.prev_y
                
                            self.yaw -= dx * self.sensitivity
                            self.pitch -= dy * self.sensitivity
                
                            if self.pitch > 89.0: self.pitch = 89.0
                            if self.pitch < -89.0: self.pitch = -89.0
                
                            p.resetDebugVisualizerCamera(self.dist, self.yaw, self.pitch, self.target_pos)
                            
                            self.prev_x = mouse_x
                            self.prev_y = mouse_y
        

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
        
        self.camera = Camera()
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        #Do uczenia
        self.previous_click=0
        self.uczenie_toggle=False
        self.przycisk_uczenie_on=p.addUserDebugParameter("Tryb Uczenia", 1, 0, 0)
    def add_robot(self, robot):
        self.robot = robot
                            
    def przycisk_uczenia(self):
        self.uczenie_on=p.readUserDebugParameter(self.przycisk_uczenie_on)
        current_click=self.uczenie_on
        if current_click>self.previous_click:
            self.previous_click=current_click
            self.uczenie_toggle = not self.uczenie_toggle
            if self.uczenie_toggle:
                print("Tryb uczenia włączony")
            else:
                print("Tryb uczenia wyłączony")
                            
    def tryb_uczenia(self):
        self.przycisk_uczenia()
        if self.uczenie_toggle:
            p.readUserDebugParameter(self.przycisk_uczenie_on)

    def run(self):
        # Główna pętla symualacji    
        print("\nSymulacja uruchomiona.")
        try:
            while True:
                p.stepSimulation()
                self.camera.handle_camera()
                if self.robot is not None:
                    self.robot.update_from_sliders()
                #self.tryb_uczenia()
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