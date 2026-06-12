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
        
        self.use_ik = False 
        self.is_grabbing = False
        self.grab_constraint_id = None

        self._load_urdf()
        self._setup_joints_and_sliders()

#Wczytywanie modelu robota
    def _load_urdf(self):
        try:
            self.robot_id = p.loadURDF(self.urdf_path, self.start_pos, self.start_ori, useFixedBase=True)
            print("Wczytano URDF robota")
        except p.error as e:
            print(f"Błąd podczas wczytywania URDF: {e}")
            p.disconnect()
            exit()
#Utworzenie sliderów do sterowania przegubami
    def _setup_joints_and_sliders(self):
        num_joints = p.getNumJoints(self.robot_id)
        
        # Setup dla kinematyki prostej
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            joint_name = joint_info[1].decode("utf-8")
            
            if joint_type != p.JOINT_FIXED:
                self.movable_joints.append(i)
                
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                
                if lower_limit < upper_limit:
                    slider_min = lower_limit
                    slider_max = upper_limit
                else:
                    slider_min = -math.pi
                    slider_max = math.pi
                    
                slider = p.addUserDebugParameter(f"FK: {joint_name}", slider_min, slider_max, 0)
                self.sliders.append(slider)
                
        # Setup suwaków dla kinematyki odwrotnej
        self.ik_x_slider = p.addUserDebugParameter("IK_Target_X", -1.2, 1.2, 0.4)
        self.ik_y_slider = p.addUserDebugParameter("IK_Target_Y", -1.2, 1.2, 0.0)
        self.ik_z_slider = p.addUserDebugParameter("IK_Target_Z", 0.0, 1.6, 0.6)
        
#Zmiana trybu między sterowanie kątami i kinematyką odwrotną
    def toggle_mode(self):
        self.use_ik = not self.use_ik
        aktualny_tryb = "IK (Kinematyka Odwrotna)" if self.use_ik else "FK (Kinematyka Prosta)"
        print(f"Przełączono na tryb: {aktualny_tryb}")
        
#Rozwiązanie zadania odwrotnej kinematyki
    def ik_analytical(self, x, y, z):
        z_base_offset = 0.025 + 0.3  
        x_offset = 0.16 - 0.14       
        L1 = 0.6                     
        L2 = 0.5                    
        
        R2 = x**2 + y**2
        if R2 < x_offset**2:
            return None
            
        y_plan = math.sqrt(R2 - x_offset**2)
        theta1 = math.atan2(y, x) - math.atan2(-y_plan, x_offset)
        theta1 = (theta1 + math.pi) % (2 * math.pi) - math.pi
        
        z_plan = z - z_base_offset
        D = (z_plan**2 + y_plan**2 - L1**2 - L2**2) / (2 * L1 * L2)
        D = max(-1.0, min(1.0, D))
        
        best_t2, best_t3 = 0, 0
        
        for sign in [1, -1]:
            t3 = sign * math.acos(D)
            t2 = math.atan2(y_plan, z_plan) - math.atan2(L2 * math.sin(t3), L1 + L2 * math.cos(t3))
            
            t2 = (t2 + math.pi) % (2 * math.pi) - math.pi
            t3 = (t3 + math.pi) % (2 * math.pi) - math.pi
            
            if -1.0 <= t2 <= 1.75 and -2.3 <= t3 <= 2.4:
                best_t2, best_t3 = t2, t3
                break
                
        best_t1 = max(-2.618, min(2.618, theta1))
        return [best_t1, best_t2, best_t3]
    
#Aktualizacja przegubów na podstawie sliderów
    def update_from_sliders(self):
        if not self.use_ik:
            for i, joint_idx in enumerate(self.movable_joints):
                target_angle = p.readUserDebugParameter(self.sliders[i])
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_angle,
                    force=400
                )
        else:
            x = p.readUserDebugParameter(self.ik_x_slider)
            y = p.readUserDebugParameter(self.ik_y_slider)
            z = p.readUserDebugParameter(self.ik_z_slider)
            
            target_angles = self.ik_analytical(x, y, z)
            
            if target_angles:
                for i, joint_idx in enumerate(self.movable_joints):
                    p.setJointMotorControl2(
                        bodyIndex=self.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angles[i],
                        force=400
                    )

#Wyciągnięcie nazwy linka
    def get_link_index(self, link_name):
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode("utf-8") == link_name:
                return i
        return -1
    
#Włączanie i wyłączanie chwytania
    def toggle_grab(self, target_id):
        if not self.is_grabbing:
            self.grab(target_id)
        else:
            self.release(target_id)

#Wykonanie chwytu
    def grab(self, target_id):
        ee_idx = self.get_link_index("manipulator_link")

        ee_state = p.getLinkState(self.robot_id, ee_idx)
        ee_pos = ee_state[4] 
        ee_ori = ee_state[5] 
        target_pos, target_ori = p.getBasePositionAndOrientation(target_id)

        dist = math.sqrt(sum((a - b)**2 for a, b in zip(ee_pos, target_pos)))

        if dist < 0.15:
            print("Chwytam obiekt!")
            for i in range(-1, p.getNumJoints(self.robot_id)):
                p.setCollisionFilterPair(self.robot_id, target_id, i, -1, enableCollision=0)

            inv_ee_pos, inv_ee_ori = p.invertTransform(ee_pos, ee_ori)
            rel_pos, rel_ori = p.multiplyTransforms(inv_ee_pos, inv_ee_ori, target_pos, target_ori)

            self.grab_constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=ee_idx,
                childBodyUniqueId=target_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=rel_pos,
                childFramePosition=[0, 0, 0],
                parentFrameOrientation=rel_ori)
            
            self.is_grabbing = True
        else:
            print("Obiekt jest za daleko!")
            
#Wykonanie upuszczenia klocka
    def release(self, target_id):
        if self.grab_constraint_id is not None:
            p.removeConstraint(self.grab_constraint_id)
            self.grab_constraint_id = None
            print("Puszczam obiekt!")
            
        for i in range(-1, p.getNumJoints(self.robot_id)):
            p.setCollisionFilterPair(self.robot_id, target_id, i, -1, enableCollision=1)
            
        self.is_grabbing = False


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

#Włączanie i wyłącznie ruchu kamerą   
    def toggle_c(self, keys):
        if ord('c') in keys:  
            if keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                self.custom_camera_enabled = not self.custom_camera_enabled
                
#Zależność ruchu kamery od ruchu myszki
    def handle_camera(self, keys):
        self.toggle_c(keys)
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


class tryb_uczenia:
    def __init__(self, robot, target_id):
        self.robot = robot
        self.target_id = target_id
        
        self.previous_click = 0
        self.uczenie_toggle = False
        self.przycisk_uczenie_on = p.addUserDebugParameter("Tryb Uczenia", 1, 0, 0)
        
        self.btn_play = p.addUserDebugParameter("Odtworz Nagranie", 1, 0, 0)
        self.prev_play_clicks = 0
        

        self.btn_reset_cube = p.addUserDebugParameter("Resetuj Klocek", 1, 0, 0)
        self.prev_reset_clicks = 0
        
        self.is_playing = False
        
        self.trajectory = []
        self.playback_step = 0
        self.prep_step = 0  
        
        self.cube_start_state = None
#Przycisk odpowiedzialny za tryb uczenia
    def przycisk_uczenia(self):
        self.uczenie_on = p.readUserDebugParameter(self.przycisk_uczenie_on)
        current_click = self.uczenie_on
        
        if current_click > self.previous_click:
            self.previous_click = current_click
            
            if not self.is_playing:
                self.uczenie_toggle = not self.uczenie_toggle
                
                if self.uczenie_toggle:
                    print("Tryb uczenia włączony (Nagrywanie)")
                    self.trajectory = []
                    if self.target_id is not None:
                        pos, ori = p.getBasePositionAndOrientation(self.target_id)
                        self.cube_start_state = (pos, ori)
                        print("Zapisano pozycję startową klocka.")
                else:
                    print(f"Tryb uczenia wyłączony. Zapisano {len(self.trajectory)} klatek ruchu.")
#Aktualizacja trybu uczenia
    def update(self):
        self.przycisk_uczenia()
        
        reset_clicks = p.readUserDebugParameter(self.btn_reset_cube)
        if reset_clicks > self.prev_reset_clicks:
            self.prev_reset_clicks = reset_clicks
            
            if self.target_id is not None and self.cube_start_state is not None:
                p.resetBasePositionAndOrientation(self.target_id, self.cube_start_state[0], self.cube_start_state[1])
                p.resetBaseVelocity(self.target_id, [0,0,0], [0,0,0])
                print("Zresetowano pozycję klocka do momentu rozpoczęcia nagrywania.")
            else:
                print("Brak zapisanej pozycji klocka (musisz najpierw rozpocząć nagrywanie).")

        play_clicks = p.readUserDebugParameter(self.btn_play)
        if play_clicks > self.prev_play_clicks:
            self.prev_play_clicks = play_clicks
            
            if not self.uczenie_toggle and len(self.trajectory) > 0 and not self.is_playing:
                self.is_playing = True
                self.playback_step = 0
                self.prep_step = 120 
                
                if self.robot.is_grabbing:
                    self.robot.release(self.target_id)

        #Nagrywanie
        if self.uczenie_toggle:
            current_state = {
                'joints': [],
                'is_grabbing': self.robot.is_grabbing
            }
            for joint_idx in self.robot.movable_joints:
                joint_state = p.getJointState(self.robot.robot_id, joint_idx)
                current_state['joints'].append(joint_state[0])
            self.trajectory.append(current_state)

        #Odtwarzanie
        elif self.is_playing:
            if self.prep_step > 0:
                start_state = self.trajectory[0]
                self._apply_state(start_state, apply_grab=False) 
                self.prep_step -= 1
                if self.prep_step == 0:
                    print("Odtwarzanie sekwencji")
            else:
                if self.playback_step < len(self.trajectory):
                    state = self.trajectory[self.playback_step]
                    self._apply_state(state, apply_grab=True)
                    self.playback_step += 1
                else:
                    print("Odtwarzanie zakończone.")
                    self.is_playing = False
                    
#Aktualizacja kątów przegubów w trakcie symulacji
    def _apply_state(self, state, apply_grab=True):
        for i, joint_idx in enumerate(self.robot.movable_joints):
            p.setJointMotorControl2(
                bodyIndex=self.robot.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=state['joints'][i],
                force=400
            )
 
#Uwzględnienie chwytania w trybie uczenia           
        if apply_grab:
            recorded_grab = state['is_grabbing']
            if recorded_grab and not self.robot.is_grabbing:
                self.robot.grab(self.target_id)
            elif not recorded_grab and self.robot.is_grabbing:
                self.robot.release(self.target_id)


class Simulation:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("podloga.urdf", useFixedBase=True)
        
        try:
            self.cube_id = p.loadURDF("kostka.urdf", [0, -0.4, 0.4])
        except p.error:
            print("Nie udało się wczytać kostki.")
            self.cube_id = None
            
        self.robot = None
        self.uczenie = None
        self.camera = Camera()
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        
#Wczytanie atrybutów klasy robot       
    def add_robot(self, robot):
        self.robot = robot
        self.uczenie = tryb_uczenia(self.robot, self.cube_id)

#Obsługa przycisków 'm' oraz spacji
    def handle_keyboard(self, keys):
        if self.uczenie and self.uczenie.is_playing:
            return

        space_key = ord(' ')
        m_key = ord('m')
        
        if space_key in keys and keys[space_key] & p.KEY_WAS_TRIGGERED:
            self.robot.toggle_grab(self.cube_id)
                
        if m_key in keys and keys[m_key] & p.KEY_WAS_TRIGGERED:
            self.robot.toggle_mode()
            
#Właściwa pętla symulacji
    def run(self):
        try:
            print("\nSterowanie:")
            print(" [SPACJA] - Chwyć / Puść kostkę")
            print(" [M]      - Przełącz tryb FK / IK")
            print(" [C]      - Aktywuj sterowanie kamerą (przytrzymaj LPM)\n")
            
            while True:
                keys = p.getKeyboardEvents()
                self.handle_keyboard(keys)
                self.camera.handle_camera(keys)
                
                if self.robot is not None:
                    if not self.uczenie.is_playing:
                        self.robot.update_from_sliders()
                        
                self.uczenie.update()
                
                p.stepSimulation()   
                time.sleep(1.0 / 240.0)
                
        except KeyboardInterrupt:
            print("\nZakończono symulację.")
        finally:
            self.cleanup()
#Koniec symulacji
    def cleanup(self):
        p.disconnect()

if __name__ == "__main__":
    sim = Simulation()
    my_robot = Robot("model1.urdf")
    sim.add_robot(my_robot)
    sim.run()