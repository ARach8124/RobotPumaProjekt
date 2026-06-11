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

    def toggle_mode(self):
        self.use_ik = not self.use_ik
        aktualny_tryb = "IK (Kinematyka Odwrotna)" if self.use_ik else "FK (Kinematyka Prosta)"
        print(f"Przełączono na tryb: {aktualny_tryb}")

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
        valid_solution_found = False
        
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

    def get_link_index(self, link_name):
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode("utf-8") == link_name:
                return i
        return -1

    def toggle_grab(self, target_id):
        if not self.is_grabbing:
            self.grab(target_id)
        else:
            self.release(target_id)

    def grab(self, target_id):
        ee_idx = self.get_link_index("manipulator_link")
        if ee_idx == -1:
            print("Manipulator link not found!")
            return

        ee_state = p.getLinkState(self.robot_id, ee_idx)
        ee_pos = ee_state[4] 
        ee_ori = ee_state[5] 
        target_pos, target_ori = p.getBasePositionAndOrientation(target_id)

        dist = math.sqrt(sum((a - b)**2 for a, b in zip(ee_pos, target_pos)))

        if dist < 0.15:
            print("Chwytam obiekt w jego obecnej pozycji!")
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
                parentFrameOrientation=rel_ori
            )
            self.is_grabbing = True
        else:
            print("Obiekt jest za daleko!")

    def release(self, target_id):
        if self.grab_constraint_id is not None:
            p.removeConstraint(self.grab_constraint_id)
            self.grab_constraint_id = None
            
        for i in range(-1, p.getNumJoints(self.robot_id)):
            p.setCollisionFilterPair(self.robot_id, target_id, i, -1, enableCollision=1)
            
        self.is_grabbing = False


class Simulation:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        
        try:
            self.cube_id = p.loadURDF("kostka.urdf", [0, -0.4, 0.4])
        except p.error:
            print("Nie udało się wczytać kostki.")
            self.cube_id = None
            
        self.robot = None

    def add_robot(self, robot):
        self.robot = robot

    def handle_keyboard(self):
        keys = p.getKeyboardEvents()
        
        space_key = ord(' ')
        m_key = ord('m')
        
        if space_key in keys and keys[space_key] & p.KEY_WAS_TRIGGERED:
            self.robot.toggle_grab(self.cube_id)
                
        if m_key in keys and keys[m_key] & p.KEY_WAS_TRIGGERED:
            self.robot.toggle_mode()

    def run(self):
        try:
            print("\nSterowanie:")
            print(" [SPACJA] - Chwyć / Puść kostkę")
            print(" [M]      - Przełącz tryb FK / IK\n")
            
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
    sim = Simulation()
    my_robot = Robot("model1.urdf")
    sim.add_robot(my_robot)
    sim.run()