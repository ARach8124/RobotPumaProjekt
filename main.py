import pybullet as p
import pybullet_data
import time
import math

# 1. Uruchomienie symulatora z interfejsem graficznym
physicsClient = p.connect(p.GUI)

# Ustawienie ścieżki, aby móc załadować domyślne podłoże (plane.urdf)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Ustawienie grawitacji (oś Z w dół)
p.setGravity(0, 0, -9.81)

# Wczytanie płaskiego podłoża
planeId = p.loadURDF("plane.urdf")

# Wczytanie kostki
cubeId = p.loadURDF("kostka.urdf")

# Wczytanie robota
start_pos = [0, 0, 0] # Pozycja początkowa (x, y, z)
start_ori = p.getQuaternionFromEuler([0, 0, 0]) # Orientacja początkowa (roll, pitch, yaw)

try:
    print("Wczytywanie model1.urdf...")
    robotId = p.loadURDF("model1.urdf", start_pos, start_ori, useFixedBase=True)
    print("Robot wczytany pomyślnie!")
    
    # --- NOWA SEKCJA: Wyszukiwanie stawów i tworzenie suwaków ---
    num_joints = p.getNumJoints(robotId)
    movable_joints = []
    sliders = []
    
    print("\nZnalezione ruchome stawy:")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        joint_type = joint_info[2]
        joint_name = joint_info[1].decode("utf-8")
        
        if joint_type != p.JOINT_FIXED:
            movable_joints.append(i)
            
            # --- EXTRACT LIMITS ---
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            
            # PyBullet sets lower > upper (e.g. 0 and -1) if no limits are defined 
            # in the URDF (like for continuous joints). We need a fallback.
            if lower_limit < upper_limit:
                slider_min = lower_limit
                slider_max = upper_limit
            else:
                slider_min = -math.pi
                slider_max = math.pi
                
            # Create slider using the extracted limits
            slider = p.addUserDebugParameter(joint_name, slider_min, slider_max, 0)
            sliders.append(slider)
            
            print(f"- {joint_name} (Indeks: {i}, Zakres: {slider_min:.2f} do {slider_max:.2f})")

    # -------------------------------------------------------------
    
except p.error as e:
    print(f"Błąd podczas wczytywania URDF: {e}")
    print("Upewnij się, że plik model1.urdf znajduje się w folderze ze skryptem.")
    # Jeśli wystąpił błąd, kończymy działanie
    p.disconnect()
    exit()

# Pętla symulacji
print("\nSymulacja uruchomiona. Możesz sterować suwakami po prawej stronie. Naciśnij Ctrl+C w konsoli, aby zakończyć.")
try:
    while True:
        # --- NOWA SEKCJA: Odczyt suwaków i zadawanie kątów ---
        for i, joint_idx in enumerate(movable_joints):
            # Odczyt aktualnej wartości z przypisanego suwaka
            target_angle = p.readUserDebugParameter(sliders[i])
            
            # Wymuszenie pozycji na silniku w symulatorze
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=100  # Maksymalna siła silnika
            )
        # -----------------------------------------------------
        
        p.stepSimulation()
        time.sleep(1.0 / 240.0) # Standardowe taktowanie PyBullet (240 Hz)
except KeyboardInterrupt:
    print("Zakończono symulację.")

# Sprzątanie po zakończeniu
p.disconnect()