import pybullet as p
import pybullet_data
import time

# 1. Uruchomienie symulatora z interfejsem graficznym
physicsClient = p.connect(p.GUI)

# Ustawienie ścieżki, aby móc załadować domyślne podłoże (plane.urdf)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Ustawienie grawitacji (oś Z w dół)
p.setGravity(0, 0, -9.81)

# 2. Wczytanie płaskiego podłoża
planeId = p.loadURDF("plane.urdf")

# 3. Wczytanie Twojego robota
# Parametr useFixedBase=True jest kluczowy dla ramion robotycznych, 
# aby podstawa była "przykręcona" do podłoża i robot się nie przewrócił.
start_pos = [0, 0, 0] # Pozycja początkowa (x, y, z)
start_ori = p.getQuaternionFromEuler([0, 0, 0]) # Orientacja początkowa (roll, pitch, yaw)

try:
    print("Wczytywanie model1.urdf...")
    robotId = p.loadURDF("model1.urdf", start_pos, start_ori, useFixedBase=True)
    print("Robot wczytany pomyślnie!")
    
except p.error as e:
    print(f"Błąd podczas wczytywania URDF: {e}")
    print("Upewnij się, że plik model1.urdf znajduje się w folderze ze skryptem.")

# 4. Pętla symulacji działająca w nieskończoność
print("Symulacja uruchomiona. Naciśnij Ctrl+C w konsoli, aby zakończyć.")
try:
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0) # Standardowe taktowanie PyBullet (240 Hz)
except KeyboardInterrupt:
    print("Zakończono symulację.")

# Sprzątanie po zakończeniu
p.disconnect()