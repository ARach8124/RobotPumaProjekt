import pybullet as p
import pybullet_data
import time
import sys

try:
    print("Uruchamianie symulatora PyBullet...")
    # 1. Uruchomienie symulatora
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    print("Wczytywanie podłoża...")
    # 2. Wczytanie podłoża
    planeId = p.loadURDF("plane.urdf")

    print("Wczytywanie robota...")
    # 3. Wczytanie Twojego robota
    start_pos = [0, 0, 0.05]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Tutaj najczęściej występuje błąd, jeśli pliku nie ma w folderze!
    robotId = p.loadURDF("model1.urdf", start_pos, start_orientation, useFixedBase=True)

    print("\nSUKCES! Symulacja działa.")
    print("Aby zakończyć, po prostu zamknij okno graficzne PyBullet (iksem).")

    # 4. Główna pętla programu
    # Będzie działać tak długo, jak długo okno PyBullet jest otwarte
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1./240.)

except Exception as e:
    # 5. Przechwytywanie błędu
    print("\n" + "="*40)
    print("❌ WYSTĄPIŁ BŁĄD KRYTYCZNY:")
    print(e)
    print("="*40)

    
    # Ta linijka zatrzymuje zamykanie konsoli, dopóki nie wciśniesz Enter
    input("\nWciśnij Enter, aby zamknąć to okno...")

finally:
    # 6. Bezpieczne rozłączenie przy zamykaniu
    if p.isConnected():
        p.disconnect()
        