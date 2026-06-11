import pybullet as p
import time

p.connect(p.GUI)
p.loadURDF("plane.urdf")

yaw = 0.0
pitch = -20.0
dist = 5.0

while True:
    p.stepSimulation()
    
    # Pobieranie zdarzeń myszy z interfejsu
    mouse_events = p.getMouseEvents()
    
    for event in mouse_events:
        # Struktura eventu: (eventType, mousePosX, mousePosY, buttonIndex, buttonState)
        # eventType: 1 (ruch myszką), 2 (kliknięcie przycisku)
        
        # Przykład: wypisywanie zdarzeń do konsoli w celach debugowania
        print(f"Typ: {event[0]}, Pozycja X: {event[1]}, Pozycja Y: {event[2]}")
        
        # Tutaj możesz dopisać własną matematykę (obliczanie delty X/Y) 
        # i modyfikować zmienne yaw/pitch, a następnie wywoływać:
        # p.resetDebugVisualizerCamera(dist, yaw, pitch, [0, 0, 0])

    time.sleep(1./240.)