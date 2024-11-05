import Mavcode
import threading
import sys

Telemetry_thread = threading.Thread(target=Mavcode.Telemetry)
Telemetry_thread.start()
while True:
    input=input("Exit?")
    if input.upper()=="Y":
        Telemetry_thread.join()
        sys.exit()