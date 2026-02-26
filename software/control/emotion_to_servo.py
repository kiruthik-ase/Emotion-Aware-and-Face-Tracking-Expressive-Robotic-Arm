import serial
import time

ser = serial.Serial("COM7", 115200, timeout=1)
time.sleep(2)

print("Connected to Arduino")

# Test poses
poses = [
    ("Neutral", 90, 0, 90),
    ("Happy",   100, 20, 120),
    ("Sad",     80, 50, 60),
    ("Angry",   110, 35, 40),
]

for name, b, s, e in poses:
    print("Sending:", name)
    ser.write(f"B:{b} S:{s} E:{e}\n".encode())
    time.sleep(2)

ser.write(b"B:90 S:0 E:90\n")
print("Done")
