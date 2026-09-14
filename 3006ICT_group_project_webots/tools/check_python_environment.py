"""Check Python packages before running the Webots controller."""

import sys

print("Python:", sys.executable)

for name in ["numpy", "cv2"]:
    try:
        module = __import__(name)
        print(name, "OK", getattr(module, "__version__", ""))
    except Exception as exc:
        print(name, "FAILED:", exc)

print()
print("Use the full path above in:")
print("Webots -> Tools -> Preferences -> General -> Python command")
print("If the path contains spaces, enclose the full path in double quotes.")
