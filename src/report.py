import json
import matplotlib.pyplot as plt

with open("json_results.txt") as f:
    data = json.load(f)

plt.figure()
plt.subplot(311)
plt.plot(data["sim"]["xCar"], data["sim"]["yCar"],
         data["waypoints"]["nodes"]["x"], data["waypoints"]["nodes"]["y"], "o",
         data["waypoints"]["fit"]["x"], data["waypoints"]["fit"]["y"])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend(["Sim", "Waypoint Nodes", "Waypoint Fit"])

plt.subplot(312)
plt.plot(data["sim"]["vCar"])
plt.xlabel("Index")
plt.ylabel("vCar (m/s)")

plt.subplot(313)
plt.plot(data["sim"]["aSteer"])
plt.xlabel("Index")
plt.ylabel("aSteer (kph)")

plt.figure()
plt.plot(data["sim"]["cte"])
plt.plot(data["sim"]["err_psiCar"])
plt.plot(data["sim"]["err_vCar"])
plt.plot(data["sim"]["mag_aSteer"])
plt.plot(data["sim"]["mag_gAccel"])
plt.legend(["cte", "err_psiCar", "err_vCar", "mag_aSteer", "mag_gAccel" ])

plt.show()

