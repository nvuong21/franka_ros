import json
import numpy as np
import matplotlib.pyplot as plt
import os

def read_data(folder):
    parent = os.path.dirname(os.path.realpath("__file__"))
    package_folder = os.path.dirname(parent)
    save_folder = os.path.join("data", folder)
    save_folder = os.path.join(package_folder, save_folder)
    save_file = os.path.join(save_folder, "data.json")

    with open(save_file, "r") as f:
        data = json.load(f)

    return data

def plot_joint_response(data):
    num_point = len(data["q"])
    q0 = [i[0] for i in data["q"]]
    q0d = [i[0] for i in data["q_d"]]
    time = [i for i in data["time"]]
    print(len(q0), len(time), len(q0d))
    plt.plot(time[:800], q0[:800])
    plt.plot(time[:800], q0d[:800])

def plot_torque_response(data, joint_index=0):
    tau = [i[joint_index] for i in data["tau_J"]]
    taud = [i[joint_index] for i in data["tau_J_d"]]
    time = [i for i in data["time"]]
    # print(len(tau), len(time))
    N = np.min(np.array([len(tau), len(time)]))
    # print N
    plt.plot(time[:N], tau[:N])
    #plt.plot(time[:N], taud[:N])


folder = "car_linear_001_T8"
data = read_data(folder)
# print(data)
plot_joint_response(data)
#plot_torque_response(data)
plt.show()
