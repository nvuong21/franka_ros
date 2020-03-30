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

def plot_joint_response(data, idx=0):
    num_point = len(data["q"])
    q0 = [i[idx] for i in data["q"]]
    q0d = [i[idx] for i in data["q_d"]]
    time = [i for i in data["time"]]
    N = np.min([len(q0), len(time), len(q0d)])
    print(compute_error(q0, q0d))

    plt.plot(time[:N], q0[:N])
    plt.plot(time[:N], q0d[:N])
    plt.xlabel("time")
    plt.ylabel("q{}".format(idx))
    plt.legend(["q{}".format(idx), "q{}d".format(idx)])

def plot_cartesian_response(data, axis="x"):
    if axis=="x":
        idx = 12
    elif axis=="y":
        idx = 13
    else:
        idx = 14

    x = [i[idx] for i in data["O_T_EE"]]
    xd = [i[idx] for i in data["O_T_EE_d"] ]
    t = [i for i in data["time"]]
    N = np.min([len(x), len(xd), len(t)])
    # print(compute_error(x, xd))

    error = calc_ss_error(x[:N], xd[:N], t[:N])
    print("steady state error {}: {}".format(axis, error))

    plt.plot(t[:N], np.array(x[:N]) - np.array(xd[:N]))
    # plt.plot(t[:N], xd[:N])
    plt.xlabel("time")
    plt.ylabel(axis)
    plt.legend([axis, axis + "d"])


def compute_error(x, xd):
    N = np.min([len(x), len(xd)])
    rms = np.linalg.norm(np.array(xd[: N-1]) - np.array(x[1:N])) / (N - 1)
    max_error = np.max(np.abs(np.array(xd[: N-1]) - np.array(x[1:N])))
    return rms, max_error

def plot_torque_response(data, joint_index=0):
    tau = [i[joint_index] for i in data["tau_J"]]
    taud = [i[joint_index] for i in data["tau_J_d"]]
    time = [i for i in data["time"]]
    # print(len(tau), len(time))
    N = np.min(np.array([len(tau), len(time)]))
    # print N
    plt.plot(time[:N], tau[:N])
    #plt.plot(time[:N], taud[:N])

def calc_ss_error(q, qd, t, length=1):
    # q =
    # arr_length =
    tend = t[-1]
    tstart = tend - length
    # print(tstart, tend)

    q1 = []
    qd1 = []

    for i in range(len(t)):
        if t[i] >= tstart and t[i] <= tend:
            q1.append(q[i])
            qd1.append(qd[i])

    error = 0
    for i in range(len(q1)):
        error += np.abs(q1[i] - qd1[i])
    return error / len(q1)

folder = "cartesian_1_linear_001"
data = read_data(folder)
# print(data)
# plot_torque_response(data)
# plot_joint_response(data, idx=3)
plt.subplot(311)
plot_cartesian_response(data, axis="x")
plt.subplot(312)
plot_cartesian_response(data, axis="y")
plt.subplot(313)
plot_cartesian_response(data, axis="z")

# folder = "car_linear_001_T8"
# data = read_data(folder)
# # print(data)
# plot_joint_response(data)
# #plot_torque_response(data)
plt.show()

# joint 4 and 5, 6 large error
