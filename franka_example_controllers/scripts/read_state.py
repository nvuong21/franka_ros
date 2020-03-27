import os
import json
import rospy
from franka_msgs.msg import FrankaState
from std_msgs.msg import Bool
from franka_example_controllers.msg import JointTorqueComparison


class StateSaver:
    def __init__(self, controller_ns):
        rospy.init_node("read_state");
        self._state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self._callback)
        self._control_pub = rospy.Publisher(
            controller_ns,
            Bool, queue_size=1)

        rospy.sleep(0.5)

        self._record = False
        self._data = {
            "q"         : [],
            "q_d"       : [],
            "dq"        : [],
            "dq_d"      : [],
            "ddq_d"     : [],
            "tau_J"     : [],
            "dtau_J"    : [],
            "tau_J_d"   : [],
            "K_F_ext_hat_K": [],
            "elbow"     : [],
            "elbow_d"   : [],
            "O_F_ext_hat_K" : [],
            "O_dP_EE_d" : [],
            "O_dP_EE_c" : [],
            "O_ddP_EE_c": [],
            "tau_ext_hat_filtered": [],
            "O_T_EE"    : [],
            "O_T_EE_d"  : [],
            "O_T_EE_c"  : [],
            "time"      : [],
        }

        if "impedance" in controller_ns:
            self._torque_compare_sub = rospy.Subscriber(
                "/joint_impedance_example_controller/torque_comparison",
                JointTorqueComparison, self._torque_compare_callback)

            self._data["tau_error"] = []
            self._data["tau_commanded"] = []
            self._data["tau_measured"] = []
            self._data["rms_error"] = []

    def _callback(self, msg):
        # check if self._data exists
        if "_data" in dir(self):
            if self._record:
                self._data["q"].append(msg.q)
                self._data["q_d"].append(msg.q_d)
                self._data["dq"].append(msg.dq)
                self._data["dq_d"].append(msg.dq_d)
                self._data["ddq_d"].append(msg.ddq_d)
                self._data["tau_J"].append(msg.tau_J)
                self._data["tau_J_d"].append(msg.tau_J_d)
                self._data["dtau_J"].append(msg.dtau_J)
                self._data["K_F_ext_hat_K"].append(msg.K_F_ext_hat_K)
                self._data["elbow"].append(msg.elbow)
                self._data["elbow_d"].append(msg.elbow_d)
                self._data["O_F_ext_hat_K"].append(msg.O_F_ext_hat_K)
                self._data["O_dP_EE_d"].append(msg.O_dP_EE_d)
                self._data["O_dP_EE_c"].append(msg.O_dP_EE_c)
                self._data["tau_ext_hat_filtered"].append(msg.tau_ext_hat_filtered)
                self._data["O_T_EE"].append(msg.O_T_EE)
                self._data["O_T_EE_d"].append(msg.O_T_EE_d)
                self._data["O_T_EE_c"].append(msg.O_T_EE_c)
                self._data["time"].append(msg.time)

    def _torque_compare_callback(self, msg):
        if "_data" in dir(self):
            if self._record:
                self._data["tau_error"].append(msg.tau_error)
                self._data["tau_commanded"].append(msg.tau_commanded)
                self._data["tau_measured"].append(msg.tau_measured)
                self._data["rms_error"].append(msg.root_mean_square_error)

    def save(self, save_dir):
        with open(save_dir, "w") as f:
            json.dump(self._data, f)

    def start_controller(self):
        msg = Bool()
        self._control_pub.publish(msg)

    def start_record_data(self):
        self._record = True

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--exp_name', '-n', type=str, required=True)
    args = parser.parse_args()
    params = vars(args)


    parent = os.path.dirname(os.path.realpath("__file__"))
    package_folder = os.path.dirname(parent)
    save_folder = os.path.join("data", params["exp_name"])
    save_folder = os.path.join(package_folder, save_folder)
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    save_file = os.path.join(save_folder, "data.json")
    saver = StateSaver("/joint_position_example_controller/command")
#    saver = StateSaver("/cartesian_pose_example_controller/command")
#    saver = StateSaver("/joint_impedance_example_controller/command")
    saver.start_record_data()
    rospy.sleep(0.5)
    saver.start_controller()
    rospy.sleep(5)
    saver.save(save_file)
