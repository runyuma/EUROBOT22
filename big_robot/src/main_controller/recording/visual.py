import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv("recording4_28.csv")
def csv_tolist(col_name):
    return df[col_name].tolist()

AMCL = 0
EKF = 1
def plot_x():
    time_index = csv_tolist("time_index")
    time_index = [i/30.0 for i in time_index]
    if AMCL:
        amcl_x = csv_tolist("amcl_x")
        plt.plot(time_index, amcl_x, label="amcl")
        ekf_x = csv_tolist("ekf_x")
        plt.plot(time_index, ekf_x, label="ekf")
    if EKF:
        odom_x = csv_tolist("orign_x")
        plt.plot(time_index, odom_x, label="odom")
        ekf_x = csv_tolist("ekf_x")
        plt.plot(time_index, ekf_x, label="ekf")


    plt.title("X direction localization")
    plt.legend()
    plt.show()
def plot_y():
    time_index = csv_tolist("time_index")
    time_index = [i / 30.0 for i in time_index]
    if AMCL:
        amcl_x = csv_tolist("amcl_y")
        plt.plot(time_index, amcl_x, label="amcl")
        ekf_x = csv_tolist("ekf_y")
        plt.plot(time_index, ekf_x, label="ekf")
    if EKF:
        odom_x = csv_tolist("orign_y")
        plt.plot(time_index, odom_x, label="odom")
        ekf_x = csv_tolist("ekf_y")
        plt.plot(time_index, ekf_x, label="ekf")


    plt.title("Y direction localization")
    plt.legend()
    plt.show()
def plot_z():
    time_index = csv_tolist("time_index")
    time_index = [i / 30.0 for i in time_index]
    if AMCL:
        amcl_x = csv_tolist("amcl_z")
        plt.plot(time_index, amcl_x, label="amcl")
        ekf_x = csv_tolist("ekf_z")
        plt.plot(time_index, ekf_x, label="ekf")
    if EKF:
        odom_x = csv_tolist("orign_z")
        plt.plot(time_index, odom_x, label="odom")
        ekf_x = csv_tolist("ekf_z")
        plt.plot(time_index, ekf_x, label="ekf")


    plt.title("Z direction localization")
    plt.legend()
    plt.show()
plot_z()