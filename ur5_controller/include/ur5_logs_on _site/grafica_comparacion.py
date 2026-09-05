import pandas as pd
import matplotlib.pyplot as plt
import argparse
import pathlib
import sys
import re
import sympy as sp
from datetime import datetime


ubicacion = "/home/david/Documentos/Tesis/tesis_ws/src/ur5_simulation/ur5_controller/include/ur5_logs_on _site/"
df1 = pd.read_csv(f"{ubicacion}IMP_linea_recta.csv")
df2 = pd.read_csv(f"{ubicacion}SMC_linea_recta.csv")
df3 = pd.read_csv(f"{ubicacion}QP_linea_recta.csv")

print(len(df1["t"]), len(df2["t"]), len(df3["t"]))

# plt.plot(df1["t"], df1["x_des_x"], label="IMP", linewidth=1.5)
# plt.plot(df1["t"], df1["x_meas_x"], label="IMP", linewidth=1.5, ls="--", alpha=0.8)
# plt.plot(df1["t"], df2["x_meas_x"], label="SMC", linewidth=1.5, ls="--", alpha=0.8)
# plt.plot(df1["t"], df3["x_meas_x"], label="QP", linewidth=1.5, ls="--", alpha=0.8)
# plt.xlabel("Tiempo (s)", fontsize=12)
# plt.ylabel("Posición deseada (m)", fontsize=12)
# plt.title("Comparación de controladores en trayectoria lineal", fontsize=14)
# plt.legend()

# plt.show()
    