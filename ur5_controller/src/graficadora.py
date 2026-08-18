#obtener la data de /home/david/.ros/ur5_logs/
#se asume que la data esta en formato .csv y tiene cabeceras
#graficar con matplotlib x_des vs x_meas
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import pathlib
import sys
import re
import sympy as sp
from datetime import datetime

_TS_REGEX = re.compile(r"(\d{8}_\d{6})")

def _extraer_timestamp(path: pathlib.Path):
	"""Devuelve datetime si encuentra patron YYYYMMDD_HHMMSS en el nombre, si no None."""
	m = _TS_REGEX.search(path.name)
	if not m:
		return None
	stamp = m.group(1)
	try:
		return datetime.strptime(stamp, "%Y%m%d_%H%M%S")
	except ValueError:
		return None

def listar_csv_logs(log_dir: pathlib.Path):
	if not log_dir.exists():
		print(f"Directorio no existe: {log_dir}")
		return []
	archivos = [p for p in log_dir.glob("*.csv") if p.is_file()]
	# Ordenar por timestamp extraído; si no tiene timestamp -> va al inicio
	archivos.sort(key=lambda p: (_extraer_timestamp(p) is None, _extraer_timestamp(p) or datetime.min))
	return archivos

def cargar_csv(path: pathlib.Path) -> pd.DataFrame:
	try:
		df = pd.read_csv(path)
		return df
	except Exception as e:
		print(f"Error leyendo {path}: {e}")
		return pd.DataFrame()


def graficar_trayectoria_3d(df: pd.DataFrame, guardar: bool = False, salida: pathlib.Path | None = None):
	columnas_requeridas = ["x_des_x","x_des_y","x_des_z","x_meas_x","x_meas_y","x_meas_z"]
	for c in columnas_requeridas:
		if c not in df.columns:
			print(f"Columna faltante en CSV: {c}")
			return

	x_des = df[["x_des_x","x_des_y","x_des_z"]].values
	x_meas = df[["x_meas_x","x_meas_y","x_meas_z"]].values

	fig = plt.figure(figsize=(10, 8))
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(x_des[:, 0], x_des[:, 1], x_des[:, 2], label="Trayectoria Deseada", linewidth=1.5)
	ax.plot(x_meas[:, 0], x_meas[:, 1], x_meas[:, 2], label="Trayectoria Medida",ls="--",linewidth=1.5, alpha=0.8)
	ax.set_xlabel("X (m)", fontsize=12)
	ax.set_ylabel("Y (m)", fontsize=12)
	ax.set_zlabel("Z (m)", fontsize=12)
	ax.set_title("Trayectoria 3D del End-Effector", fontsize=14)
	ax.legend(fontsize=12)
	ax.grid(True, linestyle='--', alpha=0.4)

	if guardar:
		if salida is None:
			salida = pathlib.Path("trayectoria_3d.png")
		fig.savefig(salida)
		print(f"Gráfico guardado en {salida}")
	else:
		plt.show()

def graficar_esfuerzos(df: pd.DataFrame, guardar: bool = False, salida: pathlib.Path | None = None):
	columnas_requeridas = [f"u_control_{i}" for i in range(6)]
	for c in columnas_requeridas:
		if c not in df.columns:
			print(f"Columna faltante en CSV: {c}")
			return

	t = df["t"].values
	u_control = df[[f"u_control_{i}" for i in range(6)]].values

	fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
	for i in range(6):
		ax = axes[i//2, i%2]
		ax.plot(t, u_control[:, i], label=f"{sp.Symbol(f'u_control_{i+1}')}", linewidth=1.0)
		ax.set_ylabel(r"$\mu_{%d}$ (N·m)" % (i+1), fontsize=12)
		ax.grid(True, linestyle='--', alpha=0.4)
		ax.set_title(f"Esfuerzo de Control de la Articulación {i+1}", fontsize=12)
	for ax in axes[-1, :]:
		ax.set_xlabel("t (s)", fontsize=12)
	fig.suptitle("Esfuerzos de Control Articulares", fontsize=14)
	fig.tight_layout(rect=(0,0,1,0.97))

	if guardar:
		if salida is None:
			salida = pathlib.Path("esfuerzos_control.png")
		fig.savefig(salida)
		print(f"Gráfico guardado en {salida}")
	else:
		plt.show()


def graficar_posiciones(df: pd.DataFrame, guardar: bool = False, salida: pathlib.Path | None = None):
	columnas_requeridas = [
		"t",
		"x_des_x","x_des_y","x_des_z",
		"x_meas_x","x_meas_y","x_meas_z"
	]
	for c in columnas_requeridas:
		if c not in df.columns:
			print(f"Columna faltante en CSV: {c}")
			return

	t = df["t"].values
	x_des = df[["x_des_x","x_des_y","x_des_z"]].values
	x_meas = df[["x_meas_x","x_meas_y","x_meas_z"]].values

	fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
	etiquetas = ["X", "Y", "Z"]
	for i in range(3):
		ax = axes[i]
		ax.plot(t, x_des[:, i], label=f"${{{etiquetas[i].lower()}}}_{{des}}$", linewidth=1.0)
		ax.plot(t, x_meas[:, i], label=f"${{{etiquetas[i].lower()}}}_{{med}}$", linewidth=1.0, alpha=0.8, ls="--")
		ax.set_ylabel(f"Posición {etiquetas[i]} (m)", fontsize=12)
		ax.grid(True, linestyle='--', alpha=0.4)
		ax.legend(bbox_to_anchor=(1, 0.8), loc='upper left', fontsize=12)
	for ax in axes[:-1]:
		ax.set_xlabel("t (s)", fontsize=12)
	fig.suptitle("Posición Deseada vs Medida", fontsize=14)
	fig.tight_layout(rect=(0, 0, 0.88, 0.97))  # deja espacio para leyenda a la derecha

	if guardar:
		if salida is None:
			salida = pathlib.Path("posicion_vs_medida.png")
		fig.savefig(salida)
		print(f"Gráfico guardado en {salida}")
	else:
		plt.show()

def main():
	parser = argparse.ArgumentParser(description="Graficar x_des vs x_meas del último log CSV del UR5.")
	parser.add_argument("--dir", type=str, default=str(pathlib.Path.home()/".ros"/"ur5_logs"), help="Directorio de logs CSV")
	parser.add_argument("--archivo", type=str, default="", help="Ruta a CSV específico (si se proporciona se ignora --latest)")
	parser.add_argument("--latest", action="store_true", help="Usar el archivo más reciente automáticamente")
	parser.add_argument("--guardar", action="store_true", help="Guardar PNG en lugar de mostrar")
	parser.add_argument("--salida", type=str, default="", help="Ruta de salida para PNG")
	args = parser.parse_args()

	log_dir = pathlib.Path(args.dir)
	if args.archivo:
		csv_path = pathlib.Path(args.archivo)
		if not csv_path.exists():
			print(f"Archivo especificado no existe: {csv_path}")
			sys.exit(1)	
	else:
		archivos = listar_csv_logs(log_dir)
		print(archivos)
		if not archivos:
			print(f"No se encontraron CSV en {log_dir}")
			sys.exit(1)
		if args.latest or not args.archivo:
			csv_path = archivos[-1]  # último según orden temporal real
		else:
			csv_path = archivos[-1]

	print(f"Usando CSV: {csv_path}")
	df = cargar_csv(csv_path)
	if df.empty:
		print("DataFrame vacío, abortando")
		sys.exit(1)

	salida = pathlib.Path(args.salida) if args.salida else None
	graficar_trayectoria_3d(df, guardar=args.guardar, salida=salida)
	graficar_posiciones(df, guardar=args.guardar, salida=salida)
	graficar_esfuerzos(df, guardar=args.guardar, salida=salida)
if __name__ == "__main__":
	main()

'''  /home/david/.ros/ur5_logs/  - Graficadora de datos de UR5
Impedancia - Linea Recta
ur5_log_r1_20260226_211610.csv

Impedancia - Curva Helicoidal
ur5_log_r1_20260226_213503.csv

Sliding Mode - Linea Recta
ur5_log_r1_20260226_211041.csv

Sliding Mode - Curva Helicoidal
ur5_log_r1_20260226_212134.csv
'''
