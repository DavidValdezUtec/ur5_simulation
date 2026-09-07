"""Deteccion y lanzamiento de los dispositivos hapticos (Geomagic Touch,
via el paquete omni_common). HapticModule es un objeto de composicion:
no conoce widgets de la UI, solo expone el estado ready de cada haptico
para que la ventana actualice sus LEDs."""
import os
import subprocess

from ur5_panel.funciones import buscar_dispositivos as _buscar_dispositivos_hw

from .procesos import terminar_proceso_gracefully


class HapticModule:
    def __init__(self):
        self.multi_process = None
        self.single_process = None
        self.haptic1_ready = False
        self.haptic2_ready = False

    def buscar(self):
        """Ejecuta la deteccion de dispositivos y actualiza haptic1_ready/
        haptic2_ready. Devuelve el dict que entrega funciones.buscar_dispositivos."""
        resultado = _buscar_dispositivos_hw()
        num = resultado["num_dispositivos"]
        self.haptic1_ready = num >= 1
        self.haptic2_ready = num >= 2
        return resultado

    def lanzar_launch(self):
        """Lanza el launch de dispositivos hápticos (dual o single) según
        cuantos esten listos."""
        if self.haptic1_ready and self.haptic2_ready:
            if self.multi_process is not None:
                print("Haptic launch ya está corriendo")
                return
            try:
                self.multi_process = subprocess.Popen(
                    ['ros2', 'launch', 'omni_common', 'ddual.launch.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                print(f"Haptic launch iniciado (PID: {self.multi_process.pid})")
            except Exception as e:
                print(f"Error al lanzar haptic: {e}")
        elif self.haptic1_ready:
            if self.single_process is not None:
                print("Haptic launch ya está corriendo")
                return
            try:
                self.single_process = subprocess.Popen(
                    ['ros2', 'launch', 'omni_common', 'single_omni_state.launch.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                print(f"Haptic launch iniciado (PID: {self.single_process.pid})")
            except Exception as e:
                print(f"Error al lanzar haptic: {e}")
        else:
            print("No hay dispositivos hápticos disponibles para lanzar")

    def detener_launch(self):
        """Detiene los launches hápticos activos (dual y/o single)."""
        if self.multi_process is not None:
            terminar_proceso_gracefully(self.multi_process, 'multi_haptic')
            self.multi_process = None
        if self.single_process is not None:
            terminar_proceso_gracefully(self.single_process, 'single_haptic')
            self.single_process = None
