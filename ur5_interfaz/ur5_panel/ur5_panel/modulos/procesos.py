"""Utilidad compartida para terminar procesos externos (ros2 launch/run)
lanzados con subprocess.Popen. La usan todos los modulos que manejan un
proceso propio (camera, haptic, robots_launch)."""
import os
import signal
import subprocess


def terminar_proceso_gracefully(proceso, nombre):
    """Termina un proceso de forma gradual: SIGINT -> SIGTERM -> SIGKILL."""
    if proceso is None:
        return

    try:
        pgid = os.getpgid(proceso.pid)

        print(f"[Shutdown] Enviando SIGINT a {nombre}...")
        os.killpg(pgid, signal.SIGINT)
        try:
            proceso.wait(timeout=8)
            print(f"[Shutdown] {nombre} cerrado correctamente")
            return
        except subprocess.TimeoutExpired:
            print(f"[Shutdown] {nombre} no respondió a SIGINT, escalando...")

        print(f"[Shutdown] Enviando SIGTERM a {nombre}...")
        os.killpg(pgid, signal.SIGTERM)
        try:
            proceso.wait(timeout=5)
            print(f"[Shutdown] {nombre} cerrado con SIGTERM")
            return
        except subprocess.TimeoutExpired:
            print(f"[Shutdown] {nombre} no respondió a SIGTERM, forzando cierre...")

        print(f"[Shutdown] Enviando SIGKILL a {nombre}...")
        os.killpg(pgid, signal.SIGKILL)
        proceso.wait(timeout=2)
        print(f"[Shutdown] {nombre} terminado forzosamente")

    except ProcessLookupError:
        print(f"[Shutdown] {nombre} ya no existe")
    except Exception as e:
        print(f"[Shutdown] Error al detener {nombre}: {e}")
