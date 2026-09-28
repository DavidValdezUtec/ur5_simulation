"""Utilidad compartida para terminar procesos externos (ros2 launch/run)
lanzados con subprocess.Popen. La usan todos los modulos que manejan un
proceso propio (camera, haptic, robots_launch)."""
import os
import signal
import subprocess


def _matar_restos(pgid, nombre):
    """SIGKILL a lo que quede del grupo tras morir el proceso principal: si
    'ros2 launch' sale por SIGTERM no espera a sus hijos, y los del driver UR
    que estan reintentando conectar con un robot inalcanzable ignoran las
    senales hasta ~1 min (seguirian ocupando /rN y los puertos)."""
    try:
        os.killpg(pgid, signal.SIGKILL)
        print(f"[Shutdown] Procesos restantes de {nombre} terminados")
    except ProcessLookupError:
        pass


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
            _matar_restos(pgid, nombre)
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
