import os
import subprocess
import glob



def buscar_camara():
    video_dispositivos = glob.glob('/dev/video*')
    num_dispositivos = len(video_dispositivos)/2  # Asumiendo que cada dispositivo tiene un video asociado
    print(f"Dispositivos encontrados: {num_dispositivos}")
    for disp in video_dispositivos:
        print(f"  - {disp}")
    
    if num_dispositivos == 0:
        print("No se encontraron dispositivos de cámara conectados.")
        return {"dispositivos": [], "num_dispositivos": 0}
    return {"dispositivos": video_dispositivos, "num_dispositivos": num_dispositivos}


def buscar_dispositivos():
    """
    Busca dispositivos Geomagic Touch conectados via USB y verifica su calibración.
    
    Sistema: Ubuntu 22.04
    Dispositivos: Geomagic Touch
    Conexión: USB (ACM0, ACM1, etc)
    
    Returns:
        dict: Información sobre los dispositivos encontrados y su estado de calibración
    """
    print("Buscando dispositivos hápticos Geomagic Touch...")
    
    # Buscar dispositivos USB ACM
    dispositivos = glob.glob('/dev/ttyACM*')
    num_dispositivos = len(dispositivos)   
    
    # Buscar dispositivos USB video     
    
    
    print(f"Dispositivos encontrados: {num_dispositivos}")
    for disp in dispositivos:
        print(f"  - {disp}")
    
    if num_dispositivos == 0:
        print("No se encontraron dispositivos Geomagic Touch conectados.")
        return {"dispositivos": [], "calibrado": False, "num_dispositivos": 0}
    
    # Verificar calibración
    config_dir = "/home/david/.3dsystems/config"
    calibrado = os.path.exists(config_dir) and len(os.listdir(config_dir)) > 0
    
    if calibrado:
        print(f"Archivos de configuración encontrados en {config_dir}")
        return {"dispositivos": dispositivos, "calibrado": True, "num_dispositivos": num_dispositivos}
    else:
        print(f"No se encontraron archivos de configuración en {config_dir}")
        print("Es necesario calibrar los dispositivos.")
        
        # Intentar calibrar automáticamente
        calibrar_dispositivos(num_dispositivos)
        
        return {"dispositivos": dispositivos, "calibrado": False, "num_dispositivos": num_dispositivos}


def calibrar_dispositivos(num_dispositivos):
    """
    Calibra los dispositivos Geomagic Touch según el número de dispositivos conectados.
    
    Args:
        num_dispositivos (int): Número de dispositivos conectados
    """
    touch_driver_bin = os.environ.get(
        "TOUCH_DRIVER_BIN_DIR",
        os.path.expanduser("~/.local/share/geomagic/bin")
    )
    setup_path = os.path.join(touch_driver_bin, "Touch_HeadlessSetup")
    
    if not os.path.exists(setup_path):
        print(f"Error: No se encontró el ejecutable de calibración en {setup_path}")
        return False
    
    if num_dispositivos == 1:
        comando = f"{setup_path} auto=phantom1"
        print(f"Calibrando 1 dispositivo con: {comando}")
    elif num_dispositivos == 2:
        comando = f"{setup_path} auto=phantom1,phantom2"
        print(f"Calibrando 2 dispositivos con: {comando}")
    else:
        print(f"Advertencia: Se encontraron {num_dispositivos} dispositivos. Solo se soportan 1 o 2.")
        return False
    
    try:
        print("Iniciando calibración...")
        resultado = subprocess.run(comando, shell=True, capture_output=True, text=True, timeout=60)
        
        if resultado.returncode == 0:
            print("Calibración completada exitosamente.")
            print(resultado.stdout)
            return True
        else:
            print(f"Error durante la calibración. Código de salida: {resultado.returncode}")
            print(resultado.stderr)
            return False
    except subprocess.TimeoutExpired:
        print("Error: La calibración excedió el tiempo límite de 60 segundos.")
        return False
    except Exception as e:
        print(f"Error al ejecutar la calibración: {e}")
        return False


def verificar_configuracion():
    """
    Verifica si existen archivos de configuración de los dispositivos.
    
    Returns:
        bool: True si existe configuración válida, False en caso contrario
    """
    config_dir = "/home/david/.3dsystems/config"
    
    if not os.path.exists(config_dir):
        return False
    
    archivos = os.listdir(config_dir)
    return len(archivos) > 0
    


resultado = buscar_dispositivos()
resultado["num_dispositivos"]