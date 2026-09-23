# import sys
# from PyQt5.QtCore import Qt
# from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QVBoxLayout, QLabel
# from ur5_interfaz_library.RvizWrapper import RVizQtWidget

# class VentanaFlotante(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Ventana Flotante")
#         self.resize(300, 200)
        
#         # Indica que es una ventana independiente (flotante)
#         self.setWindowFlags(Qt.Window) 
#         self.rviz_widget = RVizQtWidget(
#                       urdf_path="", 
#                       description_topic="",  # Sin tópico inicial - robots se agregan dinámicamente
#                       fixed_frame="world"
#                   )
#         layout = QVBoxLayout()
#         label = QLabel("¡Hola! Soy una ventana flotante.")
#         layout.addWidget(label)
#         layout.addWidget(self.rviz_widget)
#         self.setLayout(layout)

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Ventana Principal")
#         self.resize(500, 400)
        

#         # Botón para abrir la ventana flotante
#         btn = QPushButton("Abrir Ventana Flotante", self)
#         btn.setGeometry(150, 150, 200, 50)
#         btn.clicked.connect(self.mostrar_flotante)

#         # Guardar referencia para evitar que el recolector de basura la elimine
#         self.ventana_flotante = None

#     def mostrar_flotante(self):
#         if self.ventana_flotante is None or not self.ventana_flotante.isVisible():
#             self.ventana_flotante = VentanaFlotante()
#             self.ventana_flotante.show()

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     ventana_principal = MainWindow()
#     ventana_principal.show()
#     sys.exit(app.exec_())

import sys
from PyQt5.QtWidgets import (
    QApplication, 
    QMainWindow, 
    QTextEdit, 
    QDockWidget, 
    QWidget, 
    QVBoxLayout, 
    QPushButton, 
    QLabel
)
from PyQt5.QtCore import Qt

class VentanaPrincipal(QMainWindow):
    def __init__(self):
        super().__init__()
        self.inicializar_ui()

    def inicializar_ui(self):
        # 1. Configurar la ventana principal
        self.setWindowTitle("Ejemplo Completo de QDockWidget")
        self.setGeometry(100, 100, 800, 500)

        # 2. Crear el widget central (un editor de texto)
        self.editor = QTextEdit()
        self.editor.setPlaceholderText("Escribe algo aquí...")
        self.setCentralWidget(self.editor)

        # 3. Crear y configurar el QDockWidget
        self.crear_dock_herramientas()

    def crear_dock_herramientas(self):
        # Instanciar el Dock con un título
        dock = QDockWidget("Herramientas de Texto", self)
        
        # Configurar restricciones (Permitir cerrar, mover y flotar)
        dock.setFeatures(QDockWidget.DockWidgetClosable | 
                         QDockWidget.DockWidgetMovable | 
                         QDockWidget.DockWidgetFloatable)
        
        # Limitar las zonas donde se puede acoplar (Solo Izquierda y Derecha)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)

        # 4. Crear el contenido interno del Dock (un layout con botones)
        widget_interno = QWidget()
        layout = QVBoxLayout()

        etiqueta = QLabel("Cambiar color de fuente:")
        etiqueta.setAlignment(Qt.AlignCenter)
        layout.addWidget(etiqueta)

        # Botón 1: Texto Rojo
        btn_rojo = QPushButton("Rojo")
        btn_rojo.clicked.connect(lambda: self.cambiar_color("red"))
        layout.addWidget(btn_rojo)

        # Botón 2: Texto Azul
        btn_azul = QPushButton("Azul")
        btn_azul.clicked.connect(lambda: self.cambiar_color("blue"))
        layout.addWidget(btn_azul)

        # Botón 3: Resetear color
        btn_reset = QPushButton("Negro (Default)")
        btn_reset.clicked.connect(lambda: self.cambiar_color("black"))
        layout.addWidget(btn_reset)

        # Añadir un espacio estirable para empujar los botones hacia arriba
        layout.addStretch()
        
        # Asignar el layout al widget interno y este al Dock
        widget_interno.setLayout(layout)
        dock.setWidget(widget_interno)

        # 5. Añadir el Dock a la ventana principal en el lado izquierdo
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

    def cambiar_color(self, color):
        # Cambia el color del texto seleccionado o del nuevo texto
        self.editor.setTextColor(Qt.GlobalColor.__dict__[color.capitalize()] if hasattr(Qt, color.capitalize()) else Qt.black)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = VentanaPrincipal()
    ventana.show()
    sys.exit(app.exec_())
