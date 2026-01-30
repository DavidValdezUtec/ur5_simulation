import sys
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QRadioButton, QLabel)

class MatrizSeleccion(QWidget):
    def __init__(self):
        super().__init__()
        
        # Configuración de la matriz
        self.num_filas = 3
        self.num_cols = 3
        
        # Matriz para guardar referencias a los botones: self.botones[fila][col]
        self.botones = [] 
        
        # Estado interno: índice de la columna seleccionada para cada fila.
        # Inicializamos en diagonal (0, 1, 2) para que empiece sin conflictos.
        self.seleccion_actual = [i for i in range(self.num_filas)]

        self.init_ui()

    def init_ui(self):
        layout_principal = QVBoxLayout()
        
        # Crear las 3 filas
        for i in range(self.num_filas):
            grupo = QGroupBox()
            layout_grupo = QHBoxLayout()
            botones_fila = []
            
            for j in range(self.num_cols):
                radio = QRadioButton()
                
                # Marcar si coincide con el estado inicial
                if self.seleccion_actual[i] == j:
                    radio.setChecked(True)
                
                # Conectar la señal 'clicked'. Usamos lambda para pasar fila y col.
                # IMPORTANTE: Usar 'clicked' y no 'toggled' evita bucles recursivos
                # cuando cambiamos los botones por código.
                radio.clicked.connect(lambda checked, r=i, c=j: self.manejar_clic(r, c))
                
                layout_grupo.addWidget(radio)
                botones_fila.append(radio)
            
            grupo.setLayout(layout_grupo)
            layout_principal.addWidget(grupo)
            self.botones.append(botones_fila)

        self.setLayout(layout_principal)
        self.setWindowTitle('Matriz de Selección Única')
        self.resize(400, 300)

    def manejar_clic(self, fila_clicada, nueva_columna):
        # 1. Identificar qué columna tenía esta fila ANTES del cambio
        columna_antigua = self.seleccion_actual[fila_clicada]
        
        # Si el usuario clicó la que ya estaba seleccionada, no hacemos nada
        if columna_antigua == nueva_columna:
            return

        # 2. Buscar si hay otra fila que ya tenga la 'nueva_columna' (Conflicto)
        fila_en_conflicto = -1
        for r in range(self.num_filas):
            if r != fila_clicada and self.seleccion_actual[r] == nueva_columna:
                fila_en_conflicto = r
                break
        
        # 3. Lógica de Intercambio
        if fila_en_conflicto != -1:
            # La fila en conflicto toma la columna antigua de la fila clicada
            print(f"Conflicto en Fila {fila_en_conflicto+1}. Intercambiando a Columna {columna_antigua+1}")
            
            # Actualizamos la UI de la fila conflictiva
            self.botones[fila_en_conflicto][columna_antigua].setChecked(True)
            # Actualizamos el estado interno de la fila conflictiva
            self.seleccion_actual[fila_en_conflicto] = columna_antigua

        # 4. Actualizar el estado interno de la fila clicada
        # (La UI ya se actualizó sola al hacer clic)
        self.seleccion_actual[fila_clicada] = nueva_columna
        
        print(f"Estado actual: {self.seleccion_actual}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MatrizSeleccion()
    ex.show()
    sys.exit(app.exec_())