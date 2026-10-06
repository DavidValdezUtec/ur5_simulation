# importing the required libraries

from PyQt5.QtCore import * 
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtGui import * 
from PyQt5.QtWidgets import * 
import sys


class QToggleButton(QCheckBox):
    def __init__(
        self, 
        width=60,
        height=30,
        bg_color="#2c3e50",    
        circle_color="#ecf0f1",
        active_color="#3498db",
        animation_curve=QEasingCurve.OutBounce,
        debug=False
    ):
        """Boton de alternancia (toggle button) con animacion y colores personalizables.
        Hereda de QCheckBox y se comporta como tal (checked/unchecked).
        Se puede conectar a cualquier slot que acepte un bool (checked/unchecked).
        Parametros:
        - width, height: dimensiones del boton
        - bg_color: color de fondo cuando esta desactivado
        - circle_color: color del circulo
        - active_color: color de fondo cuando esta activado
        - animation_curve: curva de animacion (QEasingCurve: QEasingCurve.Linear, QEasingCurve.InOutQuad, OutInQuad, InOutCubic, OutInCubic, InOutQuart, OutInQuart, InOutQuint, OutInQuint, InOutSine, OutInSine, InOutExpo, OutInExpo, InOutCirc, OutInCirc, InOutElastic, OutInElastic, InOutBack, OutInBack, InOutBounce, OutInBounce)
        - debug: si True, imprime en consola cuando se hace click (checked/unchecked)"""
        QCheckBox.__init__(self)
        
        
        #PARAMETROS POR DEFECTO
        self.setFixedSize(width, height)
        #self.setCursor(Qt.PointingHandCursor)
        
        #COLORES
        self._bg_color = bg_color
        self._circle_color = circle_color
        self._active_color = active_color
        
        #CREAMOS LA ANIMACION
        self._circle_position = 2
        self.animation = QPropertyAnimation(self, b"circle_position", self)
        self.animation.setEasingCurve(animation_curve)
        self.animation.setDuration(500)
        
        #CONECTAR EL CAMBIO DE ESTADO DEL BOTON A LA ANIMACION
        self.stateChanged.connect(self.start_transition)
        self._debug = False
        self.debug = debug

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, enabled):
        enabled = bool(enabled)
        if enabled == self._debug:
            return

        self._debug = enabled
        if enabled:
            self.stateChanged.connect(self.button_clicked)
        else:
            self.stateChanged.disconnect(self.button_clicked)
    
    @pyqtProperty(float)
    def circle_position(self):
        return self._circle_position

    @circle_position.setter
    def circle_position(self, pos):
        self._circle_position = pos
        self.update()
        
    def start_transition(self, value):
        self.animation.stop()
        if value:
            self.animation.setEndValue(self.width() - self.height() + 2)
        else:
            self.animation.setEndValue(2)
        self.animation.start()
    
    def hitButton(self, pos: QPoint) -> bool:
            return self.contentsRect().contains(pos)
    
    def paintEvent(self, _event):
        # SETEAMOS EL PAINTER
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        
        # SETEAMOS COMO NO PEN LA LINEA DEL BORDE Y EL COLOR DE FONDO
        p.setPen(Qt.NoPen)
        
        # DIBUJAMOS EL RECTANGULO DE FONDO  
        rect = QRect(0, 0, self.width(), self.height())
        
        # CONDICIONALES PARA EL COLOR DE FONDO
        if not self.isChecked():
            #PINTAMOS EL FONDO 
            p.setBrush(QColor(self._bg_color))
            p.drawRoundedRect(0,0,rect.width(), rect.height(), rect.height()/2, rect.height()/2)
    
            #DIBUJAMOS CIRCULO
            p.setBrush(QColor(self._circle_color))
            p.drawEllipse(QRectF(self._circle_position, 2, rect.height()-4, rect.height()-4))
        else:
            #PINTAMOS EL FONDO 
            p.setBrush(QColor(self._active_color))
            p.drawRoundedRect(0,0,rect.width(), rect.height(), rect.height()/2, rect.height()/2)
    
            #DIBUJAMOS CIRCULO
            p.setBrush(QColor(self._circle_color))
            p.drawEllipse(QRectF(self._circle_position, 2, rect.height()-4, rect.height()-4))
        
        p.end()
        
    def button_clicked(self):
        if self.isChecked():
            print("Boton activado")
        else:
            print("Boton desactivado")

    


if __name__ == "__main__":
    
                
    app = QApplication(sys.argv)
    window = QWidget()
    window.resize(400, 300)
    window.setWindowTitle("Toggle Button Example")
    
    layout = QVBoxLayout(window)
    toggle_button = QToggleButton(debug = True)
    layout.addWidget(toggle_button, Qt.AlignCenter, Qt.AlignCenter)
    
    window.show()
    sys.exit(app.exec_())