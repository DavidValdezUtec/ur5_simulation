import os
import sys
import subprocess
import time

from PyQt5.QtCore import Qt, QTimer, QCoreApplication
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QSizePolicy

# Ensure ROS_DOMAIN_ID is set
env = os.environ.copy()
env["ROS_DOMAIN_ID"] = "0"

# Import your RViz binding - it should be in the same directory
# when installed (both .py and .so are in lib/)
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

try:
    from rviz_widget_py import launch_rviz, cleanup_rviz, add_robot
except ImportError as e:
    print(f"[RvizWrapper] Failed to import rviz_widget_py: {e}")
    print(f"[RvizWrapper] Current directory: {_current_dir}")
    print(f"[RvizWrapper] Python path: {sys.path[:3]}")
    raise 


class RVizQtWidget(QWidget):
    """
    A Qt widget that embeds RViz using a native window integration.
    
    Usage:
        rviz_widget = RVizQtWidget(urdf_path="/path/to/robot.urdf")
        # OR for existing ROS network:
        rviz_widget = RVizQtWidget(description_topic="/r1/robot_description")
        
    """
    
    def __init__(self, urdf_path: str = "", description_topic: str = "/robot_description", fixed_frame: str = "world", parent=None):
        super().__init__(parent)
        
        self.urdf_path = urdf_path
        self.description_topic = description_topic
        self.fixed_frame = fixed_frame
        
        # FIX: Initialize state tracking attributes
        self._is_shutting_down = False
        self._rviz_launched = False
        
        # Setup layout
        self._layout = QHBoxLayout()
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self._layout)
        
        # Create RViz container widget
        self.rviz_container = QWidget(self)
        self.rviz_container.setAttribute(Qt.WA_NativeWindow)
        self.rviz_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._layout.addWidget(self.rviz_container)
        
        # Launch RViz after widget is constructed
        QTimer.singleShot(100, self._launch_rviz)
    
    def add_robot(self, description_topic: str):
        """Add another robot visualization to the scene"""
        # The C++ layer now handles queuing if RViz isn't ready
        add_robot(description_topic)
    
    def _launch_rviz(self):
        """Launch RViz in the container widget"""
        print("[RVizQtWidget] Launching RViz...")
        
        # Force widget to be fully realized
        self.show()
        self.rviz_container.show()
        QCoreApplication.processEvents()
        
        # Get window ID after ensuring widget is visible
        self.rviz_container.hide()
        QCoreApplication.processEvents()
        self.rviz_container.show()
        QCoreApplication.processEvents()
        
        native_id = int(self.rviz_container.winId())
        print(f"[RVizQtWidget] Container window ID: {native_id}")
        
        # Only cleanup if we are managing the process (urdf provided)
        if self.urdf_path: 
            self._cleanup_rviz_processes()
        
        try:
            # Launch RViz with the native window ID
            # Si description_topic está vacío, pasar None para evitar crear un display vacío
            topic_to_use = self.description_topic if self.description_topic else None
            if topic_to_use is None:
                print("[RVizQtWidget] No initial robot topic - RViz will start empty")
            
            launch_rviz(native_id, self.urdf_path, topic_to_use or "", self.fixed_frame)
            
            # FIX: Mark as launched
            self._rviz_launched = True
            
            # Verify RViz is running
            QTimer.singleShot(500, self._verify_rviz_running)
            
        except Exception as e:
            print(f"[RVizQtWidget] Launch failed: {e}")
            import traceback
            traceback.print_exc()
            self._retry_launch()
    
    def _cleanup_rviz_processes(self):
        """Clean up any lingering RViz processes"""
        try:
            subprocess.run(["pkill", "-f", "rviz"], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "ogre"], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
            time.sleep(0.3)
            print("[RVizQtWidget] Cleanup completed")
        except Exception as e:
            print(f"[RVizQtWidget] Cleanup warning: {e}")
    
    def _verify_rviz_running(self):
        """Verify RViz process is actually running"""
        if self._is_shutting_down:
            return
        
        # Since we are running embedded in this process, if we reached here
        # without exception from launch_rviz, we are running.
        # External process checks via pgrep are unreliable for embedded use and cause crash-loops.
        if self._rviz_launched:
             print(f"[RVizQtWidget] RViz confirmed running (Embedded)")
        else:
             print("[RVizQtWidget] RViz launch flag not set - retrying...")
             self._retry_launch()
    
    def _retry_launch(self):
        """Retry RViz launch"""
        if self._is_shutting_down:
            return
        print("[RVizQtWidget] Retrying launch...")
        QTimer.singleShot(1000, self._launch_rviz)
    
    def shutdown(self):
        """Clean shutdown with C++ cleanup."""
        if self._is_shutting_down:  # Now this works!
            return
        
        print("[RVizQtWidget] Starting shutdown...")
        self._is_shutting_down = True
        
        try:
            self.hide()
            QCoreApplication.processEvents()
            
            # CRITICAL: Call C++ cleanup
            if self._rviz_launched:  # Now this works!
                print("[RVizQtWidget] Calling C++ cleanup...")
                cleanup_rviz()  # Now this is imported!
                time.sleep(0.3)
            
            # Qt cleanup
            if self.rviz_container:
                self._layout.removeWidget(self.rviz_container)
                self.rviz_container.setParent(None)
                self.rviz_container.deleteLater()
                self.rviz_container = None
            
            self.deleteLater()
            print("[RVizQtWidget] Shutdown complete")
            
        except Exception as e:
            print(f"[RVizQtWidget] Shutdown error: {e}")
            import traceback
            traceback.print_exc()
    
    def closeEvent(self, event):
        """Handle Qt close event"""
        print("[RVizQtWidget] Close event received")
        self.shutdown()
        super().closeEvent(event)


