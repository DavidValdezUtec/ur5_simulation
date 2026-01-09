#include <QApplication>
#include "ur5_interfaz_panel/mainWindow.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    
    MainWindow window(&app);
    window.show();
    
    return app.exec();
}
