#include "ur5_interfaz_panel/mainWindow.hpp"
#include <QLabel>
#include <QFile>
#include <QDebug>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QGridLayout>



MainWindow::MainWindow(QApplication *app, QWidget *parent)
    : QMainWindow(parent), app_(app) {
    
    // Crear widget central, contenedor de los elementos
    //DEclaraciones que luego se moveran a hpp
    QWidget *menuWidget_;
    QWidget *rvizWidget_;
    QHBoxLayout *mainLayout_;

    

    centralWidget_ = new QWidget();
    menuWidget_ = new QWidget();
    menuWidget_->setObjectName("menuWidget");
    menuWidget_->setFixedWidth(300);

    // Inicializar rvizWidget antes de usarlo
    rvizWidget_ = new QWidget();
    rvizWidget_->setObjectName("rvizWidget"); 

    mainLayout_ = new QHBoxLayout(centralWidget_);
    mainLayout_->addWidget(menuWidget_);
    mainLayout_->addWidget(rvizWidget_);

    Layout_menu_ = new QVBoxLayout(menuWidget_); //Layout_menu_ estará dentro de menuWidget_
    
    // Crear título
    QLabel *titleLabel = new QLabel("Interfaz UR5");
    titleLabel->setStyleSheet("font-size: 20px; font-weight: bold;");
    titleLabel->setAlignment(Qt::AlignCenter);
    Layout_menu_->addWidget(titleLabel);
    
    // Agregar espacio
    Layout_menu_->addStretch();
    
    // Crear botón de salir
    exitButton_ = new QPushButton("Salir");
    exitButton_->setObjectName("exitButton");
    exitButton_->setFixedSize(150, 40);
    Layout_menu_->addWidget(exitButton_, 0, Qt::AlignCenter);
    
    // Conectar señal del botón
    connect(exitButton_, &QPushButton::clicked, this, &MainWindow::onExitClicked);
    
    // Crear widget contenedor
    QWidget *containerWidget = new QWidget();
    containerWidget->setObjectName("buttonContainer");

    // Crear layout y asignarlo al widget
    QHBoxLayout *buttonLayout = new QHBoxLayout(containerWidget);
    buttonLayout->addWidget(new QPushButton("Botón 1"));
    buttonLayout->addWidget(new QPushButton("Botón 2"));

    // Agregar el widget (no el layout) al layout principal
    Layout_menu_->addWidget(containerWidget);

    // Crear grid para controles
    QGridLayout *controlGrid = new QGridLayout();
    controlGrid->addWidget(new QLabel("X:"), 0, 0);
    controlGrid->addWidget(new QLineEdit(), 0, 1);
    Layout_menu_->addLayout(controlGrid);
    
    // Configurar widget central
    setCentralWidget(centralWidget_);
    
    // Configurar ventana
    setWindowTitle("Panel de Control UR5");
    resize(400, 300);
    
    // Cargar estilos QSS
    loadStyleSheet();

}

void MainWindow::loadStyleSheet() {
    QFile styleFile(":/style/style.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        QString styleSheet = QLatin1String(styleFile.readAll());
        app_->setStyleSheet(styleSheet);
        qDebug() << "Estilos QSS cargados correctamente";
        styleFile.close();
    } else {
        qDebug() << "Error: No se pudo cargar el archivo style.qss";
    }
}

void MainWindow::onExitClicked() {
    app_->quit();
}

MainWindow::~MainWindow() {
}