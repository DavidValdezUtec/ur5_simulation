#include "ros2_qt_rviz/simple_viz.hpp"

#include <QVBoxLayout>
#include <QTimer>
#include <QDebug>
#include <QShowEvent> // Necesario para el evento
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

SimpleViz::SimpleViz(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QWidget(parent), node_(node)
{
    // FIX 1: Atributos para compatibilidad con Ogre/OpenGL
    // Esto evita que Qt intente optimizar el renderizado y cause conflictos
    this->setAttribute(Qt::WA_NativeWindow);
    this->setAttribute(Qt::WA_PaintOnScreen);
    this->setAttribute(Qt::WA_NoSystemBackground);

    auto layout = new QVBoxLayout(this);
    
    // Crear el Panel
    render_panel_ = new rviz_common::RenderPanel(this);
    layout->addWidget(render_panel_);

    // Crear la abstracción del nodo
    node_abstraction_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    
    // Iniciar el loop de ROS interno
    auto rviz_node = node_abstraction_->get_raw_node();
    auto spin_timer = new QTimer(this);
    connect(spin_timer, &QTimer::timeout, [this, rviz_node]() {
        if(rclcpp::ok() && rviz_node) {
            rclcpp::spin_some(rviz_node);
        }
    });
    spin_timer->start(10); 
}

void SimpleViz::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    // Solo inicializamos la primera vez que se muestra la ventana
    if (initialized_) return;

    qDebug() << "--- INICIO DE INICIALIZACIÓN RVIZ ---";

    auto rviz_node = node_abstraction_->get_raw_node();
    if (!rviz_node) {
        qCritical() << "Error fatal: Nodo RViz nulo.";
        return;
    }

    try {
        qDebug() << "[1] Creando VisualizationManager...";
        rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_abs_weak = node_abstraction_;

        manager_ = new rviz_common::VisualizationManager(
            render_panel_, 
            node_abs_weak, // Pasamos el weak_ptr explícito
            nullptr, 
            rviz_node->get_clock()
        );

        qDebug() << "[2] Inicializando RenderPanel (Ogre Window)...";
        render_panel_->initialize(manager_);

        qDebug() << "[3] Inicializando Manager (Plugins)...";
        manager_->initialize();

        qDebug() << "[4] Iniciando actualizaciones...";
        manager_->startUpdate();

        qDebug() << "[5] Configurando Frame Fijo...";
        manager_->setFixedFrame("map"); 

        qDebug() << "[6] Cargando Grid...";
        auto grid = manager_->createDisplay("rviz_default_plugins/Grid", "MyGrid", true);
        if(grid) {
            qDebug() << "Grid cargado OK.";
        }

        initialized_ = true;
        qDebug() << "--- INICIALIZACIÓN COMPLETADA ---";

    } catch (const std::exception& e) {
        qCritical() << "EXCEPCIÓN EN INICIALIZACIÓN: " << e.what();
    } catch (...) {
        qCritical() << "EXCEPCIÓN DESCONOCIDA EN INICIALIZACIÓN";
    }
}

SimpleViz::~SimpleViz()
{
    if (manager_) {
        manager_->stopUpdate();
        delete manager_; 
    }
}