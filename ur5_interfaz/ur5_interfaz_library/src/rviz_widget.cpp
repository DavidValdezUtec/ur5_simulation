/**
 * @file rviz_widget.cpp
 * @brief Implementation of RViz widget embedding functionality for PyQt5 integration
 * 
 * This file provides the core C++ implementation for embedding ROS2 RViz visualization
 * into Qt widgets. It handles the complete lifecycle of RViz including initialization,
 * rendering, and proper cleanup to prevent memory leaks and segmentation faults.
 * 
 * Key Features:
 * - Native Qt widget integration using window ID embedding
 * - Automatic ROS2 node and publisher management
 * - URDF loading and robot visualization
 * - Thread-safe cleanup with mutex protection
 * - Emergency cleanup handlers for unexpected termination
 * 
 * @author Haris Ceribasic
 * @date 2025
 */

#include "ur5_interfaz_panel/rviz_widget.hpp"

#include <QApplication>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>
#include <QPointer>

#include <fstream>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <ctime>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <unistd.h>

namespace rviz_embed {

// Forward declaration
class DummyWindowManager;

namespace {
const std::string NODE_NAME = "embedded_rviz_node";

// CRITICAL FIX: Use raw pointer instead of shared_ptr for global node
// This prevents static destruction order issues
static rviz_common::VisualizationManager* manager = nullptr;
static rviz_common::RenderPanel* render_panel = nullptr;
static QWidget* wrapper_widget = nullptr;
static rviz_common::Display* robot_disp = nullptr;
static rviz_common::Display* grid_display = nullptr;
static DummyWindowManager* dummy_window_mgr = nullptr;
static rclcpp::Node* global_node = nullptr;  // CRITICAL: Changed from shared_ptr to raw pointer
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_publisher = nullptr;
static std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> global_abstraction = nullptr;

static bool is_initialized = false;
static bool is_shutting_down = false;
static int instance_counter = 0;
static std::mutex cleanup_mutex;  // Thread safety for cleanup
static std::vector<std::string> pending_robots;
}

/**
 * @class DummyWindowManager
 * @brief Minimal implementation of WindowManagerInterface for embedded RViz
 * 
 * For some reason, RViz requires a WindowManagerInterface to manage panels and status messages.
 * This dummy implementation satisfies the interface requirements while providing
 * no actual panel management functionality, suitable for simple embedded use cases.
 */
class DummyWindowManager : public rviz_common::WindowManagerInterface {
public:
    explicit DummyWindowManager(QWidget* parent) : parent_(parent) {}
    ~DummyWindowManager() override = default;

    QWidget* getParentWindow() override {
        return parent_;
    }

    void setStatus(const QString&) override {}

    rviz_common::PanelDockWidget* addPane(
        const QString&,
        QWidget*,
        Qt::DockWidgetArea,
        bool) override
    {
        return nullptr;
    }

private:
    QWidget* parent_;
};

/**
 * @brief Performs comprehensive cleanup of all RViz resources
 * 
 * This function executes a carefully ordered shutdown sequence to prevent
 * segmentation faults and memory leaks. The order is critical:
 * 
 * 1. Stop RViz updates
 * 2. Disable displays
 * 3. Delete visualization manager (destroys ROS publishers/subscribers)
 * 4. Delete Qt widgets
 * 5. Clear ROS publishers
 * 6. Delete ROS node (MUST BE LAST)
 * 
 * 
 * @note This function handles all exceptions to ensure cleanup completes.
 */
void cleanup_rviz()
{
    std::lock_guard<std::mutex> lock(cleanup_mutex);
    
    if (is_shutting_down) {
        printf("[Cleanup] Already shutting down, skipping duplicate cleanup\n");
        return;
    }
    
    if (!is_initialized) {
        printf("[Cleanup] Not initialized, nothing to cleanup\n");
        return;
    }
    
    // Clear pending robots
    pending_robots.clear();
    
    printf("[Cleanup] ========================================\n");
    printf("[Cleanup] Starting comprehensive RViz cleanup...\n");
    printf("[Cleanup] ========================================\n");
    
    is_shutting_down = true;

    try {
        /**
        * @note: The order of deletion matters !
        */

        // STEP 1: Stop manager updates (while ROS is still valid)
        if (manager) {
            printf("[Cleanup] [1/10] Stopping manager updates...\n");
            try {
                manager->stopUpdate();
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Warning stopping manager updates: %s\n", e.what());
            }
        }

        // STEP 2: Disable and clear displays (!!! must happen before manager deletion)
        if (robot_disp) {
            printf("[Cleanup] [2/10] Disabling robot display...\n");
            try {
                robot_disp->setEnabled(false);
                // Don't delete - manager owns it
                robot_disp = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Warning disabling robot display: %s\n", e.what());
            }
        }
        
        if (grid_display) {
            printf("[Cleanup] [3/10] Disabling grid display...\n");
            try {
                grid_display->setEnabled(false);
                // Don't delete - manager owns it
                grid_display = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Warning disabling grid display: %s\n", e.what());
            }
        }

        // STEP 3: Delete visualization manager (this destroys ROS publishers/subscribers)
        // CRITICAL: Must happen BEFORE node deletion
        if (manager) {
            printf("[Cleanup] [4/10] Deleting visualization manager...\n");
            try {
                delete manager;
                manager = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error deleting manager: %s\n", e.what());
                manager = nullptr;  // Mark as null even if delete failed
            }
        }

        // STEP 4: Delete render panel
        if (render_panel) {
            printf("[Cleanup] [5/10] Deleting render panel...\n");
            try {
                render_panel->setParent(nullptr);
                delete render_panel;
                render_panel = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error deleting render panel: %s\n", e.what());
                render_panel = nullptr;
            }
        }

        // STEP 5: Delete dummy window manager
        if (dummy_window_mgr) {
            printf("[Cleanup] [6/10] Deleting dummy window manager...\n");
            try {
                delete dummy_window_mgr;
                dummy_window_mgr = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error deleting window manager: %s\n", e.what());
                dummy_window_mgr = nullptr;
            }
        }

        // STEP 6: Delete wrapper widget
        if (wrapper_widget) {
            printf("[Cleanup] [7/10] Deleting wrapper widget...\n");
            try {
                wrapper_widget->hide();
                wrapper_widget->setParent(nullptr);
                delete wrapper_widget;
                wrapper_widget = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error deleting wrapper widget: %s\n", e.what());
                wrapper_widget = nullptr;
            }
        }

        // STEP 7: Clear publisher (before node deletion)
        if (global_publisher) {
            printf("[Cleanup] [8/10] Clearing global publisher...\n");
            try {
                global_publisher.reset();
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error clearing publisher: %s\n", e.what());
                global_publisher = nullptr;
            }
        }

        // STEP 8: Clear abstraction (before node deletion)
        if (global_abstraction) {
            printf("[Cleanup] [9/10] Clearing ROS abstraction...\n");
            try {
                global_abstraction.reset();
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error clearing abstraction: %s\n", e.what());
                global_abstraction = nullptr;
            }
        }

        // STEP 9: CRITICAL - Delete node LAST
        // All ROS publishers/subscribers/services MUST be destroyed first
        if (global_node) {
            printf("[Cleanup] [10/10] Deleting ROS node (CRITICAL STEP)...\n");
            try {
                delete global_node;
                global_node = nullptr;
            } catch (const std::exception& e) {
                fprintf(stderr, "[Cleanup] Error deleting node: %s\n", e.what());
                global_node = nullptr;
            }
        }

        // STEP 10: Force Qt event processing
        if (QApplication::instance()) {
            printf("[Cleanup] Processing Qt events...\n");
            QApplication::processEvents();
        }

        // Give DDS time to clean up internal structures
        printf("[Cleanup] Waiting for DDS cleanup...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Kill external processes
        printf("[Cleanup] Killing external processes...\n");
        std::system("pkill -9 -f joint_state_publisher_gui 2>/dev/null");
        std::system("pkill -9 -f joint_state_publisher 2>/dev/null");
        std::system("pkill -9 -f robot_state_publisher 2>/dev/null");

        // Reset flags
        is_initialized = false;
        
        printf("[Cleanup] ========================================\n");
        printf("[Cleanup] Cleanup completed successfully\n");
        printf("[Cleanup] ========================================\n");
        
    } catch (const std::exception& e) {
        fprintf(stderr, "[Cleanup] CRITICAL ERROR during cleanup: %s\n", e.what());
        
        // Force cleanup of all pointers even on error
        manager = nullptr;
        render_panel = nullptr;
        dummy_window_mgr = nullptr;
        wrapper_widget = nullptr;
        robot_disp = nullptr;
        grid_display = nullptr;
        global_publisher = nullptr;
        global_abstraction = nullptr;
        global_node = nullptr;
        is_initialized = false;
    }
    
    is_shutting_down = false;
}

// Emergency cleanup function - called at program exit
void emergency_cleanup() {
    printf("[Emergency] ========================================\n");
    printf("[Emergency] Emergency cleanup triggered\n");
    printf("[Emergency] ========================================\n");
    
    if (is_initialized && !is_shutting_down) {
        fprintf(stderr, "[Emergency] WARNING: Normal cleanup was not called!\n");
        fprintf(stderr, "[Emergency] Performing emergency shutdown...\n");
        cleanup_rviz();
    } else {
        printf("[Emergency] Normal cleanup already completed\n");
    }
}

/**
 * @brief Launches RViz embedded in a Qt widget
 * 
 * This is the main entry point for creating an embedded RViz instance.
 * The function performs the following operations:
 * 
 * 1. Validates parent widget exists
 * 2. Initializes ROS2 context if needed
 * 3. Loads URDF file and publishes to /robot_description topic
 * 4. Creates RViz visualization manager and render panel
 * 5. Configures displays (Robot Model, Grid)
 * 6. Sets up camera view parameters
 * 
 * @param parent_win_id Native window ID of the parent Qt widget
 * @param urdf_path Absolute path to the URDF robot description file
 * 
 * @note This function uses QTimer::singleShot to defer initialization,
 *       preventing GUI blocking during startup
 * 
 * @warning The parent widget must be visible before calling this function
 * 
 * @throws std::exception if URDF file cannot be opened or ROS2 initialization fails
 */
void launch_rviz(
    int64_t parent_win_id, 
    const std::string& urdf_path,
    const std::string& description_topic,
    const std::string& fixed_frame)
{
    printf("[Launch] ========================================\n");
    printf("[Launch] Starting RViz launch (v2)\n");
    printf("[Launch] URDF: %s\n", urdf_path.empty() ? "[PASSIVE MODE - External Publishers]" : urdf_path.c_str());
    printf("[Launch] Topic: %s\n", description_topic.c_str());
    printf("[Launch] Frame: %s\n", fixed_frame.c_str());
    printf("[Launch] Parent: %ld\n", parent_win_id);
    printf("[Launch] ========================================\n");
    
    // Clean up any existing instance first
    if (is_initialized) {
        printf("[Launch] Cleaning up existing instance...\n");
        cleanup_rviz();
        
        // Give time for cleanup to complete
        if (QApplication::instance()) {
            QApplication::processEvents();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Ensure QApplication exists
    if (QApplication::instance() == nullptr) {
        printf("[Launch] Creating QApplication...\n");
        int argc = 0;
        char** argv = nullptr;
        new QApplication(argc, argv);
    }

    // Find parent widget
    QWidget* parent_widget = QWidget::find(static_cast<WId>(parent_win_id));
    if (!parent_widget) {
        fprintf(stderr, "[Launch] ERROR: Could not find parent widget for WId: %ld\n", parent_win_id);
        return;
    }
    printf("[Launch] Found parent widget: %p\n", (void*)parent_widget);

    // Guard the parent widget pointer to prevent use-after-free if Python destroys it
    QPointer<QWidget> guarded_parent = parent_widget;

    // Defer ROS/RViz initialization to avoid blocking
    QTimer::singleShot(200, [guarded_parent, urdf_path, description_topic, fixed_frame]() {
        printf("[Launch] Timer callback executing...\n");
        
        if (guarded_parent.isNull()) {
            fprintf(stderr, "[Launch] ABORTED: Parent widget no longer exists!\n");
            return;
        }
        
        QWidget* parent_widget = guarded_parent.data();
        bool passive_mode = urdf_path.empty();

        try {
            // Initialize ROS 2 if needed
            if (!rclcpp::ok()) {
                printf("[Launch] Initializing ROS 2 context...\n");
                int argc = 0;
                char** argv = nullptr;
                rclcpp::init(argc, argv);
            }

            // Only perform active setup if we have a URDF path
            if (!passive_mode) {
                // Load URDF
                printf("[Launch] Loading URDF from: %s\n", urdf_path.c_str());
                std::ifstream urdf_file(urdf_path);
                if (!urdf_file.is_open()) {
                    fprintf(stderr, "[Launch] ERROR: Failed to open URDF file: %s\n", urdf_path.c_str());
                    // Don't abort, try to continue in passive-ish mode? No, abort for now.
                    return;
                }
                
                std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)), 
                                    std::istreambuf_iterator<char>());
                printf("[Launch] URDF loaded successfully (%zu bytes)\n", urdf_string.size());

                // Kill any existing publishers (Active mode only)
                printf("[Launch] Killing existing publishers...\n");
                std::system("pkill -9 -f robot_state_publisher 2>/dev/null");
                std::system("pkill -9 -f joint_state_publisher 2>/dev/null");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                // Start fresh publishers
                printf("[Launch] Starting robot_state_publisher...\n");
                std::string robot_state_cmd = "ros2 run robot_state_publisher robot_state_publisher " + 
                                            urdf_path + " > /dev/null 2>&1 &";
                std::system(robot_state_cmd.c_str());

                printf("[Launch] Starting joint_state_publisher...\n");
                std::system("ros2 run joint_state_publisher joint_state_publisher > /dev/null 2>&1 &");
                
                // CRITICAL: Create node with NEW, not make_shared
                instance_counter++;
                std::stringstream ss;
                ss << NODE_NAME << "_" << getpid() << "_" << instance_counter;
                std::string unique_node_name = ss.str();
                
                std::string pub_node_name = unique_node_name + "_pub";
                
                printf("[Launch] Creating ROS node: %s\n", pub_node_name.c_str());
                global_node = new rclcpp::Node(pub_node_name);

                // Create publisher
                printf("[Launch] Creating %s publisher...\n", description_topic.c_str());
                global_publisher = global_node->create_publisher<std_msgs::msg::String>(
                    description_topic, rclcpp::QoS(10).transient_local());

                // Publish URDF
                printf("[Launch] Publishing URDF...\n");
                std_msgs::msg::String msg;
                msg.data = urdf_string;
                global_publisher->publish(msg);
            } else {
                printf("[Launch] Passive mode: Skipping publisher creation/cleanup\n");
                // In passive mode, we still need a node for RViz abstraction
                instance_counter++;
                std::stringstream ss;
                ss << NODE_NAME << "_" << getpid() << "_" << instance_counter;
                std::string unique_node_name = ss.str();
                
                global_node = new rclcpp::Node(unique_node_name); // Minimal node
            }

             // Create RViz abstraction
            // Use a unique name for the RViz visualizer node
            std::stringstream ss_rviz;
            ss_rviz << "rviz_node_" << instance_counter;
            
            // NOTE: RosNodeAbstraction needs a unique name to verify it doesn't conflict
            std::string rviz_node_name = std::string(global_node->get_name()) + "_wrapper";
            
            printf("[Launch] Creating ROS abstraction with name: %s\n", rviz_node_name.c_str());
            global_abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
                rviz_node_name); 
            std::weak_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> abstraction_wp = 
                global_abstraction;

            // Set up render panel
            printf("[Launch] Creating render panel...\n");
            render_panel = new rviz_common::RenderPanel();
            render_panel->setFocusPolicy(Qt::StrongFocus);
            render_panel->setMouseTracking(true);
            render_panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

            // Create wrapper widget
            printf("[Launch] Creating wrapper widget...\n");
            wrapper_widget = new QWidget(parent_widget);
            wrapper_widget->setAttribute(Qt::WA_NativeWindow);
            
            auto* layout = new QVBoxLayout(wrapper_widget);
            layout->setContentsMargins(0, 0, 0, 0);
            layout->addWidget(render_panel);

            // FIX: Add layout to parent_widget so wrapper_widget resizes automatically
            if (!parent_widget->layout()) {
                 auto* parent_layout = new QVBoxLayout(parent_widget);
                 parent_layout->setContentsMargins(0, 0, 0, 0);
                 parent_layout->addWidget(wrapper_widget);
            } else {
                 parent_widget->layout()->addWidget(wrapper_widget);
            }
            
            wrapper_widget->show();
            render_panel->show();
            render_panel->update();
            QApplication::processEvents();

            // Create dummy window manager
            printf("[Launch] Creating window manager...\n");
            dummy_window_mgr = new DummyWindowManager(wrapper_widget);

            // Create and initialize RViz manager
            printf("[Launch] Creating visualization manager...\n");
            manager = new rviz_common::VisualizationManager(
                render_panel, abstraction_wp, dummy_window_mgr, global_node->get_clock());
            
            printf("[Launch] Initializing visualization manager...\n");
            manager->initialize();
            render_panel->initialize(manager);

            // Setup tools and views
            printf("[Launch] Setting up tools...\n");
            auto* tool_mgr = manager->getToolManager();
            tool_mgr->initialize();
            tool_mgr->setCurrentTool(tool_mgr->addTool("rviz_default_plugins/Interact"));

            printf("[Launch] Setting up views...\n");
            auto view_mgr = manager->getViewManager();
            view_mgr->initialize();
            auto orbit = view_mgr->create("rviz_default_plugins/Orbit");
            view_mgr->setCurrentFrom(orbit);
            orbit->subProp("Target Frame")->setValue(fixed_frame.c_str());

            // Set specific camera parameters for optimal robot viewing
            auto view = view_mgr->getCurrent();
            view->subProp("Yaw")->setValue(0.5);
            view->subProp("Pitch")->setValue(0.240398);
            view->subProp("Focal Point")->setValue("0; 0; 0");
            view->subProp("Distance")->setValue(5.63583);

            manager->setFixedFrame(fixed_frame.c_str());
            
            printf("[Launch] Starting manager updates...\n");
            manager->startUpdate();

            // Process events and give Ogre time to initialize
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Create displays
            // CRITICAL: Only create robot display if description_topic is not empty
            // Creating a RobotModel display with an empty topic causes segmentation faults
            if (!description_topic.empty()) {
                printf("[Launch] Creating robot display for topic: %s\n", description_topic.c_str());
                try {
                    robot_disp = manager->createDisplay("rviz_default_plugins/RobotModel", "RobotModel", true);
                    if (robot_disp) {
                        robot_disp->subProp("Description Source")->setValue("Topic");
                        robot_disp->subProp("Description Topic")->setValue(description_topic.c_str());
                        printf("[Launch] Robot display created successfully\n");
                    } else {
                        fprintf(stderr, "[Launch] Warning: Robot display creation returned null\n");
                    }
                } catch (const std::exception& e) {
                    fprintf(stderr, "[Launch] Error creating robot display: %s\n", e.what());
                    robot_disp = nullptr;
                }
            } else {
                printf("[Launch] No robot topic specified - RViz starting empty (no robot display)\n");
                robot_disp = nullptr;
            }

            // Process events before creating grid
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            printf("[Launch] Creating grid display...\n");
            try {
                grid_display = manager->createDisplay("rviz_default_plugins/Grid", "Grid", true);
                if (grid_display) {
                    printf("[Launch] Grid display created successfully\n");
                } else {
                    fprintf(stderr, "[Launch] Warning: Grid display creation returned null\n");
                }
            } catch (const std::exception& e) {
                fprintf(stderr, "[Launch] Error creating grid display: %s\n", e.what());
                grid_display = nullptr;
            }

            // Finalize widget setup
            wrapper_widget->setMinimumSize(parent_widget->size());
            wrapper_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            wrapper_widget->show();

            // Mark as initialized
            is_initialized = true;

            // Process pending robots via Timer to allow stack unwinding
            if (!pending_robots.empty()) {
                // Increased delay to 1s to allow Ogre/Qt to stabilize fully.
                QTimer::singleShot(1000, [](){
                    try {
                        if (!is_initialized || !manager) return;

                        // Double check manager is usable
                        if (!manager->getRootDisplayGroup()) {
                             fprintf(stderr, "[RViz] Manager display group not ready yet.\n");
                             return;
                        }

                        if (!pending_robots.empty()) {
                            printf("[Launch] Processing %zu pending robots (delayed 1s)...\n", pending_robots.size());
                            
                            // Copy waiting list to avoid iterating while modyfing globally (unlikely but safe)
                            std::vector<std::string> robots_to_add = pending_robots;
                            pending_robots.clear();

                            for (const auto& topic : robots_to_add) {
                                add_robot(topic);
                            }
                        }
                    } catch (const std::exception& e) {
                        fprintf(stderr, "[RViz] Error in delayed robot processor: %s\n", e.what());
                    } catch (...) {
                         fprintf(stderr, "[RViz] Unknown error in delayed robot processor\n");
                    }
                });
            }

            printf("[Launch] ========================================\n");
            printf("[Launch] RViz launched successfully!\n");
            printf("[Launch] ========================================\n");
            
        } catch (const std::exception& e) {
            fprintf(stderr, "[Launch] CRITICAL ERROR during launch: %s\n", e.what());
            
            // Clean up on failure
            cleanup_rviz();
        }
    });
}

void add_robot(const std::string& description_topic)
{
    if (!manager || !is_initialized) {
        printf("[RViz] Manager not ready. Queuing robot: %s\n", description_topic.c_str());
        pending_robots.push_back(description_topic);
        return;
    }

    printf("[RViz] Adding additional RobotModel for topic: %s\n", description_topic.c_str());
    
    // We execute this on the Qt thread ideally, but since we are likely called from Python main thread 
    // which holds the GIL and Qt loop, it might be fine. 
    // To be safe, use QMetaObject::invokeMethod or just assume caller is in UI thread.
    
    try {
        auto* display = manager->createDisplay("rviz_default_plugins/RobotModel", ("Robot_" + description_topic).c_str(), true);
        if (display) {
            display->subProp("Description Source")->setValue("Topic");
            display->subProp("Description Topic")->setValue(description_topic.c_str());
            printf("[RViz] Additional robot added successfully\n");
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[RViz] Error adding robot: %s\n", e.what());
    }
}

} // namespace rviz_embed