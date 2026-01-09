#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QApplication>

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QApplication *app, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onExitClicked();

private:
    void loadStyleSheet();
    
    QApplication *app_;
    QWidget *centralWidget_;
    QVBoxLayout *Layout_menu_;
    QVBoxLayout *Layout_rviz_;
    QPushButton *exitButton_;
};

#endif // MAINWINDOW_HPP