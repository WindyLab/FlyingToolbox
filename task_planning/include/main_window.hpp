/**
 * @file main_window.hpp
 * @brief Qt-based GUI for UAV Monitoring and Motion Planning
 * 
 * This application provides:
 * - Real-time UAV position monitoring
 * - Motion trajectory planning
 * - Collision detection
 * - Task configuration and generation
 * 
 * @date November 2024
 **/
#ifndef task_planning_MAIN_WINDOW_H
#define task_planning_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

// Qt Core and Main Widgets
#include <QtWidgets/QMainWindow>
#include <QMainWindow>
#include <QLabel>
#include <QTimer>

// Qt Graphics Related
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPixmap>

// Qt Layout and Widgets
#include <QVBoxLayout>
#include <QTableWidget>
#include <QMessageBox>

// Qt Events and Data Types
#include <QMouseEvent>
#include <QVector3D>
#include <QMap>

// Project specific headers
#include "ui_main_window.h"
#include "qnode.hpp"

namespace task_planning {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    // Constructor and destructor
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    
    // Point management
    void addPoint(QPoint point);
protected:
    // Event handlers
    void mousePressEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *object, QEvent *event) override;

public Q_SLOTS:
    // UI event handlers
    void on_button_connect_clicked();
    void showNoMasterMessage();
    void UAV1_Pose_Display(const geometry_msgs::TransformStamped& pose);
    void UAV2_Pose_Display(const geometry_msgs::TransformStamped& pose);
    void removeRow();
    void generateYamlFile();
    void Collision_Detection();
    void drawCirclesFromTable();

private:
    // UI components
    Ui::MainWindowDesign ui;
    QNode qnode_;
    QGraphicsView *view_;
    QGraphicsScene *scene_;
    QPointF last_position_;
    QLabel *image_label_;
    QTableWidget *table_widget_;
    QPixmap pixmap_;
    int max_track_id_;
    double collision_threshold_;  // Collision detection threshold in meters

    // Helper functions
    void displayImage_1();
    void displayImage_2();
    void addPositionToScene(const QPointF &position, const QColor &color);
    void setupTableWidget();
    bool isPositionInBounds(const QPointF &position) const;
    void logPositionError(const QPointF &position) const;
    void setupConnections();
    void setupInitialUI();
};

}  // namespace task_planning

#endif // task_planning_MAIN_WINDOW_H
