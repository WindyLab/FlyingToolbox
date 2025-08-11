#include "../include/main_window.hpp"

namespace task_planning {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode_(argc,argv)
    , max_track_id_(0)
    , collision_threshold_(1.0)  // Default collision threshold
{
    // Load background image once during initialization
    if (!pixmap_.load(":/images/lab.png")) {
        QMessageBox::warning(this, "Warning", "Failed to load image.");
    }
    // Initialize UI components and setup connections
    ui.setupUi(this);
    setupConnections();
    setupInitialUI();
    displayImage_1();  // Display main monitoring view
    displayImage_2();  // Display trajectory planning view
    setupTableWidget();
}

void MainWindow::setupConnections() {
    // Register ROS message type for pose updates
    qRegisterMetaType<geometry_msgs::TransformStamped>("geometry_msgs::TransformStamped");

    // Connect UI control signals
    connect(ui.button_quit, &QPushButton::clicked, qApp, &QApplication::quit);
    connect(ui.button_quit_2, &QPushButton::clicked, qApp, &QApplication::quit);
    connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    connect(ui.delete_row, &QPushButton::clicked, this, &MainWindow::removeRow);
    connect(ui.generate_task, &QPushButton::clicked, this, &MainWindow::generateYamlFile);
    connect(ui.collision_detection, &QPushButton::clicked, this, &MainWindow::Collision_Detection);

    // Connect ROS communication signals
    connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(&qnode_, SIGNAL(UAV1_updatePose(const geometry_msgs::TransformStamped&)),
            this, SLOT(UAV1_Pose_Display(const geometry_msgs::TransformStamped&)));
    connect(&qnode_, SIGNAL(UAV2_updatePose(const geometry_msgs::TransformStamped&)),
            this, SLOT(UAV2_Pose_Display(const geometry_msgs::TransformStamped&)));
}

void MainWindow::setupInitialUI() {
    // Set window icon and initial tab
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.UAV_Monitor->setCurrentIndex(0);

    // Initialize position display fields with default values
    const QStringList lineEdits = {"lineEdit_1", "lineEdit_2", "lineEdit_3", 
                                  "lineEdit_4", "lineEdit_5", "lineEdit_6"};
    for (const auto& editName : lineEdits) {
        QLineEdit* edit = findChild<QLineEdit*>(editName);
        if (edit) {
            edit->setText("0.0");
            edit->setAlignment(Qt::AlignRight);
        }
    }
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::on_button_connect_clicked() {
    if (!qnode_.init()) {
        showNoMasterMessage();
    } else {
        ui.button_connect->setEnabled(false);
    }
}

void MainWindow::displayImage_1() {
    // Initialize graphics view for real-time monitoring
    view_ = new QGraphicsView(ui.widget_show);
    scene_ = new QGraphicsScene(this);

    // Setup graphics scene with background image
    QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem(pixmap_);
    scene_->addItem(pixmapItem);
    scene_->setSceneRect(pixmap_.rect());
    view_->setScene(scene_);

    // Add view to layout
    QVBoxLayout *layout = new QVBoxLayout(ui.widget_show);
    layout->addWidget(view_);
    ui.widget_show->setLayout(layout);
}

void MainWindow::displayImage_2() {
    image_label_ = new QLabel(ui.widget);
    image_label_->setPixmap(pixmap_);
    QSize pixmapSize = pixmap_.size();
    image_label_->setGeometry(ui.widget->geometry().x(), ui.widget->geometry().y(), 
                             pixmapSize.width(), pixmapSize.height());

    QVBoxLayout *layout = new QVBoxLayout(ui.widget);
    layout->addWidget(image_label_);
    ui.widget->setLayout(layout);

    image_label_->installEventFilter(this);
    drawCirclesFromTable();
}

void MainWindow::UAV1_Pose_Display(const geometry_msgs::TransformStamped &UAV1_pose) {
    // Update position display in UI
    ui.lineEdit_1->setText(QString::number(UAV1_pose.transform.translation.x));
    ui.lineEdit_2->setText(QString::number(UAV1_pose.transform.translation.y));
    ui.lineEdit_3->setText(QString::number(UAV1_pose.transform.translation.z));

    // Convert ROS coordinates to display coordinates
    // Note: Coordinate transformation includes scaling and offset
    float temp_x = UAV1_pose.transform.translation.y * 100 + 500.0;  // Scale by 100 and offset by 500
    float temp_y = UAV1_pose.transform.translation.x * 100 + 300.0;  // Scale by 100 and offset by 300
    QPointF new_position(temp_x, temp_y);

    // Validate and display position
    if (isPositionInBounds(new_position)) {
        addPositionToScene(new_position, Qt::red);
    } else {
        logPositionError(new_position);
    }
}


void MainWindow::UAV2_Pose_Display(const geometry_msgs::TransformStamped &UAV2_pose) {
    // Similar to UAV1 but for second UAV
    // Update position display
    ui.lineEdit_4->setText(QString::number(UAV2_pose.transform.translation.x));
    ui.lineEdit_5->setText(QString::number(UAV2_pose.transform.translation.y));
    ui.lineEdit_6->setText(QString::number(UAV2_pose.transform.translation.z));

    // Convert coordinates with same transformation as UAV1
    float temp_x = UAV2_pose.transform.translation.y * 100 + 500.0;
    float temp_y = UAV2_pose.transform.translation.x * 100 + 300.0;
    QPointF new_position(temp_x, temp_y);

    // Validate and display position
    if (isPositionInBounds(new_position)) {
        addPositionToScene(new_position, Qt::green);
    } else {
        logPositionError(new_position);
    }
}

bool MainWindow::isPositionInBounds(const QPointF &position) const {
    // Check if position is within the scene boundaries
    return position.x() >= scene_->sceneRect().left() && 
           position.x() <= scene_->sceneRect().right() &&
           position.y() >= scene_->sceneRect().top() && 
           position.y() <= scene_->sceneRect().bottom();
}

// Helper function for position error logging
void MainWindow::logPositionError(const QPointF &position) const {
    // Log detailed position error information using ROS logging
    ROS_ERROR("Drone position out of range");
    // Check and log specific boundary violations
    if(position.x() < scene_->sceneRect().left()) {
        ROS_ERROR("position.x()=\t%f", position.x());
        ROS_ERROR("scene_->sceneRect().left()=\t%f", scene_->sceneRect().left());
    }
    if(position.x() > scene_->sceneRect().right()){
        ROS_ERROR("position.x()=\t%f",position.x());
        ROS_ERROR("scene_->sceneRect().right()=\t%f",scene_->sceneRect().right());
    }
    if(position.y() < scene_->sceneRect().top()){
        ROS_ERROR("position.y()=\t%f",position.y());
        ROS_ERROR("scene_->sceneRect().top()=\t%f",scene_->sceneRect().top());
    }
    if(position.y() > scene_->sceneRect().bottom()){
        ROS_ERROR("position.y()=\t%f",position.y());
        ROS_ERROR("scene_->sceneRect().bottom()=\t%f",scene_->sceneRect().bottom());
    }
}

void MainWindow::addPositionToScene(const QPointF &position, const QColor &color = Qt::blue) {
    QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(position.x() - 5, position.y() - 5, 10, 10);
    ellipse->setBrush(color);
    scene_->addItem(ellipse);
}

void MainWindow::drawCirclesFromTable() {
    QPainter painter(&pixmap_);

    // Define a set of colors for different indices
    QVector<QColor> colors = {Qt::red, Qt::green, Qt::blue, Qt::yellow, Qt::magenta};

    QPointF previousPoint(-1, -1);  // Initialize with an invalid point
    int previousIndex = -1;  // Track the previous index

    for (int row = 0; row < ui.tableWidget->rowCount(); ++row) {
        QTableWidgetItem *indexItem = ui.tableWidget->item(row, 0); // Column index for Index
        QTableWidgetItem *yItem = ui.tableWidget->item(row, 1); // Column index for X
        QTableWidgetItem *xItem = ui.tableWidget->item(row, 2); // Column index for Y

        if (indexItem && xItem && yItem) {
            bool indexOk, xOk, yOk;
            int index = indexItem->text().toInt(&indexOk);
            double x = xItem->text().toDouble(&xOk) * 100 + 500;
            double y = yItem->text().toDouble(&yOk) * 100 + 300;

            if (indexOk && xOk && yOk) {
                // Choose the color based on the index
                QColor color = colors[index % colors.size()];
                painter.setBrush(QBrush(color));
                painter.setPen(QPen(color, 2));  // Set pen color and width for drawing lines

                QPointF position(x, y);
                painter.drawEllipse(position, 5, 5); // Draw a small circle

                // Draw line connecting to the previous point if it has the same index
                if (previousPoint != QPointF(-1, -1) && previousIndex == index) {
                    painter.drawLine(previousPoint, position);
                }

                previousPoint = position;  // Update the previous point
                previousIndex = index;  // Update the previous index
            }
        }
    }

    image_label_->setPixmap(pixmap_); // Set the modified pixmap_ back to the label
}

void MainWindow::setupTableWidget()
{
    // Set the number of columns
    ui.tableWidget->setColumnCount(13);

    // Set column headers
    QStringList headers = {"UAV Id", "X", "Y", "Z", "task_type", "hold_time", "end_eff_type", 
                    "end_eff_switch_bef", "end_eff_switch", "end_eff_switch_aft", "end_dock_offset", "docking_pick_change", "path_time"};
    ui.tableWidget->setHorizontalHeaderLabels(headers);
    ui.tableWidget->setRowCount(0);  // Initially, there are no rows

    // Enable row selection
    ui.tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    ui.tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);

    // Adjust column widths automatically
    ui.tableWidget->resizeColumnsToContents();
}


bool MainWindow::eventFilter(QObject *object, QEvent *event) {
    if (object == image_label_ && event->type() == QEvent::MouseButtonPress) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        QPoint pos = mouseEvent->pos();
        addPoint(pos);  // add the point to the table
        return true;
    }
    return QMainWindow::eventFilter(object, event);
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
    QPointF point = view_->mapToScene(event->pos());
    addPoint(point.toPoint());
}

void MainWindow::addPoint(QPoint point) {
    int rowCount = ui.tableWidget->rowCount();
    ui.tableWidget->insertRow(rowCount);

    // Retrieve the current selected index of a QComboBox component.
    int comboBoxIndex = ui.comboBox->currentIndex();

    // Fill the index from QComboBox as indexItem into the table.
    QTableWidgetItem *indexItem = new QTableWidgetItem(QString::number(comboBoxIndex));
    ui.tableWidget->setItem(rowCount, 0, indexItem);

    // need to switch X and Y cause to different frame
    QTableWidgetItem *xItem = new QTableWidgetItem(QString::number((point.y() - 300) * 0.01));
    ui.tableWidget->setItem(rowCount, 1, xItem);

    QTableWidgetItem *yItem = new QTableWidgetItem(QString::number((point.x() - 500) * 0.01));
    ui.tableWidget->setItem(rowCount, 2, yItem);

    QTableWidgetItem *zItem = new QTableWidgetItem(QString::number(0)); // Z default is 0
    ui.tableWidget->setItem(rowCount, 3, zItem);
    // # task type: 0->move, 1->dock, 2->manipulation, 3->move and wait, 4->release tool  
    QTableWidgetItem *task_typeItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 4, task_typeItem);
    // # holding time for end-effector working 
    QTableWidgetItem *hold_timeItem = new QTableWidgetItem(QString::number(2));
    ui.tableWidget->setItem(rowCount, 5, hold_timeItem);
    // # number of object ID.  if end_eff_type> 4, it means the target is a manipulation target.
    QTableWidgetItem *end_eff_typeItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 6, end_eff_typeItem);
    // # end-effector is open or close in before manipulation. griper: true-> open, false -> close. Only valide for the manipulaton.
    QTableWidgetItem *end_eff_switch_befItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 7, end_eff_switch_befItem);
    // # end-effector is open or close. true-> open, false -> close. Only valide for the manipulaton.
    QTableWidgetItem *end_eff_switchItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 8, end_eff_switchItem);
    // # end-effector is open or close in after manipulation. griper: true-> open, false -> close. Only valide for the manipulaton. 
    QTableWidgetItem *end_eff_switch_aftItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 9, end_eff_switch_aftItem);
    // # offset for docking and manipulation ID 1->2.8 ID 2- >0.036 
    QTableWidgetItem *end_dock_offsetItem = new QTableWidgetItem(QString::number(0.05));
    ui.tableWidget->setItem(rowCount, 10, end_dock_offsetItem);
    // # pick up or change end-effector : false --- pick, true --- change
    QTableWidgetItem *docking_pick_changeItem = new QTableWidgetItem(QString::number(1));
    ui.tableWidget->setItem(rowCount, 11, docking_pick_changeItem);
    // fly time from one point to another
    QTableWidgetItem *path_timeItem = new QTableWidgetItem(QString::number(3));
    ui.tableWidget->setItem(rowCount, 12, path_timeItem);
    // Redraw circles
    drawCirclesFromTable();

    // add max_track_id_
    max_track_id_++;
}

void MainWindow::removeRow() {
    // Get the range of selected cells
    QList<QTableWidgetItem*> selectedItems = ui.tableWidget->selectedItems();

    // Display warning if no cells are selected
    if (selectedItems.isEmpty()) {
        QMessageBox::warning(this, "Warning", "No row selected to delete.");
        return;
    }

    // Remove the selected row if valid
    int currentRow = ui.tableWidget->currentRow();
    if (currentRow >= 0) {
        ui.tableWidget->removeRow(currentRow);
        // Update visualization after row removal
        drawCirclesFromTable();
    }
}
void MainWindow::Collision_Detection() {
    collision_threshold_ = qnode_.getCollisionThreshold();  // Get threshold from ROS params
    // Map structure to store trajectory data for each UAV
    // Key: UAV ID, Value: Map of time points to 3D positions
    std::map<int, std::map<double, QVector3D>> allTimePositionMaps;

    // Process each waypoint from the table
    for (int row = 0; row < ui.tableWidget->rowCount(); ++row) {
        QTableWidgetItem *indexItem = ui.tableWidget->item(row, 0); // index
        QTableWidgetItem *xItem = ui.tableWidget->item(row, 1); // Position X
        QTableWidgetItem *yItem = ui.tableWidget->item(row, 2); // Position Y
        QTableWidgetItem *zItem = ui.tableWidget->item(row, 3); // Position Z
        QTableWidgetItem *holdTimeItem = ui.tableWidget->item(row, 5); // hold_time
        QTableWidgetItem *pathTimeItem = ui.tableWidget->item(row, 12); // path_time

        if (indexItem && xItem && yItem && zItem && holdTimeItem) {
            bool indexOk, xOk, yOk, zOk, holdOk, pathOk;

            int index = indexItem->text().toInt(&indexOk);
            double x = xItem->text().toDouble(&xOk);
            double y = yItem->text().toDouble(&yOk);
            double z = zItem->text().toDouble(&zOk);
            double holdTime = holdTimeItem->text().toDouble(&holdOk);
            double pathTime = 0.0;

            if (row > 0 && pathTimeItem) {
                pathTime = pathTimeItem->text().toDouble(&pathOk);
            }

            if (indexOk && xOk && yOk && zOk && holdOk) {
                QVector3D position(x, y, z);

                // Get or create map for corresponding index
                std::map<double, QVector3D> &timePositionMap = allTimePositionMaps[index];

                // Calculate cumulative time
                double cumulativeTime = timePositionMap.empty() ? 0.0 : timePositionMap.rbegin()->first;
                qDebug() << "index:" << index << "timePositionMap.empty():" << timePositionMap.empty() << "cumulativeTime:" << cumulativeTime;

                // If not the first point and belongs to the same trajectory, perform linear interpolation
                if (row > 0 && ui.tableWidget->item(row - 1, 0)->text().toInt() == index) {
                    QTableWidgetItem *prevXItem = ui.tableWidget->item(row - 1, 1);
                    QTableWidgetItem *prevYItem = ui.tableWidget->item(row - 1, 2);
                    QTableWidgetItem *prevZItem = ui.tableWidget->item(row - 1, 3);

                    if (prevXItem && prevYItem && prevZItem) {
                        double prevX = prevXItem->text().toDouble();
                        double prevY = prevYItem->text().toDouble();
                        double prevZ = prevZItem->text().toDouble();
                        QVector3D prevPosition(prevX, prevY, prevZ);

                        // Perform linear interpolation
                        for (double t = 0; t <= pathTime; t += 0.1) {
                            double ratio = t / pathTime;
                            double interpX = prevX + ratio * (x - prevX);
                            double interpY = prevY + ratio * (y - prevY);
                            double interpZ = prevZ + ratio * (z - prevZ);
                            QVector3D interpPosition(interpX, interpY, interpZ);

                            timePositionMap[cumulativeTime + t] = interpPosition;
                        }
                        // Update cumulative time only for the same trajectory
                        cumulativeTime += pathTime;
                    }
                }

                // Fill time-position mapping every 0.1 seconds
                for (double t = 0; t <= holdTime; t += 0.1) {
                    timePositionMap[cumulativeTime + t] = position;
                }

                // Update cumulative time by adding hold_time
                cumulativeTime += holdTime;
            }
        }
    }

    // Ensure image is loaded correctly
    if (pixmap_.isNull()) {
        QMessageBox::warning(this, "Warning", "Failed to load image.");
        return;
    }

    // Create a copy of pixmap_ for drawing
    QPixmap updatedPixmap = pixmap_;
    QPainter painter(&updatedPixmap);

    // Define different colors for different trajectories
    QVector<QColor> colors = {Qt::red, Qt::green, Qt::blue, Qt::yellow, Qt::magenta};

    // Draw all time-position mappings
    int colorIndex = 0;
    for (const auto &mapEntry : allTimePositionMaps) {
        int uavIndex = mapEntry.first;  // Get UAV index
        QColor color = colors[uavIndex % colors.size()];  // Get color based on UAV index
        
        painter.setBrush(QBrush(color));
        painter.setPen(QPen(color, 2));  // Set pen color and width

        for (const auto &entry : mapEntry.second) {
            QVector3D position_meter = entry.second;
            qDebug() << "UAV:" << mapEntry.first << "Time:" << entry.first << "Position:" << position_meter;

            double y = position_meter.x() * 100 + 300;
            double x = position_meter.y() * 100 + 500;
            QPointF position(x, y);
            painter.drawEllipse(position, 5, 5); // Draw position marker
        }

        ++colorIndex;
    }

    // Update QLabel with the modified pixmap
    image_label_->setPixmap(updatedPixmap);

    // Collision detection
    for (const auto &timeEntry : allTimePositionMaps) {
        const std::map<double, QVector3D>& timePositionMap = timeEntry.second;
        for (const auto &innerTimeEntry : timePositionMap) {
            double currentTime = innerTimeEntry.first;
            const QVector3D &currentPosition = innerTimeEntry.second;

            // Check positions of other UAVs at the same time point
            for (const auto &otherEntry : allTimePositionMaps) {
                if (otherEntry.first != timeEntry.first) {  // Exclude same UAV
                    const auto &otherTimeMap = otherEntry.second;
                    auto it = otherTimeMap.find(currentTime);
                    if (it != otherTimeMap.end()) {
                        const QVector3D &otherPosition = it->second;
                        if ((currentPosition - otherPosition).length() < collision_threshold_) { // Check if distance is less than collision_threshold_
                            QMessageBox::warning(this, "Collision Warning",
                                                 QString("Collision detected!\n"
                                                         "Time: %1\n"
                                                         "UAV %2 Position: (%3, %4, %5)\n"
                                                         "UAV %6 Position: (%7, %8, %9)")
                                                     .arg(currentTime)
                                                     .arg(timeEntry.first)
                                                     .arg(currentPosition.x())
                                                     .arg(currentPosition.y())
                                                     .arg(currentPosition.z())
                                                     .arg(otherEntry.first)
                                                     .arg(otherPosition.x())
                                                     .arg(otherPosition.y())
                                                     .arg(otherPosition.z()));
                            return; // Stop further collision detection
                        }
                    }
                }
            }
        }
    }
}

void MainWindow::generateYamlFile()
{
    QFile file("corridor.yaml");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Error", "Failed to open file for writing.");
        return;
    }

    QTextStream out(&file);

    // Write YAML file header
    out << "# This file is used for describing the corridor in this task.\n";
    out << "# number of path\n";
    
    // Write num_path
    int rowCount = ui.tableWidget->rowCount();
    int numPath = (rowCount > 1) ? rowCount - 1 : 0;
    out << "num_path: " << numPath << "\n";
    out << "# data of the task"  << "\n";

    // Write task data
    out << "task_data:\n";
    for (int i = 0; i < rowCount - 1; ++i) {
        out << "# task " << (i + 1) << "\n";
        out << "  # start point" << "\n";
        out << "  - start_pt:\n";
        out << "      pos: [" << ui.tableWidget->item(i, 1)->text() << ", "
            << ui.tableWidget->item(i, 2)->text() << ", "
            << ui.tableWidget->item(i, 3)->text() << "]\n";
        out << "      vel: [0.0, 0.0, 0.0]\n";
        out << "      acc: [0.0, 0.0, 0.0]\n";
        out << "  # end point " << "\n";
        out << "    end_pt:\n";
        out << "      pos: [" << ui.tableWidget->item(i + 1, 1)->text() << ", "
            << ui.tableWidget->item(i + 1, 2)->text() << ", "
            << ui.tableWidget->item(i + 1, 3)->text() << "]\n";
        out << "      vel: [0.0, 0.0, 0.0]\n";
        out << "      acc: [0.0, 0.0, 0.0]\n";

        // Get task parameters with default values
        int taskType = ui.tableWidget->item(i, 4) ? ui.tableWidget->item(i, 4)->text().toInt() : 1;
        float holdTime = ui.tableWidget->item(i, 5) ? ui.tableWidget->item(i, 5)->text().toFloat() : 5.0;
        int endEffType = ui.tableWidget->item(i, 6) ? ui.tableWidget->item(i, 6)->text().toInt() : 1;
        bool endEffSwitchBef = ui.tableWidget->item(i, 7) ? ui.tableWidget->item(i, 7)->text().toInt() : 0;
        bool endEffSwitch = ui.tableWidget->item(i, 8) ? ui.tableWidget->item(i, 8)->text().toInt() : 0;
        bool endEffSwitchAft = ui.tableWidget->item(i, 9) ? ui.tableWidget->item(i, 9)->text().toInt() : 0;
        float endDockOffset = ui.tableWidget->item(i, 10) ? ui.tableWidget->item(i, 10)->text().toFloat() : 0.050;
        bool dockingPickChange = ui.tableWidget->item(i, 11) ? ui.tableWidget->item(i, 11)->text() == "true" : false;

        out << "  # task type: 0->move, 1->dock, 2->manipulation, 3->move and wait, 4->release tool  " << "\n";
        out << "    task_type: " << taskType << "\n";
        out << "  # holding time for end-effector working " << "\n";
        out << "    hold_time: " << QString::number(holdTime, 'f', 1) << "\n";
        out << "  # number of object ID.  if end_eff_type> 4, it means the target is a manipulation target." << "\n";
        out << "    end_eff_type: " << endEffType << "\n";
        out << "  # end-effector is open or close in before manipulation. griper: true-> open, false -> close. Only valide for the manipulaton." << "\n";
        out << "    end_eff_switch_bef: " << (endEffSwitchBef ? "true" : "false") << "\n";
        out << "  # end-effector is open or close. true-> open, false -> close. Only valide for the manipulaton." << "\n";
        out << "    end_eff_switch: " << (endEffSwitch ? "true" : "false") << "\n";
        out << "  # end-effector is open or close in after manipulation. griper: true-> open, false -> close. Only valide for the manipulaton. " << "\n";
        out << "    end_eff_switch_aft: " << (endEffSwitchAft ? "true" : "false") << "\n";
        out << "  # offset for docking and manipulation ID 1->2.8 ID 2- >0.036 " << "\n";
        out << "    end_dock_offset: " << endDockOffset << "\n";
        out << "  # pick up or change end-effector : false --- pick, true --- change" << "\n";
        out << "  # if end_eff_type == 0, docking_pick_change is useless" << "\n";
        out << "    docking_pick_change: " << (dockingPickChange ? "true" : "false") << "\n";
 
        out << "  # corridor " << "\n";
        // Write corridor data

        out << "    num_polyhedron: 1\n";
        out << "    corridor:\n";
        out << "  # corridor 1" << "\n";
        out << "      - s_scale: 8.0\n";
        out << "        upper_vel: [1.0,1.0,1.0]\n";
        out << "        lower_vel: [-1.0, -1.0, -1.0]\n";
        out << "        upper_acc: [1.0,1.0,1.0]\n";
        out << "        lower_acc: [-1.0, -1.0, -1.0]\n";
        out << "        A_pos: \n";
        out << "          row: 3\n";
        out << "          col: 3\n";
        out << "          data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0,0.0, 0.0, 1.0]\n";
        out << "        b_u_pos: [1.0, 2.0, -1.0]\n";
        out << "        b_l_pos: [-1.0, -2.0, -3.0]\n\n";
    }

    file.close();
    QMessageBox::information(this, "Success", "corridor.yaml has been generated.");
}


}  // namespace task_planning
