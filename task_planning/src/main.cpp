/**
 * @file /src/main.cpp
 * @brief Qt based GUI application
 * @date November 2024
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

/*****************************************************************************
 * Main Function
 *****************************************************************************/
int main(int argc, char **argv) 
{
    // Initialize Qt Application
    QApplication app(argc, argv);
    
    // Create and show main window
    task_planning::MainWindow w(argc, argv);
    w.show();
    
    // Connect quit signal when last window is closed
    app.connect(&app, SIGNAL(lastWindowClosed()), 
               &app, SLOT(quit()));

    // Start event loop and get exit code
    int result = app.exec();
    
    return result;
}