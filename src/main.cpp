/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/hg_gui/main_window.hpp"
#include <QSplashScreen>
#include <QElapsedTimer>

/*****************************************************************************
** Main
*****************************************************************************/

int user_graphic_cmd_index = 0;
int scara_movep_group_index = 0;

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
//    QPixmap pixmap(":/images/scara_operation_load.png");
//    QSplashScreen splash(pixmap);
//    splash.show();
//    int delayTime = 5;
//    QElapsedTimer timer;
//    timer.start();
//    while(timer.elapsed() < (delayTime * 500))
//    {
//        app.processEvents();
//    }
    hg_gui::MainWindow w(argc,argv);
    w.show();
//    QTranslator trans;
//    trans.load("qt_zh_CN.qm","/home/pickle/hpro_ws/src/workstation/hg_gui");
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
