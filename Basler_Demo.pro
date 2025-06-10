QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    BaslerCameraControl.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    BaslerCameraControl.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += $$PWD/pylon/include
INCLUDEPATH += $$PWD/pylon/lib/x64
DEPENDPATH += $$PWD/pylon/lib/x64
win32: LIBS += -L$$PWD/pylon/lib/x64/ -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lgxapi_v15 -lPylonBase_v9 -lPylonC_v9 -lPylonDataProcessing_v3 -lPylonGUI_v9 -lPylonUtility_v9 -luxapi_v14 -luxtopapi_v9

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/opencv/x64/vc16/lib/ -lopencv_world4100
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/opencv/x64/vc16/lib/ -lopencv_world4100d
INCLUDEPATH += $$PWD/opencv/include
DEPENDPATH += $$PWD/opencv/include
