QT       += core gui 3dcore 3dextras 3drender 3dinput widgets charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# Пути к Boost
INCLUDEPATH += C:/local/boost_1_87_0
LIBS += -LC:/local/boost_1_87_0/stage/lib

SOURCES += \
    firstmod.cpp \
    firstmod2.cpp \
    firstmod3.cpp \
    main.cpp \
    mainwindow.cpp \
    secondmod.cpp \
    secondmod2.cpp \
    secondwindow.cpp

HEADERS += \
    firstmod.h \
    firstmod2.h \
    firstmod3.h \
    flight_model.h \
    mainwindow.h \
    secondmod.h \
    secondmod2.h \
    secondwindow.h

FORMS += \
    firstmod.ui \
    firstmod2.ui \
    firstmod3.ui \
    mainwindow.ui \
    secondmod.ui \
    secondmod2.ui \
    secondwindow.ui

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
