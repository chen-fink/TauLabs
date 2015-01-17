TEMPLATE = lib
TARGET = Misc
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += Misc.json

HEADERS += \
    miscplugin.h \
    naze32pro.h

SOURCES += \
    miscplugin.cpp \
    naze32pro.cpp

RESOURCES += \
    misc.qrc
