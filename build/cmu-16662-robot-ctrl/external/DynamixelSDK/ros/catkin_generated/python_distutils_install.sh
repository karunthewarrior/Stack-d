#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/winata/Stack-d/src/cmu-16662-robot-ctrl/external/DynamixelSDK/ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/winata/Stack-d/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/winata/Stack-d/install/lib/python2.7/dist-packages:/home/winata/Stack-d/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/winata/Stack-d/build" \
    "/usr/bin/python" \
    "/home/winata/Stack-d/src/cmu-16662-robot-ctrl/external/DynamixelSDK/ros/setup.py" \
    build --build-base "/home/winata/Stack-d/build/cmu-16662-robot-ctrl/external/DynamixelSDK/ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/winata/Stack-d/install" --install-scripts="/home/winata/Stack-d/install/bin"
