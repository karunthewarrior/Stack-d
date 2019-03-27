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

echo_and_run cd "/home/ktw/ws/robauton_example/src/cmu-16662-robot-ctrl/external/DynamixelSDK/ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ktw/ws/robauton_example/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ktw/ws/robauton_example/install/lib/python2.7/dist-packages:/home/ktw/ws/robauton_example/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ktw/ws/robauton_example/build" \
    "/usr/bin/python" \
    "/home/ktw/ws/robauton_example/src/cmu-16662-robot-ctrl/external/DynamixelSDK/ros/setup.py" \
    build --build-base "/home/ktw/ws/robauton_example/build/cmu-16662-robot-ctrl/external/DynamixelSDK/ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ktw/ws/robauton_example/install" --install-scripts="/home/ktw/ws/robauton_example/install/bin"
