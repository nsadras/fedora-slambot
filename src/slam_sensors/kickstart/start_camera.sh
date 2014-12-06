#!/bin/bash
while [ 0 ]; do
    rosservice call /camera/start_capture && break
    sleep 1
done;
exec "$@"
