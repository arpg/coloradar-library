#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${BAG_PATH:-/bags/your_file.bag}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"
VIEWER="${VIEWER:-rqt}"         # rqt | rviz | image_view
PLAY_ARGS="${PLAY_ARGS:---clock}"
LOOP="${LOOP:-1}"

pids=()
cleanup() { kill -INT "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "[bag_viewer] Starting roscore..."
roscore &
pids+=($!)

# Wait for ROS master to be ready
until rostopic list >/dev/null 2>&1; do sleep 0.2; done
echo "[bag_viewer] ROS master is up."

# If using simulated time, set param so viewers sync to /clock
if [[ "$PLAY_ARGS" == *"--clock"* ]]; then
  rosparam set use_sim_time true
fi

# Launch viewer inside the same container
case "$VIEWER" in
  rqt)
    echo "[bag_viewer] Launching rqt_image_view..."
    rosrun rqt_image_view rqt_image_view &
    pids+=($!)
    ;;
  rviz)
    echo "[bag_viewer] Launching RViz..."
    rviz &
    pids+=($!)
    ;;
  image_view)
    echo "[bag_viewer] Launching image_view on $IMAGE_TOPIC (mono8)..."
    # Force raw transport, don’t let it guess
    rosrun image_view image_view image:="$IMAGE_TOPIC" _image_transport:=raw &
    pids+=($!)
    ;;
  *)
    echo "[bag_viewer] VIEWER=$VIEWER not recognized; skipping GUI."
    ;;
esac

# Play the bag (foreground). If LOOP=1, loop forever.
echo "[bag_viewer] Playing bag: $BAG_PATH"
if [[ "$LOOP" == "1" ]]; then
  rosbag play -l $PLAY_ARGS "$BAG_PATH"
else
  rosbag play $PLAY_ARGS "$BAG_PATH"
fi
