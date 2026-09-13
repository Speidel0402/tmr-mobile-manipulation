FROM --platform=linux/arm64 ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-rosidl-runtime-py \
       ros-humble-common-interfaces \
       ros-humble-tf2-msgs \
       python3-numpy \
       python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tmr-task3
COPY deploy/hamburg/ /opt/tmr-task3/deploy/hamburg/
COPY grasp/scripts/cup_rim_detector.py grasp/scripts/detect_food_bowl.py grasp/scripts/detect_plate.py /opt/tmr-task3/grasp/scripts/
RUN chmod +x /opt/tmr-task3/deploy/hamburg/run_hamburg.sh \
    && python3 -m compileall -q /opt/tmr-task3/deploy/hamburg /opt/tmr-task3/grasp/scripts

ENTRYPOINT ["/ros_entrypoint.sh", "/opt/tmr-task3/deploy/hamburg/run_hamburg.sh"]
CMD ["check"]
