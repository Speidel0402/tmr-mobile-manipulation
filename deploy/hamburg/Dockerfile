FROM --platform=linux/arm64 ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-rosidl-runtime-py \
       ros-humble-common-interfaces \
       ros-humble-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tmr-task3/hamburg
COPY deploy/hamburg/ /opt/tmr-task3/hamburg/
RUN chmod +x /opt/tmr-task3/hamburg/run_hamburg.sh \
    && python3 -m compileall -q /opt/tmr-task3/hamburg

ENTRYPOINT ["/ros_entrypoint.sh", "/opt/tmr-task3/hamburg/run_hamburg.sh"]
CMD ["check"]
