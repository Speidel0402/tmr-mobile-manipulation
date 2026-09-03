FROM python:3.11-slim-bookworm

ARG VCS_REF=unknown
LABEL org.opencontainers.image.title="TMR Mobile Manipulation — EBiM Task 3 Phase II" \
      org.opencontainers.image.source="https://github.com/Speidel0402/tmr-mobile-manipulation" \
      org.opencontainers.image.revision="${VCS_REF}"

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

# Robot drivers, ROS and real-time control remain on the configured robot hosts.
# The existing mission coordinator dispatches their work over SSH.
RUN apt-get update \
    && apt-get install -y --no-install-recommends bash ca-certificates openssh-client \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /root/.ssh /state \
    && chmod 700 /root/.ssh

WORKDIR /opt/tmr-mobile-manipulation
COPY . .

# Match the coordinator's existing KeyboardInterrupt recovery path.
STOPSIGNAL SIGINT

# The default command only prints the existing motion-disabled plan.
CMD ["python3", "mission/scripts/run_three_object_delivery.py"]
