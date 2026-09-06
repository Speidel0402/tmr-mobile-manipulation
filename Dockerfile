FROM python:3.11-slim-bookworm

LABEL org.opencontainers.image.title="EDL Team EBiM Task 3 Phase 2"
LABEL org.opencontainers.image.description="Containerized coordinator for the EBiM Task 3 Phase 2 Stage 1 policy"
LABEL org.opencontainers.image.source="https://github.com/Speidel0402/tmr-mobile-manipulation"

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends bash ca-certificates openssh-client tar \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ebim-policy
COPY . .

RUN chmod +x docker/run_task3.sh mission/scripts/run_complete_from_start.sh \
    && bash -n docker/run_task3.sh mission/scripts/run_complete_from_start.sh \
    && python -m compileall -q base/scripts grasp/scripts mission/scripts tools

ENTRYPOINT ["bash", "docker/run_task3.sh"]
CMD ["--cup-letter", "B", "--bowl-letter", "A", "--plate-letter", "D"]
