FROM python:3.12-alpine

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /opt/pulse1

COPY scripts/pulse1_health_score.py ./
COPY scripts/pulse1_health_v2.py ./
COPY services/diagnostic-api/api.py ./

USER 65534:65534

EXPOSE 8000

CMD ["python", "api.py"]
