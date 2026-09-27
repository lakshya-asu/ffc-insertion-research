FROM nvcr.io/nvidia/isaac-sim:6.1.0@sha256:0c16dd67d09a70ea474f2c809a13c4d09bd23c738184cf1bf28af487ecd39080

USER root
# This image runs finite experiments, not the vendor streaming service.
# Exit codes and audited result files carry experiment status.
HEALTHCHECK NONE
ENV DEBIAN_FRONTEND=noninteractive \
    ACCEPT_EULA=Y \
    OMNI_KIT_ACCEPT_EULA=YES \
    PRIVACY_CONSENT=N \
    PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*
RUN /isaac-sim/python.sh -m pip install --no-cache-dir \
    pytest==9.1.1 ruff==0.16.9 scipy==1.17.1 pillow==12.3.0 mujoco==3.12.0 matplotlib==3.11.2
WORKDIR /workspace
RUN find /isaac-sim/extscache -path '*/omni.kit.pip_archive-*/pip_prebundle/numpy.libs' \
    > /etc/ld.so.conf.d/isaac-numpy.conf \
    && echo /isaac-sim/kit/python/lib/python3.12/site-packages/scipy.libs >> /etc/ld.so.conf.d/isaac-numpy.conf \
    && ldconfig \
    && /isaac-sim/python.sh -c "import numpy, scipy; print(numpy.__version__, scipy.__version__)"
COPY config/fr3-source.json config/fr3-isaac-source.json /workspace/config/
COPY scripts/fetch_fr3.py scripts/fetch_fr3_isaac.py scripts/patch_isaac_entrypoint.py /workspace/scripts/
RUN /isaac-sim/python.sh /workspace/scripts/patch_isaac_entrypoint.py
RUN /isaac-sim/python.sh /workspace/scripts/fetch_fr3_isaac.py \
    && /isaac-sim/python.sh /workspace/scripts/fetch_fr3.py
COPY config /workspace/config
COPY src /workspace/src
COPY scripts /workspace/scripts
COPY tests /workspace/tests
COPY pyproject.toml uv.lock Dockerfile compose.yaml README.md ISAAC.md /workspace/
COPY MORNING_REPORT.md CALIBRATION.md hardware-selection.md gripper-concepts.md task-specification.md /workspace/
COPY experiments /workspace/experiments
RUN /workspace/scripts/usd_python.sh -m pytest -q
ENTRYPOINT ["/isaac-sim/python.sh"]
CMD ["scripts/isaac_workcell.py", "--exercise-tool"]
