ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}
ARG PROFILE=runtime
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg
COPY config/setup/ /opt/jt16-setup/config/setup/
COPY scripts/install_dependencies.sh /opt/jt16-setup/scripts/install_dependencies.sh
ARG UBUNTU_MIRROR
RUN bash /opt/jt16-setup/scripts/install_dependencies.sh ${PROFILE}
WORKDIR /workspace/jt16
CMD ["bash"]
