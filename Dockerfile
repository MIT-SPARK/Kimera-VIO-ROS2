# syntax=docker/dockerfile:experimental
# Example: export DOCKER_BUILDKIT=1
# https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md
# docker build --progress=plain --tag ruffsl/kimera_vio_ros2 .
ARG FROM_IMAGE=osrf/ros2:nightly
ARG UNDERLAY_WS=/opt/ros/underlay_ws
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG WRAPPER_WS=/opt/ros/wrapper_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./install/underlay.repos ../
RUN vcs import ./ < ../underlay.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./install/overlay.repos ../
RUN vcs import ./ < ../overlay.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy wrapper source
ARG WRAPPER_WS
WORKDIR $WRAPPER_WS/src
COPY ./ ./MIT-SPARK/Kimera-VIO-ROS2
COPY ./install/wrapper.repos ../
RUN vcs import ./ < ../wrapper.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache

# install CI dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -q -y \
      ccache \
      lcov

# install underlay dependencies
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src

# build underlay source
COPY --from=cacher $UNDERLAY_WS/src ./src
ARG UNDERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_POSE3_EXPMAP=ON \
        -DGTSAM_ROT3_EXPMAP=ON
      # --event-handlers console_direct+


# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
        $UNDERLAY_WS/src \
      --ignore-src

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS="\
          -Wno-comment \
          -Wno-parentheses \
          -Wno-reorder \
          -Wno-sign-compare \
          -Wno-unused-but-set-variable \
          -Wno-unused-function \
          -Wno-unused-parameter \
          -Wno-unused-value \
          -Wno-unused-variable"
      # --event-handlers console_direct+

# install wrapper dependencies
ARG WRAPPER_WS
WORKDIR $WRAPPER_WS
COPY --from=cacher /tmp/$WRAPPER_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . $OVERLAY_WS/install/setup.sh && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
        $UNDERLAY_WS/src \
        $OVERLAY_WS/src \
      --ignore-src

# build wrapper source
COPY --from=cacher $WRAPPER_WS/src ./src
ARG WRAPPER_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . $OVERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $WRAPPER_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS="\
          -Wno-comment \
          -Wno-sign-compare \
          -Wno-unused-value \
          -Wno-unused-variable \
          -Wno-unused-but-set-variable \
          -Wno-reorder \
          -Wno-parentheses \
          -Wno-unused-parameter"
      # --event-handlers console_direct+

# source wrapper from entrypoint
ENV WRAPPER_WS $WRAPPER_WS
RUN sed --in-place \
      's|^source .*|source "$WRAPPER_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
