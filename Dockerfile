ARG FROM_IMAGE=ros:eloquent

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./install/underlay.repos ./
RUN vcs import src < underlay.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./install/overlay.repos ./
RUN vcs import src < overlay.repos

# copy wrapper source
ENV WRAPPER_WS /opt/wrapper_ws
RUN mkdir -p $WRAPPER_WS/src
WORKDIR $WRAPPER_WS
COPY ./install/wrapper.repos ./
RUN vcs import src < wrapper.repos
COPY ./ src/MIT-SPARK/Kimera-VIO-ROS2

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp
    # && find ./ -name "COLCON_IGNORE" | \
    #   xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
    && rm -rf /var/lib/apt/lists/*

# install opencv dependencies
RUN apt-get update && apt-get install -y \
      gfortran \
      libatlas-base-dev \
      libgtk-3-dev \
      libjpeg-dev \
      libpng-dev \
      libtiff-dev \
      libvtk6-dev \
      unzip \
    && rm -rf /var/lib/apt/lists/*

# copy underlay manifests
ENV UNDERLAY_WS /opt/underlay_ws
COPY --from=cache /tmp/underlay_ws $UNDERLAY_WS
WORKDIR $UNDERLAY_WS

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

# build underlay source
ARG UNDERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_POSE3_EXPMAP=ON \
        -DGTSAM_ROT3_EXPMAP=ON \
        -DOPENCV_EXTRA_MODULES_PATH=$UNDERLAY_WS/src/opencv/opencv/contrib/modules
      # --event-handlers console_direct+

# copy overlay manifests
ENV OVERLAY_WS /opt/overlay_ws
COPY --from=cache /tmp/overlay_ws $OVERLAY_WS
WORKDIR $OVERLAY_WS

# install overlay dependencies
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $OVERLAY_WS ./

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS="\
          -Wno-parentheses \
          -Wno-reorder \
          -Wno-sign-compare \
          -Wno-unused-but-set-variable \
          -Wno-unused-function \
          -Wno-unused-parameter \
          -Wno-unused-value \
          -Wno-unused-variable"
      # --event-handlers console_direct+

# copy wrapper manifests
ENV WRAPPER_WS /opt/wrapper_ws
COPY --from=cache /tmp/wrapper_ws $WRAPPER_WS
WORKDIR $WRAPPER_WS

# install wrapper dependencies
RUN . $OVERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
        $OVERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy wrapper source
COPY --from=cache $WRAPPER_WS ./

# build wrapper source
ARG WRAPPER_MIXINS="release ccache"
RUN . $OVERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $WRAPPER_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS="\
          -Wno-sign-compare \
          -Wno-unused-value \
          -Wno-unused-variable \
          -Wno-unused-but-set-variable \
          -Wno-reorder \
          -Wno-parentheses \
          -Wno-unused-parameter"
      # --event-handlers console_direct+

# source wrapper from entrypoint
RUN sed --in-place \
      's|^source .*|source "$WRAPPER_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
