FROM alpine:latest AS font-builder

RUN apk --update add msttcorefonts-installer \
  && update-ms-fonts

FROM mcr.microsoft.com/devcontainers/cpp:ubuntu

ARG TARGETOS TARGETARCH

COPY --from=font-builder /usr/share/fonts/truetype/msttcorefonts /usr/share/fonts/truetype/msttcorefonts

RUN chmod -R 644 /usr/share/fonts/truetype/msttcorefonts/* \
  && apt-get update && export DEBIAN_FRONTEND=noninteractive \
  && apt-get install --no-install-recommends --assume-yes \
    texlive-latex-base \
    texlive-xetex \
    texlive-latex-extra \
    texlive-plain-generic \
    texlive-lang-cyrillic \
    texlive-science \
    texlive-bibtex-extra \
    biber \
    latexmk \
    cm-super \
    fonts-liberation \
    texlive-fonts-recommended \
    fontconfig \
    ffmpeg \
    clangd \
    python3-pyosmium \
    python3-networkx \
    python3-pyproj \
    python3-pip \
    python3-venv \
  && echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections \
  && fc-cache -f -v \
  && wget "https://github.com/bazelbuild/bazelisk/releases/download/v1.29.0/bazelisk-${TARGETOS}-${TARGETARCH}" \
    "https://github.com/bazelbuild/buildtools/releases/download/v8.5.1/buildifier-${TARGETOS}-${TARGETARCH}" \
    "https://github.com/bazelbuild/buildtools/releases/download/v8.5.1/buildozer-${TARGETOS}-${TARGETARCH}" \
  && chmod 755 "bazelisk-${TARGETOS}-${TARGETARCH}" "buildifier-${TARGETOS}-${TARGETARCH}" "buildozer-${TARGETOS}-${TARGETARCH}" \
  && mv "bazelisk-${TARGETOS}-${TARGETARCH}" /usr/bin/bazel \
  && mv "buildifier-${TARGETOS}-${TARGETARCH}" /usr/bin/buildifier \
  && mv "buildozer-${TARGETOS}-${TARGETARCH}" /usr/bin/buildozer \
  && install -m 0755 /usr/lib/git-core/git-subtree "$(git --exec-path)/git-subtree"
