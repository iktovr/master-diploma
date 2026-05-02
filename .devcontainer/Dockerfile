FROM alpine:latest AS font-builder

RUN apk --update add msttcorefonts-installer \
  && update-ms-fonts 

FROM mcr.microsoft.com/devcontainers/cpp:ubuntu

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
  && echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections \
  && fc-cache -f -v \
  && wget https://github.com/bazelbuild/bazelisk/releases/download/v1.29.0/bazelisk-linux-amd64 \
    https://github.com/bazelbuild/buildtools/releases/download/v8.5.1/buildifier-linux-amd64 \
    https://github.com/bazelbuild/buildtools/releases/download/v8.5.1/buildozer-linux-amd64 \
  && chmod 755 bazelisk-linux-amd64 buildifier-linux-amd64 buildozer-linux-amd64 \
  && mv bazelisk-linux-amd64 /usr/bin/bazel \
  && mv buildifier-linux-amd64 /usr/bin/buildifier \
  && mv buildozer-linux-amd64 /usr/bin/buildozer
