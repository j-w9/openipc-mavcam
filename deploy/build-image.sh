#!/bin/bash
# Build a flashable OpenIPC firmware package with mavcam added.
#
#   deploy/build-image.sh runcam-wifilink emax-wyvern-link
#   deploy/build-image.sh ssc338q_waybeam_generic-nor      # any SSC338Q package by its full name
#
# Takes OpenIPC's released wfb-ng firmware for each device, adds /usr/bin/mavcam,
# teaches the launcher a "mavcam" telemetry router and selects it, and repacks
# the root filesystem. The kernel is passed through untouched.
#
# Output: deploy/out/<package>-mavcam.tgz, in the layout the
# camera's own updater takes:   sysupgrade --archive=/tmp/<file>.tgz
#
# Needs Docker. Downloads are cached in deploy/cache.
set -euo pipefail

[ $# -ge 1 ] || { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
mkdir -p "$HERE/cache" "$HERE/out"

docker run --rm --platform linux/amd64 \
    -v "$ROOT":/src:ro -v "$HERE/cache":/cache -v "$HERE/out":/out \
    -e ROUTER_DEFAULT="${ROUTER_DEFAULT:-mavcam}" \
    ubuntu:24.04 bash -euo pipefail -c '
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq squashfs-tools make patch wget xz-utils ca-certificates file >/dev/null

# ---- toolchain and mavcam ----
TC=toolchain.sigmastar-infinity6e
[ -f /cache/$TC.tgz ] || wget -q -O /cache/$TC.tgz \
    https://github.com/openipc/firmware/releases/download/toolchain/$TC.tgz
mkdir -p /tc && tar xzf /cache/$TC.tgz -C /tc
CC=$(echo /tc/*/bin/arm-openipc-linux-gnueabihf-gcc)
mkdir /build && cp -r /src/src /src/third_party /src/Makefile /build/
rm -f /build/src/*.o /build/mavcam
make -s -C /build CC="$CC" CFLAGS=-Os
"${CC%gcc}strip" /build/mavcam
echo "mavcam: $(file -b /build/mavcam | cut -d, -f1-2), $(stat -c %s /build/mavcam) bytes"

for dev in "$@"; do
    case "$dev" in
        ssc338q_*) base=$dev ;;
        *) base=ssc338q_wfbng_${dev}-nor ;;
    esac
    echo
    echo "== $base"
    [ -f /cache/$base.tgz ] || wget -q -O /cache/$base.tgz \
        https://github.com/OpenIPC/builder/releases/download/latest/$base.tgz
    rm -rf /w && mkdir /w && tar xzf /cache/$base.tgz -C /w
    (cd /w && md5sum -c --quiet rootfs.squashfs.ssc338q.md5sum uImage.ssc338q.md5sum)
    unsquashfs -d /w/root /w/rootfs.squashfs.ssc338q >/dev/null

    grep -E "^(BUILD_ID|BUILD_PLATFORM)=" /w/root/etc/os-release | sed "s/^/base: /"
    orig_size=$(stat -c %s /w/rootfs.squashfs.ssc338q)

    # ---- the three changes ----
    install -m 0755 /build/mavcam /w/root/usr/bin/mavcam
    patch -s --no-backup-if-mismatch /w/root/usr/bin/wifibroadcast < /src/deploy/wifibroadcast-mavcam.patch
    grep -q "router\" = \"mavcam\"" /w/root/usr/bin/wifibroadcast
    grep -q "msposd mavfwd mavcam" /w/root/usr/bin/wifibroadcast
    sh -n /w/root/usr/bin/wifibroadcast
    grep -q "^  router: " /w/root/etc/wfb.yaml
    sed -i "s/^  router: .*/  router: $ROUTER_DEFAULT/" /w/root/etc/wfb.yaml
    printf "mavcam added to %s by deploy/build-image.sh\n" "$base" > /w/root/etc/mavcam-release

    # ---- repack exactly as the original: xz, 128 KiB blocks ----
    rm /w/rootfs.squashfs.ssc338q /w/*.md5sum
    mksquashfs /w/root /w/rootfs.squashfs.ssc338q -comp xz -b 131072 -noappend -no-progress >/dev/null
    new_size=$(stat -c %s /w/rootfs.squashfs.ssc338q)
    echo "rootfs: $orig_size -> $new_size bytes ($(( (new_size - orig_size) / 1024 )) KiB larger)"
    # The stock image is already close to 8 MiB. Refuse to grow past that rather
    # than rely on the partition being bigger; the updater checks again on the camera.
    if [ "$new_size" -gt $((8192 * 1024)) ]; then
        echo "rootfs is over 8192 KiB; not packaging" >&2
        exit 1
    fi
    (cd /w && md5sum rootfs.squashfs.ssc338q > rootfs.squashfs.ssc338q.md5sum \
           && md5sum uImage.ssc338q > uImage.ssc338q.md5sum)
    tar czf /out/$base-mavcam.tgz -C /w rootfs.squashfs.ssc338q rootfs.squashfs.ssc338q.md5sum \
        uImage.ssc338q uImage.ssc338q.md5sum
    echo "wrote deploy/out/$base-mavcam.tgz ($(stat -c %s /out/$base-mavcam.tgz) bytes)"
done
' build-image "$@"
