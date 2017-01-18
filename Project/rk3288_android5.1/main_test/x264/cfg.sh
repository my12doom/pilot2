./configure \
  --cross-prefix="/toolchain-android/bin/arm-linux-androideabi-" \
  --sysroot="/toolchain-android/sysroot" \
  --host="arm-linux" \
  --enable-pic \
  --enable-static \
  --disable-shared \
  --prefix="/toolchain-android" \
  --disable-cli