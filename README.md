# μsk

Build the firmware
------------------
* Get pico sdk
git clone https://github.com/raspberrypi/pico-sdk
export PICO_SDK_PATH=/pico/sdk/path
cp [SDK]/external/pico_sdk_import.cmake ./

* Get pico extras
git clone https://github.com/raspberrypi/pico-extras
export PICO_EXTRAS_PATH=/pico/extras/path
cp [SDK]/external/pico_extras_import.cmake ./

* Get toolchain
brew install --cask gcc-arm-embedded
export PICO_TOOLCHAIN_PATH=/toolchain/path

* Install picotool (avoid building from source)
brew install picotool

* Compile
rm -rf CMakeFiles/ && rm -rf generated/ && rm CMakeCache.txt && rm -rf .cmake && rm firmware.uf2
cmake . && make && python3 prepare.py

After build the firmware, you could code like usual on clion.
