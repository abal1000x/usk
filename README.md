# μsk

Build the firmware
------------------
rm -rf CMakeFiles/ && rm -rf generated/ && rm CMakeCache.txt && rm -rf .cmake && rm firmware.uf2
cmake . && make && python3 prepare.py

After build the firmware, you could code like usual on clion.