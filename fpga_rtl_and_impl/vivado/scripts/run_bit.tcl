# Bitstream generation script for Octa-Core RV64IMAFDCV Space Processor V7 (VU9P)

set part [lindex $argv 0]
set top [lindex $argv 1]

puts "============================================"
puts " Bitstream: RV64IMAFDCV Octa-Core SoC V7"
puts " Part: $part  (VU9P UltraScale+)"
puts " Top: $top"
puts "============================================"

# Open implementation checkpoint
open_checkpoint build/${top}_impl.dcp

# Enable bitstream-level SEU mitigation for radiation tolerance
set_property BITSTREAM.SEU.ESSENTIALBITS YES [current_design]
set_property BITSTREAM.GENERAL.COMPRESS FALSE [current_design]
set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP [current_design]

# Generate bitstream
write_bitstream -force build/${top}.bit

# Generate essential bits file for SEU scrubbing
write_cfgmem -force -format BIN -interface SPIx4 -size 256 \
    -loadbit "up 0x0 build/${top}.bit" build/${top}.bin

puts "Bitstream generated: build/${top}.bit"
