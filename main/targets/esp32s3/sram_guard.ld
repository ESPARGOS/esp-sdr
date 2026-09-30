ASSERT(_iram_end <= 0x403a0000, "S3 RF ring overlaps IRAM")
ASSERT(_data_end <= 0x3fcb0000, "S3 RF ring overlaps data")
ASSERT(_bss_end <= 0x3fcb0000, "S3 RF ring overlaps BSS")
