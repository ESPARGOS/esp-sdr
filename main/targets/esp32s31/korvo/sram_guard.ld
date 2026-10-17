ASSERT(_iram_end <= 0x2f060000, "S31 RF dump overlaps IRAM")
ASSERT(_data_end <= 0x2f060000, "S31 RF dump overlaps data")
ASSERT(_bss_end <= 0x2f060000, "S31 RF dump overlaps BSS")
