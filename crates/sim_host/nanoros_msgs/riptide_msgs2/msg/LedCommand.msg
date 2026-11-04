# color info
uint8 red
uint8 green
uint8 blue

# state info
uint8 mode
uint8 target

# modes
uint8 MODE_SOLID = 0 
uint8 MODE_SLOW_FLASH = 1
uint8 MODE_FAST_FLASH = 2 
uint8 MODE_BREATH = 3
uint8 SINGLETON_FLASH = 4

# targets
uint8 TARGET_NONE = 0
uint8 TARGET_CCB = 1
uint8 TARGET_ALU = 2
uint8 TARGET_ALL = 3