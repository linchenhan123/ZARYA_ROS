# For Automatic control mission
# The message used by mission 1 action

# The current stage in mission
uint8 state

# The number_to_punch
uint8 target_num

# Stage constant
uint8 STATE_STOP                    = 0
uint8 STATE_GO_TO_READ_NUM_POSITION = 1
uint8 STATE_PARSEING_NUM            = 2
uint8 STATE_GO_TO_PREPOSITION       = 3
uint8 STATE_GO_TO_FRONT             = 4
uint8 STATE_PUNCHING_NUM            = 5
uint8 STATE_GO_BACK_TO_LINE         = 6
