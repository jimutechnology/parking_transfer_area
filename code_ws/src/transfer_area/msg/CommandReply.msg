uint8 STATE_EMPTY    = 0
uint8 STATE_READLY   = 1
uint8 STATE_SENDED   = 2
uint8 STATE_RUNNING  = 3
uint8 STATE_PREFETCH = 4
uint8 STATE_FINISH   = 5
uint8 STATE_ERROR    = 6
uint8 STATE_TIMEOUT  = 7

string command_id
uint8 state
uint8  error_code
string message