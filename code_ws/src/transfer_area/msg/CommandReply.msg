uint16 STATE_EMPTY    = 0
uint16 STATE_READLY   = 1
uint16 STATE_SENDED   = 2
uint16 STATE_RUNNING  = 3
uint16 STATE_PREFETCH = 4
uint16 STATE_FINISH   = 5
uint16 STATE_ERROR    = 6
uint16 STATE_TIMEOUT  = 7

string command_id
uint8 state
uint16 error_code
string message