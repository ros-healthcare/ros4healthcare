import crcmod
from enum import Enum, IntEnum

check_crc = crcmod.mkCrcFun(0x107, 0xff, False, 0)

class FileNames(Enum):
    PPG_FILE               = 1
    Activity_file          = 2
    Hearbeat_HRV           = 3
    Workout                = 4
    Sleep                  = 5
    Debug_Logs             = 6
    GPS                    = 8
    PPG2                   = 9
    GPS_ASSIST_ONLINE      = 10
    GPS_ASSIST_OFFLINE     = 11
    GPS_DEBUG              = 12
    ECG_file               = 13
    BioZ_file              = 14
    WORKOUT_SUMMARIES_FILE = 15
    USER_INPUT_EVENTS_FILE = 16 # such as hydration
    ANCS_BUNDLE_IDS_FILE   = 17
    GREENTEG_FILE          = 18
    LOG_FILE_BACKUP        = 19
    Philips_OUTPUTS_DEBUG  = 20
    STRESS_FILE            = 21
    ACC_FILE               = 22

class ModeID(Enum):
    MODE_SHIPPING          = 0
    POWER_SAVING_MODE      = 1
    MODE_HRM               = 2
    MODE_SLEEP_TRACKING    = 3
    MODE_PREVENTICUS       = 4
    MODE_PREVENTICUS_SLEEP = 5
    MODE_WORKOUT           = 6
    MODE_NORMAL            = 7
    MODE_CHARGING          = 8
    MODE_MAX_BATTERY       = 9

class CHG_STATUS(Enum):
    NOT_IN_CHARGER_DISCHARGING = 0
    IN_CHARGER_FULLY_CHARGED   = 1
    IN_CHARGER_CHARGING        = 3
    NOT_IN_CHARGER_CHARGING    = 4


class PLAN(IntEnum):
    STAND_BY = 0
    MAX_BATTERY = 1
    TYPICAL = 2
    HOSPITAL = 3
    HZ = 4
    HOSPITAL_RAW = 5
    RESERVED = 6
    TYPICAL_MULTICOLOR = 7
    HOSPITAL_MULTICOLOR = 8
    CONT_GREEN_INT_MULTICOLOR_RAW = 9
    CONT_GREEN_INT_MULTICOLOR = 10

class PLAN_FREQUENCY(Enum):
    FREQ_32HZ  = 1
    FREQ_64HZ  = 2
    FREQ_128HZ = 3
    FREQ_256HZ = 4
    FREQ_512HZ = 5

class VitalParameterWatchType(Enum):
    """Which vital parameter a SET/GET_VITAL_PARAM command addresses.

    Values taken from the Corsano BLE SDK (VitalParameterWatchType.watchValue,
    com.corsano.sdk.ble.data), cross-checked on the wire: the official app's
    capture shows GET_VITAL_PARAM (0xC1) issued for 0x01 and 0x02.
    """
    UNKNOWN         = 0
    BIOZ            = 1
    ACCELEROMETER   = 2
    TEMPERATURE_RAW = 4


class VitalParameterValue(Enum):
    """Monitoring mode for a vital parameter (SDK: VitalParameterValue)."""
    UNKNOWN          = 0
    DISABLE          = 1
    BEDTIME_RISETIME = 2
    CONTINUOUS       = 3


class SPECIAL_MODES(Enum):
    STRAP_OPTIMIZATION = 1

class ACTIVITY_TYPE(Enum):
    UNSPECIFIED = 0
    OTHER       = 1
    WALK        = 2
    RUN         = 4
    CYCLE       = 6
    REST        = 7
