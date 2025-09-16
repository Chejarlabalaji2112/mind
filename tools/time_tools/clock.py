import datetime
import pytz

class Clock:
    @staticmethod
    def get_current_utc_datetime():
        """Returns the current UTC datetime object."""
        return datetime.datetime.now(pytz.utc)

    @staticmethod
    def get_current_local_datetime():
        """Returns the current datetime object in the system's local timezone."""
        return datetime.datetime.now().astimezone()

    @staticmethod
    def get_current_datetime(timezone_str=None):
        """
        Returns the current datetime object, optionally localized to a specific timezone.
        If no timezone_str is provided, returns UTC datetime.
        """
        if timezone_str:
            try:
                tz = pytz.timezone(timezone_str)
                return datetime.datetime.now(pytz.utc).astimezone(tz)
            except pytz.exceptions.UnknownTimeZoneError:
                print(f"Warning: Unknown timezone '{timezone_str}'. Returning UTC datetime.")
                return Clock.get_current_utc_datetime()
        return Clock.get_current_utc_datetime()

    @staticmethod
    def get_current_time_str(timezone_str=None, format_str="%H:%M:%S"):
        """Returns the current time as a formatted string, optionally in a specific timezone."""
        dt = Clock.get_current_datetime(timezone_str)
        return dt.strftime(format_str)

    @staticmethod
    def get_current_date_str(timezone_str=None, format_str="%Y-%m-%d"):
        """Returns the current date as a formatted string, optionally in a specific timezone."""
        dt = Clock.get_current_datetime(timezone_str)
        return dt.strftime(format_str)

    @staticmethod
    def get_current_datetime_str(timezone_str=None, format_str="%d-%m, %a| %H:%M:%S"):
        """Returns the current datetime as a formatted string, optionally in a specific timezone."""
        dt = Clock.get_current_datetime(timezone_str)
        return dt.strftime(format_str)
