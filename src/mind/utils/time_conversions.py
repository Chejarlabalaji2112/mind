import re


def convert_to_seconds(hours=0, minutes=0, seconds=0):
    """
    Converts hours, minutes, and seconds into a total duration in seconds.

    Args:
        hours (int/float): Number of hours. Defaults to 0.
        minutes (int/float): Number of minutes. Defaults to 0.
        seconds (int/float): Number of seconds. Defaults to 0.

    Returns:
        int/float: The total duration in seconds.

    Raises:
        ValueError: If any input is negative.
    """
    if any(val < 0 for val in [hours, minutes, seconds]):
        raise ValueError("Time components cannot be negative.")
    return (hours * 3600) + (minutes * 60) + seconds

def format_seconds_to_hms(total_seconds):
    """
    Converts a total number of seconds into a human-readable HH:MM:SS string.
    """
    if total_seconds < 0:
        return "-Invalid Time-"
    
    # Ensure total_seconds is an integer for consistent calculation
    total_seconds = int(total_seconds)

    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return f"{hours:02}:{minutes:02}:{seconds:02}"

def parse_time_string(time_str: str) -> int:
    """
    Parses a string representing time (e.g., '1h 30m', '20m', '45s') into seconds.
    """
    if not time_str:
        return 0
    
    time_str = time_str.lower().replace(" ", "")
    
    # Check for simple number (assume seconds)
    if time_str.isdigit():
        return int(time_str)

    hours = 0
    minutes = 0
    seconds = 0

    # Extract hours
    h_match = re.search(r'(\d+)h', time_str)
    if h_match:
        hours = int(h_match.group(1))

    # Extract minutes
    m_match = re.search(r'(\d+)m', time_str)
    if m_match:
        minutes = int(m_match.group(1))

    # Extract seconds
    s_match = re.search(r'(\d+)s', time_str)
    if s_match:
        seconds = int(s_match.group(1))

    return convert_to_seconds(hours, minutes, seconds)